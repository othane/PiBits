/**
 * file: cbman.c
 * I have split the managment of the cb into this file so they can be unit tested
 * in user space since they get a little tricky to manage and debug in the kernel.
 *
 * ATM this is a big mess with the utest included. I will tidy this up into just a 
 * lib for use by the tests and driver.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// dump this in here so we can play in user land first
struct bcm2708_dma_cb {
	unsigned long info;
	unsigned long src;
	unsigned long dst;
	unsigned long length;
	unsigned long stride;
	unsigned long next;
	unsigned long pad[2];
}__attribute__((packed));

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO */
#define BCM2708_DMA_WAIT_RESP	(1 << 3)
#define BCM2708_DMA_D_DREQ	(1 << 6)
#define	BCM2708_DMA_PER_MAP(x)	((x) << 16)
#define PWM_BASE		(BCM2708_PERI_BASE + 0x20C000)
#define GPSET0			(0x1c/4)
#define GPCLR0			(0x28/4)
#define PWM_FIFO		(0x18/4)
#define BCM2708_DMA_NO_WIDE_BURSTS	(1<<26)

// Map servo channels to GPIO pins
static uint8_t servo2gpio[] = {
		4,	// P1-7
		17,	// P1-11
		1,	// P1-5 (GPIO-18, P1-12 is currently PWM0, for debug)
		21,	// P1-13
		22,	// P1-15
		23,	// P1-16
		24,	// P1-18
		25,	// P1-22
};

#define NUM_SERVOS	(sizeof(servo2gpio)/sizeof(servo2gpio[0]))

struct ctldata_s {
	struct bcm2708_dma_cb cbbufs[2][3 + 2*NUM_SERVOS];	// (gpio-lo, gpio-hi, delay, (gpio-lo, delay) x NUM_SERVOS), x 2 for double buffer
	struct bcm2708_dma_cb *cba, *cbw;					// point to active cbbuf and working cbbuf
	uint32_t gpioinit[2][2];							// initial clear and set values, x 2 for double buffer
	uint32_t gpiolo[2][NUM_SERVOS];						// clear-pin values, per servo output, x 2 for double buffer
	uint32_t pwmdata;									// the word we write to the pwm fifo
	uint32_t period;									// total cycle time (in ticks ie 10us counts), this is const for all servos
};

// Instances
static struct ctldata_s _ctl;
static struct ctldata_s *ctl = &_ctl;
static int tick_scale = 6;

// enter period in us (= T0 / (tick_scale / 600khz))
#define PERIOD(x) (x / (tick_scale * 1000/ 600))

#define cbw_index() ((ctl->cbw == ctl->cbbufs[0]) ? 0: 1)
#define in(x, y, size) ((uint32_t)x >= (uint32_t)y && (uint32_t)x < (uint32_t)y + size)

struct bcm2708_dma_cb * get_free_servo_cb(struct bcm2708_dma_cb *cbstart)
{
	int s;

	for (s = 3; s < 3 + 2*NUM_SERVOS; s+=2)
	{
		// if dst and src for both the lo and delay stages are null this is free
		if (cbstart[s].dst == 0 && cbstart[s].src == 0 && cbstart[s+1].dst == 0 && cbstart[s+1].src == 0)
			return &cbstart[s];
	}

	// no free servo (should not happen!)
	return NULL;
}

int add_servo_lo_cb(struct bcm2708_dma_cb *cbstart, int servo, int ticks)
{
	struct bcm2708_dma_cb *cbprev, *cbnew;
	int elapsed = 0, tnext;

	// find where to add the servo in the link list
	cbprev = cbstart + 2;
	for (;;) {
		// if this is a delay step then sum ticks to find the elapsed time
		if (cbprev->dst == (((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000)) {
			elapsed += cbprev->length;
			if (elapsed >= ticks)
				break;
		}

		if (cbprev->next == 0 || cbprev->next == (int)cbstart) 
			// got to the end and ticks still not up (ticks > period, should not happen)
			return -1;

		cbprev = (struct bcm2708_dma_cb *)cbprev->next;
	}
	elapsed -= cbprev->length;

	// add servo in
	*((uint32_t *)cbstart[1].src) |= 1 << servo2gpio[servo];
	if (ticks != elapsed) {
		// get new cb to link in
		cbnew = get_free_servo_cb(cbstart);
		if (cbnew == NULL)
			return -1;

		ctl->gpiolo[cbw_index()][servo] = 1 << servo2gpio[servo];
		cbnew[0].src 	= (uint32_t)&ctl->gpiolo[cbw_index()][servo];
		cbnew[0].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
		cbnew[0].dst    = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
		cbnew[0].length = sizeof(uint32_t);
		cbnew[0].stride = 0;
		cbnew[0].next 	= (uint32_t)&cbnew[1] & 0x7fffffff;
		
		cbnew[1].info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
		cbnew[1].src    = (uint32_t)(&ctl->pwmdata) & 0x7fffffff;
		cbnew[1].dst    = ((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000;
		cbnew[1].length = elapsed + cbprev->length - ticks;
		cbnew[1].stride = 0;
		cbnew[1].next = cbprev->next & 0x7fffffff;
		
		cbprev->length = ticks - elapsed;
		cbprev->next = (int)cbnew;

	} else {

		// another servo is already doing lo here so just add to the lo block
		cbnew = (struct bcm2708_dma_cb *)cbprev->next;
		*((uint32_t *)cbnew->src) |= 1 << servo2gpio[servo];

	}
}

int find_servo_lo_cb(struct bcm2708_dma_cb *cbstart, int servo, struct bcm2708_dma_cb **cb, struct bcm2708_dma_cb **cblast)
{
	struct bcm2708_dma_cb *_cb, *last;

	// walk the link list looking for the servo entry to set output lo
	_cb = cbstart + 2;
	last = cbstart + 1;
	for (;;) {
		if ((_cb->dst == (((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000)) && (*(uint32_t *)(_cb->src) & (1 << servo2gpio[servo])))
			// found a gpio set lo for this servo
			break;

		if (_cb->next == 0 || _cb->next == (unsigned long)cbstart)
			// cannot find gpio set lo for this servo
			return -1;

		last = _cb;
		_cb = (struct bcm2708_dma_cb *)_cb->next;
	}

	// found so return details
	if (cb != NULL)
		*cb = _cb;
	if (cblast != NULL)
		*cblast = last;
	return 0;
}

int remove_servo_lo_cb(struct bcm2708_dma_cb *cbstart, int servo)
{
	struct bcm2708_dma_cb *cb, *cblast;

	// find servo lo cb
	if (find_servo_lo_cb(cbstart, servo, &cb, &cblast) < 0)
		return -1;

	// remove this servos lo cb
	if (*(uint32_t *)(cb->src) & ~(1 << servo2gpio[servo])) {
		// this cb is used for several servos so just remove this servos entry
		*(uint32_t *)(cb->src) &= ~(1 << servo2gpio[servo]);
	}
	else {
		// this cb is used only for this servo so just free it
		cblast->next = ((struct bcm2708_dma_cb *)cb->next)->next;
		memset((void *)cb, 0x00, 2*sizeof(struct bcm2708_dma_cb));
	}

	return 0;
}

int copy_a2w()
{
	int ia, iw, offset;
	struct bcm2708_dma_cb *cb;

	// copy the active dma cb's into the working cb's
	memcpy(ctl->cbw, ctl->cba, sizeof(struct bcm2708_dma_cb) * (3 + 2*NUM_SERVOS));
	offset = ctl->cbw - ctl->cba;

	// copy the active gpio states into the working cb's
	iw = cbw_index();
	ia = (iw == 0)? 1: 0;
	memcpy(ctl->gpioinit[iw], ctl->gpioinit[ia], sizeof(uint32_t) * 2);
	memcpy(ctl->gpiolo[iw], ctl->gpiolo[ia], sizeof(uint32_t) * NUM_SERVOS);
	
	// update cb pointers with src as gpio to point to the copies
	cb = ctl->cbw;
	for (;;)
	{
		// update cb next to cbw not cba
		if (cb->next != (int)NULL)
			cb->next += offset * sizeof(struct bcm2708_dma_cb);
		
		// if this cb points to gpioinit or gpiolo it is for the active buffer
		// so update it to the working buffer
		if (in(cb->src, ctl->gpioinit, 4*sizeof(uint32_t)))
			cb->src = (uint32_t)&ctl->gpioinit[iw] + (cb->src - (uint32_t)ctl->gpioinit[ia]);
		if (in(cb->src, ctl->gpiolo, 2*NUM_SERVOS*sizeof(uint32_t)))
			cb->src = (uint32_t)&ctl->gpiolo[iw] + (cb->src - (uint32_t)ctl->gpiolo[ia]);;

		if (cb->next == (uint32_t)ctl->cbw)
			// end of list
			break;

		cb = (struct bcm2708_dma_cb *)cb->next;
	}
}

int set_servo_duty(int servo, float duty)
{
	int ticks = ctl->period * duty;
	struct bcm2708_dma_cb *cb;

	// sanity checks
	if (ticks < 0 || ticks > ctl->period)
		return -1;
	if (servo > NUM_SERVOS)
		return -1;

	// copy the current active buffer info into the working buffers
	//copy_a2w();

	// update double buffer to the desired sequence 
	if (ticks == 0) {	
		// 0% duty cycle
		remove_servo_lo_cb(ctl->cbw, servo);
		// set the servo to never go hi and stay lo forever
		*(uint32_t *)(ctl->cbw[0].src) |=  (1 << servo2gpio[servo]);	// set lo
		*(uint32_t *)(ctl->cbw[1].src) &=  ~(1 << servo2gpio[servo]);	// clear hi
	} else if (ticks == ctl->period) {	
		// 100% duty cycle
		remove_servo_lo_cb(ctl->cbw, servo);
		// set the servo to go hi and never lo
		*(uint32_t *)(ctl->cbw[0].src) &=  ~(1 << servo2gpio[servo]);	// clear lo
		*(uint32_t *)(ctl->cbw[1].src) |=  (1 << servo2gpio[servo]);	// set hi
	} else {
		// other duty cycles
		remove_servo_lo_cb(ctl->cbw, servo);
		// find the new servo position and insert it
		add_servo_lo_cb(ctl->cbw, servo, ticks);
	}

	//@todo wait until it is safe to swap the buffers
	
	// swap the buffers by rolling the old list into the new one
	#if 0
	cb = ctl->cba;
	for (;;)
	{
		if ((struct bcm2708_dma_cb *)cb->next == ctl->cba)
			break;
		cb = (struct bcm2708_dma_cb *)cb->next;
	}
	cb->next = (int)ctl->cbw;
	//
	_cb = ctl->cbw;
	ctl->cbw = ctl->cba;
	ctl->cba = _cb;
	#endif

	//@todo wait until the new sequence is active
	
	return 0;
}

void print_sim()
{
	uint8_t gpiostates[NUM_SERVOS][1024*10];
	uint8_t gpiostate[NUM_SERVOS];
	int T = 0, t;
	struct bcm2708_dma_cb *cb = ctl->cba;
	int cbacount=0, cbwcount = 0;
	int s;

	// workout T (max ticks before the loop repeats itself)
	cb = ctl->cba;
	for (;;) {
		if (cb->dst ==  (((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000))
			T += cb->length;
		if (cb == ctl->cba) {
			cbwcount = 0;
			cbacount++;
			if (cbacount > 1)
				break;
		} else if (cb == ctl->cbw) {
			cbacount = 0;
			cbwcount++;
			if (cbwcount > 1)
				break;
		}
		cb = (struct bcm2708_dma_cb *)cb->next;
	}
	printf("max ticks = %d\n", T);
	printf("cba = %p; cbw = %p\n", ctl->cba, ctl->cbw);

	// populate the gpiostates buffer
	memset(gpiostate, 0, NUM_SERVOS);
	cb = ctl->cba;
	t = 0;
	while (t < T) {
		
		printf("===================\n");
		printf("next, cb=%p, cbnext=%p\n", cb, (struct bcm2708_dma_cb *)cb->next);

		// clear current gpio states
		if (cb->dst == (((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000)) {
			uint32_t mask = *(uint32_t *)cb->src;
			printf("cmask=%d, src=%p, gpiolo=%p, gpioinit=%p\n", (int)mask, (void *)cb->src, ctl->gpiolo, ctl->gpioinit);
			for (s=0; s < NUM_SERVOS; s++)
				if (mask & (1 << servo2gpio[s])) {
					printf("clearing states for servo %d\n", s);
					gpiostate[s] = 0;
				}
		}

		// set current gpio states
		if (cb->dst == (((GPIO_BASE + GPSET0*4) & 0x00ffffff) | 0x7e000000)) {
			uint32_t mask = *(uint32_t *)cb->src;
			printf("smask=%d\n", (int)mask);
			for (s=0; s < NUM_SERVOS; s++)
				if (mask & (1 << servo2gpio[s])) {
					printf("setting states for servo %d\n", s);
					gpiostate[s] = 1;
				}
		}

		// delay
		if (cb->dst == (((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000)) {
			int k;
			// flood fill gpiostate for this many ticks
			printf("doing delay of %d ticks\n", (int)cb->length);
			for (k = t; t < k + cb->length; t++) {
				for (s=0; s < NUM_SERVOS; s++)
					gpiostates[s][t] = gpiostate[s];
			}
		}
		
		// next
		cb = (struct bcm2708_dma_cb *)cb->next;
	}
	
	// print the gpiostates buffer
	for (s = 0; s < NUM_SERVOS; s++) {
		for (t = 0; t < T; t++) {
			if (gpiostates[s][t]) { 
				printf("^");
			} else {
				printf("_");
			}
		}
		printf("|\n");
	}
}

int main(int argc, void *argv)
{
	int k;
	uint32_t pinmask;
	struct bcm2708_dma_cb *cb;

	// init
	memset(ctl, 0x00, sizeof(struct ctldata_s));
	ctl->cba = ctl->cbbufs[0];
	ctl->cbw = ctl->cbbufs[1];
	ctl->period = PERIOD(500);
	pinmask = 0;
	for (k = 0; k < NUM_SERVOS; k++)
		pinmask |= (1 << servo2gpio[k]);

	// initially set all servos lo (this is for 0 duty cycle)
	ctl->gpioinit[0][0]  = pinmask;
	ctl->cba[0].src    = (uint32_t)(&ctl->gpioinit[0][0]) & 0x7fffffff;
	ctl->cba[0].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
	ctl->cba[0].dst    = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
	ctl->cba[0].length = sizeof(uint32_t);
	ctl->cba[0].stride = 0;
	ctl->cba[0].next   = (uint32_t)&ctl->cba[1] & 0x7fffffff;
	// initially set no servos 100% hi
	ctl->gpioinit[0][1]  = 0;
	ctl->cba[1].src    = (uint32_t)(&ctl->gpioinit[0][1]) & 0x7fffffff;
	ctl->cba[1].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
	ctl->cba[1].dst    = ((GPIO_BASE + GPSET0*4) & 0x00ffffff) | 0x7e000000;
	ctl->cba[1].length = sizeof(uint32_t);
	ctl->cba[1].stride = 0;
	ctl->cba[1].next   = (uint32_t)&ctl->cba[2] & 0x7fffffff;
	// delay until period is done then go back around (all duties 0%)
	ctl->cba[2].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
	ctl->cba[2].src    = (uint32_t)(&ctl->pwmdata) & 0x7fffffff;
	ctl->cba[2].dst    = ((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000;
	ctl->cba[2].length = ctl->period;	
	ctl->cba[2].stride = 0;
	ctl->cba[2].next = (uint32_t)&ctl->cba[0] & 0x7fffffff;

	// add in each servos initial state
	copy_a2w();

	set_servo_duty(0, 0.0);
	set_servo_duty(1, 0.2);
	set_servo_duty(2, 0.3);
	set_servo_duty(3, 0.4);
	set_servo_duty(4, 0.4);
	set_servo_duty(5, 0.5);
	set_servo_duty(6, 0.6);
	set_servo_duty(7, 1.0);

	cb = ctl->cba;
	for (;;)
	{
		if ((struct bcm2708_dma_cb *)cb->next == ctl->cba)
			break;
		cb = (struct bcm2708_dma_cb *)cb->next;
	}
	cb->next = (int)ctl->cbw;

	print_sim();
	
	return 0;
}
