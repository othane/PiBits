/**
 * file: cbman.c
 * I have split the managment of the cbs into this file so they can be unit tested
 * in user space since they get a little tricky to manage and debug in the kernel.
 */

#ifdef TEST
#include "cbman_test.h"
#endif

extern uint8_t servo2gpio[];
extern struct ctldata_s *ctl;
extern int tick_scale;

#define PERIOD(x) (x / (tick_scale * 1000/ 600)) // enter period in us (= T0 / (tick_scale / 600khz))
#define CBW_INDEX() ((ctl->cbw == ctl->cbbufs[0]) ? 0: 1)
#define IN(x, y, size) ((uint32_t)x >= (uint32_t)y && (uint32_t)x < (uint32_t)y + size)
#define SERVOBIT(servo) (1 << servo2gpio[servo])
#define SRC2BITMASK(src) (*(uint32_t *)src)
#define IS_CLR_CB(x) (x->dst == (((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000))
#define IS_SET_CB(x) (x->dst == (((GPIO_BASE + GPSET0*4) & 0x00ffffff) | 0x7e000000))
#define IS_DELAY_CB(x) (x->dst == (((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000))
#define IS_CB_CHANGING_SERVO_STATE(cb, servo) ((SRC2BITMASK(_cb->src) & SERVOBIT(servo)) && (IS_CLR_CB(cb) || IS_SET_CB(cb)))

static struct bcm2708_dma_cb * get_free_servo_cb(struct bcm2708_dma_cb *cbstart)
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

static int find_servo_lo_cb(struct bcm2708_dma_cb *cbstart, int servo, struct bcm2708_dma_cb **cb, struct bcm2708_dma_cb **cblast)
{
	struct bcm2708_dma_cb *_cb, *last;

	// walk the link list looking for the servo entry to set output lo
	_cb = cbstart + 2;	//start after the initial state blocks
	last = cbstart + 1;
	for (;;) {
		if (IS_CLR_CB(_cb) && IS_CB_CHANGING_SERVO_STATE(_cb, servo))
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

static int add_servo_lo_cb(struct bcm2708_dma_cb *cbstart, int servo, int ticks)
{
	struct bcm2708_dma_cb *cbprev, *cbnew;
	int elapsed = 0, tnext;

	// find where to add the servo in the link list
	cbprev = cbstart + 2;
	for (;;) {
		// if this is a delay step then sum ticks to find the elapsed time
		if (IS_DELAY_CB(cbprev)) {
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
	*((uint32_t *)cbstart[1].src) |= SERVOBIT(servo);
	if (ticks != elapsed) {
		// get new cb to link in
		cbnew = get_free_servo_cb(cbstart);
		if (cbnew == NULL)
			return -1;

		ctl->gpiolo[CBW_INDEX()][servo] = SERVOBIT(servo);
		cbnew[0].src 	= (uint32_t)&ctl->gpiolo[CBW_INDEX()][servo];
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
		*((uint32_t *)cbnew->src) |= SERVOBIT(servo);
	}
}

static int remove_servo_lo_cb(struct bcm2708_dma_cb *cbstart, int servo)
{
	struct bcm2708_dma_cb *cb, *cblast;

	// find servo lo cb
	if (find_servo_lo_cb(cbstart, servo, &cb, &cblast) < 0)
		return -1;

	// remove this servos lo cb
	if (*(uint32_t *)(cb->src) & ~SERVOBIT(servo)) {
		// this cb is used for several servos so just remove this servos entry
		*(uint32_t *)(cb->src) &= ~SERVOBIT(servo);
	}
	else {
		// this cb is used only for this servo so just free it
		cblast->next = ((struct bcm2708_dma_cb *)cb->next)->next;
		memset((void *)cb, 0x00, 2*sizeof(struct bcm2708_dma_cb));
	}

	return 0;
}

int copy_a2w(void)
{
	int ia, iw, offset;
	struct bcm2708_dma_cb *cb;

	// copy the active dma cb's into the working cb's
	memcpy(ctl->cbw, ctl->cba, sizeof(struct bcm2708_dma_cb) * (3 + 2*NUM_SERVOS));
	offset = ctl->cbw - ctl->cba;

	// copy the active gpio states into the working cb's
	iw = CBW_INDEX();
	ia = (iw == 0)? 1: 0;
	memcpy(ctl->gpioinit[iw], ctl->gpioinit[ia], sizeof(uint32_t) * 2);
	memcpy(ctl->gpiolo[iw], ctl->gpiolo[ia], sizeof(uint32_t) * NUM_SERVOS);
	
	// update cb pointers with src as gpio to point to the copies
	cb = ctl->cbw;
	for (;;)
	{
		// update cb next to point in to cbw not cba
		if (cb->next != (int)NULL)
			cb->next += offset * sizeof(struct bcm2708_dma_cb);
		
		// if this cb points to gpioinit or gpiolo it is for the active buffer
		// so update it to the working buffer
		if (IN(cb->src, ctl->gpioinit, 4*sizeof(uint32_t)))
			cb->src = (uint32_t)&ctl->gpioinit[iw] + (cb->src - (uint32_t)ctl->gpioinit[ia]);
		if (IN(cb->src, ctl->gpiolo, 2*NUM_SERVOS*sizeof(uint32_t)))
			cb->src = (uint32_t)&ctl->gpiolo[iw] + (cb->src - (uint32_t)ctl->gpiolo[ia]);;

		if (cb->next == (uint32_t)ctl->cbw)
			// end of list
			break;

		cb = (struct bcm2708_dma_cb *)cb->next;
	}
}

int roll_a2w(void)
{
	struct bcm2708_dma_cb *cb, *_cb;
	cb = ctl->cba;

	// find the end of the current sequence
	for (;;)
	{
		if ((struct bcm2708_dma_cb *)cb->next == ctl->cba)
			break;
		cb = (struct bcm2708_dma_cb *)cb->next;
	}

	//@todo wait until it is safe to change the end of the sequence
	
	// roll the end of the active sequence into the working sequence
	cb->next = (int)ctl->cbw;
	//@todo wait until the new sequence is active

	// swap the active sequence with the working sequence
	_cb = ctl->cbw;
	ctl->cbw = ctl->cba;
	ctl->cba = _cb;

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
	copy_a2w();

	// update double buffer to the desired sequence 
	if (ticks == 0) {	
		// 0% duty cycle
		remove_servo_lo_cb(ctl->cbw, servo);
		// set the servo to never go hi and stay lo forever
		*(uint32_t *)(ctl->cbw[0].src) |= SERVOBIT(servo);	// set lo
		*(uint32_t *)(ctl->cbw[1].src) &= ~SERVOBIT(servo);	// clear hi
	} else if (ticks == ctl->period) {	
		// 100% duty cycle
		remove_servo_lo_cb(ctl->cbw, servo);
		// set the servo to go hi and never lo
		*(uint32_t *)(ctl->cbw[0].src) &=  ~SERVOBIT(servo);	// clear lo
		*(uint32_t *)(ctl->cbw[1].src) |=  SERVOBIT(servo);	// set hi
	} else {
		// other duty cycles
		remove_servo_lo_cb(ctl->cbw, servo);
		// find the new servo position and insert it
		add_servo_lo_cb(ctl->cbw, servo, ticks);
	}

	roll_a2w();

	//@todo wait until it is safe to swap the buffers
	
	return 0;
}

void init_servos(void)
{
	int s;
	uint32_t pinmask;

	// zero ctl structs and setup the active and working double buffer pointers
	memset(ctl, 0x00, sizeof(struct ctldata_s));
	ctl->cba = ctl->cbbufs[0];
	ctl->cbw = ctl->cbbufs[1];
	ctl->period = PERIOD(500);
	pinmask = 0;
	for (s = 0; s < NUM_SERVOS; s++)
		pinmask |= SERVOBIT(s);
	
	// initially set all servos to 0% duty
	ctl->gpioinit[0][0]  = pinmask;
	ctl->cba[0].src    = (uint32_t)(&ctl->gpioinit[0][0]) & 0x7fffffff;
	ctl->cba[0].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
	ctl->cba[0].dst    = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
	ctl->cba[0].length = sizeof(uint32_t);
	ctl->cba[0].stride = 0;
	ctl->cba[0].next   = (uint32_t)&ctl->cba[1] & 0x7fffffff;
	// hence also no servos set hi initially
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
}

