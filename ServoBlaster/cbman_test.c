/**
 * file: cbman_test.c
 * Test the cbman unit extensively in user mode so we dont get fails in kernel mode (hopefully)
 */

#include <stdio.h>
#include <stdlib.h>
#include "cbman_test.h"


void init_servos(void);
int copy_a2w(void);
int set_servo_duty(int servo, float duty);
int roll_a2w(void);


// Map servo channels to GPIO pins
uint8_t servo2gpio[] = {
		4,	// P1-7
		17,	// P1-11
		1,	// P1-5 (GPIO-18, P1-12 is currently PWM0, for debug)
		21,	// P1-13
		22,	// P1-15
		23,	// P1-16
		24,	// P1-18
		25,	// P1-22
};

// Instances
struct ctldata_s _ctl;
struct ctldata_s *ctl = &_ctl;
int tick_scale = 6;

void print_sim()
{
	uint8_t gpiostates[NUM_SERVOS][1024*10];
	uint8_t gpiostate[NUM_SERVOS];
	int T = 0, t;
	struct bcm2708_dma_cb *cb = ctl->cba;
	int cbacount=0, cbwcount = 0;
	int s;

	printf("============= NEW SIM PRINTOUT =============\n");

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
	
	printf("============= SIM PRINTOUT DONE =============\n\n\n");
}

void up(void)
{
	set_servo_duty(0, 0.0);
	set_servo_duty(1, 0.2);
	set_servo_duty(2, 0.3);
	set_servo_duty(3, 0.4);
	set_servo_duty(4, 0.4);
	set_servo_duty(5, 0.5);
	set_servo_duty(6, 0.6);
	set_servo_duty(7, 1.0);
	print_sim();
}

void down(void)
{
	set_servo_duty(0, 1.0);
	set_servo_duty(1, 0.9);
	set_servo_duty(2, 0.8);
	set_servo_duty(3, 0.7);
	set_servo_duty(4, 0.6);
	set_servo_duty(5, 0.5);
	set_servo_duty(6, 0.4);
	set_servo_duty(7, 0.0);
	print_sim();
}

int main(int argc, void *argv)
{
	int k;
	struct bcm2708_dma_cb *cb;

	// init the servo list to all lo
	init_servos();
	print_sim();

	up();
	down();

	return 0;
}
