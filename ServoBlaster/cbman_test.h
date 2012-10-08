/**
 * @file cbman_test.h
 *
 * This just basically copies structs and defines from the kernel
 * or driver so we can test it here.
 */

#ifndef CBMAN_TEST_H
#define CBMAN_TEST_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

// structs and defines pulled from the kernel to run the tests in user space
#define BCM2708_PERI_BASE			0x20000000
#define GPIO_BASE					(BCM2708_PERI_BASE + 0x200000) /* GPIO */
#define BCM2708_DMA_WAIT_RESP		(1 << 3)
#define BCM2708_DMA_D_DREQ			(1 << 6)
#define	BCM2708_DMA_PER_MAP(x)		((x) << 16)
#define PWM_BASE					(BCM2708_PERI_BASE + 0x20C000)
#define GPSET0						(0x1c/4)
#define GPCLR0						(0x28/4)
#define PWM_FIFO					(0x18/4)
#define BCM2708_DMA_NO_WIDE_BURSTS	(1<<26)

struct bcm2708_dma_cb {
	unsigned long info;
	unsigned long src;
	unsigned long dst;
	unsigned long length;
	unsigned long stride;
	unsigned long next;
	unsigned long pad[2];
}__attribute__((packed));


// other
#define NUM_SERVOS 8
struct ctldata_s {
	struct bcm2708_dma_cb cbbufs[2][3 + 2*NUM_SERVOS];	// (gpio-lo, gpio-hi, delay, (gpio-lo, delay) x NUM_SERVOS), x 2 for double buffer
	struct bcm2708_dma_cb *cba, *cbw;					// point to active cbbuf and working cbbuf
	uint32_t gpioinit[2][2];							// initial clear and set values, x 2 for double buffer
	uint32_t gpiolo[2][NUM_SERVOS];						// clear-pin values, per servo output, x 2 for double buffer
	uint32_t pwmdata;									// the word we write to the pwm fifo
	uint32_t period;									// total cycle time (in ticks ie 10us counts), this is const for all servos
};

#endif
