/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/f1/adc.h>

#include "platform.h"
#include "jtag_scan.h"
#include <usbuart.h>

#include <ctype.h>


uint8_t running_status;
volatile uint32_t timeout_counter;

jmp_buf fatal_error_jmpbuf;

static void morse_update(void);


int platform_init(void)
{

	rcc_clock_setup_in_hse_16mhz_out_72mhz();
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);
	/* Setup heartbeat timer */
	SCB_SHPR(11) &= ~((15 << 4) & 0xff);
	SCB_SHPR(11) |= ((14 << 4) & 0xff);

	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);


	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_CRCEN);

	/* Setup GPIO ports */
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
	gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
	gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TDI_PIN);


	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_UART | LED_IDLE_RUN);

	gpio_clear(LED_PORT, LED_UART | LED_IDLE_RUN);


	systick_interrupt_enable();
	systick_counter_enable();

	// Set USB pull-UP down
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	gpio_clear(USB_PU_PORT, USB_PU_PIN);

	gpio_set(USB_PU_PORT, USB_PU_PIN);

	usbuart_init();

//	SCB_VTOR = 0x2000;	// Relocate interrupt vector table here

	cdcacm_init();

	jtag_scan(NULL);

	return 0;
}

void platform_delay(uint32_t delay)
{
	timeout_counter = delay;
	while(timeout_counter);
}

void sys_tick_handler(void)
{
	if(running_status)
		gpio_toggle(LED_PORT, LED_IDLE_RUN);

	if(timeout_counter)
		timeout_counter--;

	morse_update();
}


/* Morse code patterns and lengths */
static const struct {
	uint16_t code;
	uint8_t bits;
} morse_letter[] = {
	{        0b00011101,  8}, // 'A' .-
	{    0b000101010111, 12}, // 'B' -...
	{  0b00010111010111, 14}, // 'C' -.-.
	{      0b0001010111, 10}, // 'D' -..
	{            0b0001,  4}, // 'E' .
	{    0b000101110101, 12}, // 'F' ..-.
	{    0b000101110111, 12}, // 'G' --.
	{      0b0001010101, 10}, // 'H' ....
	{          0b000101,  6}, // 'I' ..
	{0b0001110111011101, 16}, // 'J' .---
	{    0b000111010111, 12}, // 'K' -.-
	{    0b000101011101, 12}, // 'L' .-..
	{      0b0001110111, 10}, // 'M' --
	{        0b00010111,  8}, // 'N' -.
	{  0b00011101110111, 14}, // 'O' ---
	{  0b00010111011101, 14}, // 'P' .--.
	{0b0001110101110111, 16}, // 'Q' --.-
	{      0b0001011101, 10}, // 'R' .-.
	{        0b00010101,  8}, // 'S' ...
	{          0b000111,  6}, // 'T' -
	{      0b0001110101, 10}, // 'U' ..-
	{    0b000111010101, 12}, // 'V' ...-
	{    0b000111011101, 12}, // 'W' .--
	{  0b00011101010111, 14}, // 'X' -..-
	{0b0001110111010111, 16}, // 'Y' -.--
	{  0b00010101110111, 14}, // 'Z' --..
};


const char *morse_msg;
static const char * volatile morse_ptr;
static char morse_repeat;

void morse(const char *msg, char repeat)
{
	morse_msg = morse_ptr = msg;
	morse_repeat = repeat;
	SET_ERROR_STATE(0);
}

static void morse_update(void)
{
	static uint16_t code;
	static uint8_t bits;

	if(!morse_ptr) return;

	if(!bits) {
		char c = *morse_ptr++;
		if(!c) {
			if(morse_repeat) {
				morse_ptr = morse_msg;
				c = *morse_ptr++;
			} else {
				morse_ptr = 0;
				return;
			}
		}
		if((c >= 'A') && (c <= 'Z')) {
			c -= 'A';
			code = morse_letter[c].code;
			bits = morse_letter[c].bits;
		} else {
			code = 0; bits = 4;
		}
	}
	SET_ERROR_STATE(code & 1);
	code >>= 1; bits--;
}


const char *platform_target_voltage(void)
{
	static char ret[] = "unknown";
	// TODO

	return ret;
}


const char *platform_name(void)
{
	static char ret[] = "HiKoB Fox";
	return ret;
}


void assert_boot_pin(void)
{
	uint32_t crl = GPIOA_CRL;
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	/* Enable Pull on GPIOA1. We don't rely on the external pin
	 * really pulled, but only on the value of the CNF register
	 * changed from the reset value
	 */
	crl &= 0xffffff0f;
	crl |= 0x80;
	GPIOA_CRL = crl;
}

void setup_vbus_irq(void){}
