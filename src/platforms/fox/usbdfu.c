/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2013 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <stdio.h>
#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

#include "platform.h"
#include "usbdfu.h"

#define DEBUG(...)			//puts(__VA_ARGS__)

#define FORCE_DFU    GPIOC, GPIO13

void dfu_detach(void)
{
	/* USB device must detach, we just reset... */
	scb_reset_system();
}

int main(void)
{
	/* Check the force bootloader pin*/
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	if(gpio_get(FORCE_DFU)==0)
		dfu_jump_app_if_valid();

	dfu_protect_enable();

	rcc_clock_setup_in_hse_16mhz_out_72mhz();
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);

	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);

	// Configure LED
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10 | GPIO12);
	gpio_set(LED_PORT, LED_IDLE_RUN);

	systick_interrupt_enable();
	systick_counter_enable();

	// Set USB pull-UP down
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	gpio_clear(USB_PU_PORT, USB_PU_PIN);

	dfu_init(&stm32f103_usb_driver);

	gpio_set(USB_PU_PORT, USB_PU_PIN);

	dfu_main();

}

void sys_tick_handler(void)
{
	DEBUG("in sys_tick_handler ...\n");
	gpio_toggle(LED_PORT, LED_UART); /* LED2 on/off */
}
