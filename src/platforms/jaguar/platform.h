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
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <stdint.h>
#include "em_gpio.h"

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#include <setjmp.h>
#include <alloca.h>

#include "gdb_packet.h"

#define PLATFORM_HAS_POWERCONTROL 1

#define BOARD_IDENT "HiKoB Mystic Jaguar"

/* Hardware definitions... */
#define TDI_PORT	gpioPortB
#define TDI_PIN		6
#define TMS_PORT	gpioPortA
#define TMS_PIN		12
#define TCK_PORT	gpioPortC
#define TCK_PIN		4
#define TDO_PORT	gpioPortC
#define TDO_PIN		5

#define SWDIO_PORT 	TMS_PORT
#define SWCLK_PORT 	TCK_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	gpioPortA
#define TRST_PIN	13
#define SRST_PORT	gpioPortB
#define SRST_PIN	5

#define LED_PORT	gpioPortE
#define LED_IDLE_PORT	gpioPortE
#define LED_IDLE_RUN	9
#define LED_CUSTOM_PORT	gpioPortE
#define LED_CUSTOM_RUN	15

#define ACTIVITY_LED    LED_IDLE_PORT,LED_IDLE_RUN
#define CUSTOM_LED      LED_CUSTOM_PORT,LED_CUSTOM_RUN

#define TMS_SET_MODE()                                          \
    GPIO_PinModeSet(TMS_PORT, TMS_PIN, gpioModePushPull,  0);
#define SWDIO_MODE_FLOAT()                              \
    GPIO_PinModeSet(SWDIO_PORT, SWDIO_PIN, gpioModeInputPull,  0);
#define SWDIO_MODE_DRIVE()                                              \
    GPIO_PinModeSet(SWDIO_PORT, SWDIO_PIN, gpioModePushPull,  0);

#define SRST_SET_VAL(x)

extern uint8_t running_status;
extern volatile uint32_t timeout_counter;

extern jmp_buf fatal_error_jmpbuf;

extern const char *morse_msg;

#define gpio_set(port, pin) GPIO_PinOutSet((port), (pin))
#define gpio_clear(port, pin) GPIO_PinOutClear((port), (pin))
#define gpio_get(port, pin) GPIO_PinInGet((port), (pin))

#define gpio_set_val(port, pin, val) do {	\
	if(val) {					\
	    gpio_set((port), (pin)); \
	} else {					\
        gpio_clear((port), (pin)); \
	} \
} while(0)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT, LED_ERROR, state);}

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()	{setjmp(fatal_error_jmpbuf);}
#define PLATFORM_FATAL_ERROR(error)	do { 		\
	if(running_status) gdb_putpacketz("X1D");	\
		else gdb_putpacketz("EFF");		\
	running_status = 0;				\
	target_list_free();				\
	longjmp(fatal_error_jmpbuf, (error));		\
} while (0)

void morse(const char *msg, char repeat);

int platform_init(void);

/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

const char *platform_target_voltage(void);
void platform_delay(uint32_t delay);

#include <stdio.h>

void uart_write(const char* msg);
extern char print_buf[1024];
#define uart_printf(...) do { sprintf(print_buf, __VA_ARGS__); uart_write(print_buf);} while (0)

#define DEBUG(...)
//uart_printf(__VA_ARGS__)

#endif
