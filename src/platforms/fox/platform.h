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

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/usb/usbd.h>

#include <setjmp.h>
#include <alloca.h>

#include "gdb_packet.h"

#define INLINE_GPIO
#define CDCACM_PACKET_SIZE 	64
#define PLATFORM_HAS_TRACESWO
#define BOARD_IDENT            		"Black Magic Probe (HiKoB Fox), (Firmware 0.1" VERSION_SUFFIX ", build " BUILDDATE ")"
#define BOARD_IDENT_DFU		"Black Magic (Upgrade) for HiKoB Fox, (Firmware 0.1" VERSION_SUFFIX ", build " BUILDDATE ")"
#define DFU_IDENT						"Black Magic Firmware Upgrade (HiKoB Fox)"
#define DFU_IFACE_STRING			"@Internal Flash   /0x08000000/8*001Ka,56*001Kg"

extern usbd_device *usbdev;
#define CDCACM_GDB_ENDPOINT		1
#define CDCACM_UART_ENDPOINT		3


/* Important pin mappings for STM32 implementation:
0 2 1 3 4 5

 * SWD :
 	 *	VDD				=> 3.3V
 	 *	GND				=> GND
 	 *	TCK	 / SWCLK 	=> GPIO 0 (PA7)
 	 *	TMS	 / SWDIO 	=> GPIO 2 (PB0)
 	 *	NRST			=> GPIO 1 (PA6)
 	 *	TDO	 / SWO		=> GPIO 4 (PA4)
 	 * 	TDI				=> GPIO 3 (PC5)
 	 *
 	 * 	TRST			=> PA6
 	 * 	SRST			=> xxx
 * LEDs :
 	  	LED0 = 	PB10	=> UART
 	  	LED1 = 	PB12	=> IDLE
 * USB :
 	  	USB cable pull-up	:	PC7
 * DFU :
 	  	sForce DFU mode button: PC13

 */

/* Hardware definitions... */
#define TDI_PORT    GPIOC
#define TMS_PORT    GPIOB
#define TCK_PORT    GPIOA
#define TDO_PORT    GPIOA

#define TDI_PIN     GPIO5
#define TMS_PIN     GPIO0
#define TCK_PIN     GPIO7
#define TDO_PIN     GPIO4

#define SWDIO_PORT  TMS_PORT
#define SWCLK_PORT  TCK_PORT
#define SWDIO_PIN   TMS_PIN
#define SWCLK_PIN   TCK_PIN

#define TRST_PORT   GPIOA
#define TRST_PIN    GPIO6

/* SRST_PIN (system reset) not wired */

#define USB_PU_PORT GPIOC
#define USB_PU_PIN  GPIO7

#define LED_PORT      GPIOB
#define LED_PORT_UART GPIOB
#define LED_UART      GPIO10
#define LED_IDLE_RUN  GPIO12

#define TMS_SET_MODE()                                          \
    gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ,            \
                  GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
#define SWDIO_MODE_FLOAT()                              \
    gpio_set_mode(SWDIO_PORT, GPIO_MODE_INPUT,          \
                  GPIO_CNF_INPUT_FLOAT, SWDIO_PIN);
#define SWDIO_MODE_DRIVE()                                              \
    gpio_set_mode(SWDIO_PORT, GPIO_MODE_OUTPUT_50_MHZ,                  \
                  GPIO_CNF_OUTPUT_PUSHPULL, SWDIO_PIN);

#define UART_PIN_SETUP()                                                \
    gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_2_MHZ,                \
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN);

#define USB_DRIVER      stm32f103_usb_driver
#define USB_IRQ         NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR         usb_lp_can_rx0_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority. 
 */
#define IRQ_PRI_USB       (2   << 4)
#define IRQ_PRI_USBUSART  (1   << 4)
#define IRQ_PRI_USB_VBUS  (14 << 4)
#define IRQ_PRI_TRACE     (0   << 4)

#define USBUSART             USART2
#define USBUSART_CR1         USART2_CR1
#define USBUSART_IRQ         NVIC_USART2_IRQ
#define USBUSART_APB_ENR     RCC_APB1ENR
#define USBUSART_CLK_ENABLE  RCC_APB1ENR_USART2EN
#define USBUSART_PORT        GPIOA
#define USBUSART_TX_PIN      GPIO2
#define USBUSART_ISR         usart2_isr

#define TRACE_TIM TIM3
#define TRACE_TIM_CLK_EN() rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN)
#define TRACE_IRQ   NVIC_TIM3_IRQ
#define TRACE_ISR   tim3_isr

#define DEBUG(...)

extern uint8_t running_status;
extern volatile uint32_t timeout_counter;

extern jmp_buf fatal_error_jmpbuf;

extern const char *morse_msg;

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{DEBUG("error !!");}

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()	{setjmp(fatal_error_jmpbuf);}
#define PLATFORM_FATAL_ERROR(error)	{ 		\
	if(running_status) gdb_putpacketz("X1D");	\
		else gdb_putpacketz("EFF");		\
	running_status = 0;				\
	target_list_free();				\
	morse("TARGET LOST.", 1);			\
	longjmp(fatal_error_jmpbuf, (error));		\
}

int platform_init(void);
void morse(const char *msg, char repeat);
const char *platform_target_voltage(void);
int platform_hwversion(void);
void platform_delay(uint32_t delay);

/* <cdcacm.c> */
void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

/* <platform.h> */
void uart_usb_buf_drain(uint8_t ep);

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf

#ifdef INLINE_GPIO
static inline void _gpio_set(u32 gpioport, u16 gpios)
{
	GPIO_BSRR(gpioport) = gpios;
}
#define gpio_set _gpio_set

static inline void _gpio_clear(u32 gpioport, u16 gpios)
{
	GPIO_BRR(gpioport) = gpios;
}
#define gpio_clear _gpio_clear

static inline u16 _gpio_get(u32 gpioport, u16 gpios)
{
	return (u16)GPIO_IDR(gpioport) & gpios;
}
#define gpio_get _gpio_get
#endif

#endif

#define disconnect_usb() gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, 0, USB_PU_PIN);
void assert_boot_pin(void);
void setup_vbus_irq(void);
