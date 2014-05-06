/*
 * jaguar.h
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#ifndef JAGUAR_H_
#define JAGUAR_H_

#include "platform.h"

#define PLATFORM_HAS_POWERCONTROL 1

/* Important pin mappings for EnergyMicro implementation:
 * SWD :
     *  VDD             => 3.3V
     *  GND             => GND
     *  TMS  / SWDIO    => PA12
     *  TCK  / SWCLK    => PC4
     *  TDO             => PC5
     *  TDI             => PB6
     *  TRST/NRST       => PA13
     *  SRST/MCU_RESET  => PB5

 */

#define BOARD_IDENT "Black Magic on HiKoB Mystic Jaguar"

/* Hardware definitions... */
#define TDI_PORT    gpioPortB
#define TDI_PIN     6
#define TMS_PORT    gpioPortA
#define TMS_PIN     12
#define TCK_PORT    gpioPortC
#define TCK_PIN     4
#define TDO_PORT    gpioPortC
#define TDO_PIN     5

#define SWDIO_PORT  TMS_PORT
#define SWCLK_PORT  TCK_PORT
#define SWDIO_PIN   TMS_PIN
#define SWCLK_PIN   TCK_PIN

#define TRST_PORT   gpioPortA
#define TRST_PIN    13
#define SRST_PORT   gpioPortB
#define SRST_PIN    5

// Green LED (led1)
#define LED_IDLE_RUN_PORT   gpioPortE
#define LED_IDLE_RUN_PIN    9

// Red LED (led0)
#define LED_ERROR_PORT  gpioPortE
#define LED_ERROR_PIN   15

// Blue LED (led2)
#define LED_UART_PORT   gpioPortA
#define LED_UART_PIN    14

extern jmp_buf fatal_error_jmpbuf;

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()	{setjmp(fatal_error_jmpbuf);}

void jaguar_init();

void jaguar_target_3V(int enable);
void jaguar_target_5V(int enable);

int jaguar_target_3V_status();
int jaguar_target_5V_status();

/** Get current Power Sensing status, 1:active 0:inactive */
int jaguar_power_sensing_status();

typedef void (*jaguar_power_handler_t)(uint32_t rtc_timestamp, float voltage, float shunt, float current);

/** Set Power Sensing with measure handler */
void jaguar_power_sensing(jaguar_power_handler_t handler);

/** Function to call on alert interrupt */
void jaguar_power_alert(uint32_t rtc_timestamp);

#endif /* JAGUAR_H_ */

