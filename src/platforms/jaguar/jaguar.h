/*
 * jaguar.h
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#ifndef JAGUAR_H_
#define JAGUAR_H_

#include "platform.h"

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

void jaguar_init();

void jaguar_target_3V(int enable);
void jaguar_target_5V(int enable);

int jaguar_target_3V_status();
int jaguar_target_5V_status();

/** Get current Power Sensing status, 1:active 0:inactive */
int jaguar_power_sensing_status();

/** Set Power Sensing with measure handler */
void jaguar_power_sensing(
        void (*power_handler)(float voltage, float shunt, float current));

#endif /* JAGUAR_H_ */
