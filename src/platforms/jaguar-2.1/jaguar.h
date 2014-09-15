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
     *  TMS  / SWDIO    => PB11
     *  TCK  / SWCLK    => PA13
     *  TDO             => PB8
     *  TDI             => PA14
     *  TRST/NRST       => PA12
     *  SRST/MCU_RESET  => PB7
 */

#define BOARD_IDENT "Black Magic on HiKoB Mystic Jaguar v2.1"

/* Hardware definitions... */
#define TDI_PORT    gpioPortA
#define TDI_PIN     14
#define TMS_PORT    gpioPortB
#define TMS_PIN     11
#define TCK_PORT    gpioPortA
#define TCK_PIN     13
#define TDO_PORT    gpioPortB
#define TDO_PIN     8

#define SWDIO_PORT  TMS_PORT
#define SWCLK_PORT  TCK_PORT
#define SWDIO_PIN   TMS_PIN
#define SWCLK_PIN   TCK_PIN

#define TRST_PORT   gpioPortA
#define TRST_PIN    12
#define SRST_PORT   gpioPortB
#define SRST_PIN    7

// Red LED (led0)
#define LED_ERROR_PORT  gpioPortE
#define LED_ERROR_PIN   5

// Green LED (led1)
#define LED_IDLE_RUN_PORT   gpioPortE
#define LED_IDLE_RUN_PIN    4

// Blue LED (led2)
#define LED_UART_PORT   gpioPortD
#define LED_UART_PIN    2

// DBG Uart Pins
#define DBG_UART            USART2
#define DBG_UART_TX_PORT    gpioPortB
#define DBG_UART_TX_PIN     3
#define DBG_UART_RX_PORT    gpioPortB
#define DBG_UART_RX_PIN     4

// User Uart Pins
#define USR_UART            USART1
#define USR_UART_TX_PORT    gpioPortD
#define USR_UART_TX_PIN     0
#define USR_UART_RX_PORT    gpioPortD
#define USR_UART_RX_PIN     1

// Power Measure Pins
#define POWER_I2C_SDA_PORT  gpioPortA
#define POWER_I2C_SDA_PIN   0
#define POWER_I2C_SCL_PORT  gpioPortA
#define POWER_I2C_SCL_PIN   1
#define POWER_ALERT_PORT    gpioPortA
#define POWER_ALERT_PIN 6

// Target Power configuration pins
#define TARGET_EN_PORT  gpioPortD
#define TARGET_EN_PIN   4

#define TARGET_PG_PORT  gpioPortD
#define TARGET_PG_PIN   3

#define TARGET_5V_PORT  gpioPortC
#define TARGET_5V_PIN   7

// Target buffer pins
#define TARGET_BUF_EN_PORT  gpioPortB
#define TARGET_BUF_EN_PIN   12

// Target voltage selection pins
#define TARGET_VOLTAGE_2p0_PORT     gpioPortC
#define TARGET_VOLTAGE_2p0_PIN      6
#define TARGET_VOLTAGE_2p5_PORT     gpioPortD
#define TARGET_VOLTAGE_2p5_PIN      5
#define TARGET_VOLTAGE_3p3_PORT     gpioPortD
#define TARGET_VOLTAGE_3p3_PIN      8
#define TARGET_VOLTAGE_3p6_PORT     gpioPortD
#define TARGET_VOLTAGE_3p6_PIN      7
#define TARGET_VOLTAGE_4p2_PORT     gpioPortD
#define TARGET_VOLTAGE_4p2_PIN      6

// Target VDD voltage selection pins
#define TARGET_VDD_VOLTAGE_2p0_PORT     gpioPortA
#define TARGET_VDD_VOLTAGE_2p0_PIN      15
#define TARGET_VDD_VOLTAGE_2p5_PORT     gpioPortE
#define TARGET_VDD_VOLTAGE_2p5_PIN      15
#define TARGET_VDD_VOLTAGE_3p3_PORT     gpioPortE
#define TARGET_VDD_VOLTAGE_3p3_PIN      14

extern jmp_buf fatal_error_jmpbuf;

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()	{setjmp(fatal_error_jmpbuf);}

void jaguar_init();

enum JaguarVoltage
{
    JAGUAR_VOLTAGE_OFF,
    JAGUAR_VOLTAGE_2p0,
    JAGUAR_VOLTAGE_2p5,
    JAGUAR_VOLTAGE_3p3,
    JAGUAR_VOLTAGE_3p6,
    JAGUAR_VOLTAGE_4p2,
};

enum JaguarVddVoltage
{
    JAGUAR_VDD_VOLTAGE_2p0,
    JAGUAR_VDD_VOLTAGE_2p5,
    JAGUAR_VDD_VOLTAGE_3p3,
};

void jaguar_target_select_voltage(enum JaguarVoltage voltage);
void jaguar_target_select_vdd_voltage(enum JaguarVddVoltage voltage);
void jaguar_target_5V(int enable);

int jaguar_target_voltage_status();
int jaguar_target_5V_status();

/** Get current Power Sensing status, 1:active 0:inactive */
int jaguar_power_sensing_status();

typedef void (*jaguar_power_handler_t)(uint32_t rtc_timestamp, float voltage, float shunt, float current);

/** Set Power Sensing with measure handler */
void jaguar_power_sensing(jaguar_power_handler_t handler);

/** Function to call on alert interrupt */
void jaguar_power_alert(uint32_t rtc_timestamp);

#endif /* JAGUAR_H_ */

