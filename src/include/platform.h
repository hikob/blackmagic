/*
 * platform.h
 *
 *  Created on: Mar 12, 2014
 *      Author: burindes
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdint.h>
#include <setjmp.h>
#include <alloca.h>
#include <string.h>
#include "gdb_packet.h"

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;


/** Initialize the platform */
int platform_init();

/** Read target voltage */
const char *platform_target_voltage();

/** Blocking Delay, set timeout_counter to the delay value and wait for expiration */
void platform_delay(uint32_t delay);

/** Counter decremented at each system tick */
extern volatile uint32_t timeout_counter;

/** Return 1 if USB is configured */
int cdcacm_get_config();
/** Return 1 if somebody is listening */
int cdcacm_get_dtr();

/** Configure the TMS pin as output */
void jtag_tms_set_output();

/** Set TMS pin value */
void jtag_tms_set(int val);
/** Set TDI pin value */
void jtag_tdi_set(int val);
/** Set TCK pin value */
void jtag_tck_set(int val);
/** Get TDO pin value */
int jtag_tdo_get();

/** Set the SWDIO pin input for reading */
void jtag_swdio_set_input();
/** Set the SWDIO pin output for writing */
void jtag_swdio_set_output();

/** Set the value of SWDIO pin*/
void jtag_swdio_set(int val);
/** Get SWDIO pin value */
int jtag_swdio_get();

/** Set the value of SWDCLK pin*/
void jtag_swclk_set(int val);

/** Set the TRST pin value */
void jtag_trst_set(int val);
/** Set the SRST pin value */
void jtag_srst_set(int val);

/** Make JTAG pins High Z */
void jtag_pings_high_z();
/** Make JTAG pins active */
void jtag_pins_active();

extern jmp_buf fatal_error_jmpbuf;

/** Set the running state of the target (1=running, 0=halted) */
void SET_RUN_STATE(int state);
/** Set the idle state of the target */
void SET_IDLE_STATE(int state);
/** Set the error state of the target */
void SET_ERROR_STATE(int state);

void PLATFORM_FATAL_ERROR(int error);
void PLATFORM_SET_FATAL_ERROR_RECOVERY();

void led_uart_on();
void led_uart_off();
void led_uart_toggle();

extern const char *morse_msg;
void morse(const char *msg, char repeat);


#include "stdio.h"
void uart_write(const char* msg);
extern char print_buf[1024];
#define uart_printf(...) do { siprintf(print_buf, __VA_ARGS__); uart_write(print_buf);} while (0)

#ifndef DO_DEBUG
#define DO_DEBUG 0
#endif

#if DO_DEBUG
#define DEBUG(...) uart_printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

#endif /* PLATFORM_H_ */
