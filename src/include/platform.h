/*
 * platform.h
 *
 *  Created on: Mar 12, 2014
 *      Author: burindes
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

//#include "platform_implem.h"

#include <stdint.h>

/** Initialize the platform */
int platform_init(void);
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

void PLATFORM_FATAL_ERROR(int error);
void PLATFORM_SET_FATAL_ERROR_RECOVERY();

extern const char *morse_msg;
void morse(const char *msg, char repeat);

#define DEBUG(...)

#endif /* PLATFORM_H_ */
