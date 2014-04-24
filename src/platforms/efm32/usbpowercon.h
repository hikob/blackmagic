/*
 * usbpowercon.h
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#ifndef USBPOWERCON_H_
#define USBPOWERCON_H_

#include "efm_usbuart.h"

/** Initialize */
void usbpowercon_init();

void usbpowercon_start();
void usbpowercon_stop();

/** Function to call on input pin change */
void usbpowercon_pinstatechange(uint32_t rtc_timestamp, uint32_t state);

/** External definition of the line coding */
extern cdcLineCoding_TypeDef usbpowercon_cdclinecoding;

#endif /* USBPOWERCON_H_ */
