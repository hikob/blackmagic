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

/** External definition of the line coding */
extern cdcLineCoding_TypeDef usbpowercon_cdclinecoding;

#endif /* USBPOWERCON_H_ */
