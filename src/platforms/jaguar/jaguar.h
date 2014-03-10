/*
 * jaguar.h
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#ifndef JAGUAR_H_
#define JAGUAR_H_

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
