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

/* This file implements the low-level JTAG TAP interface.  */

#include <stdio.h>

#include "general.h"

#include "jtagtap.h"
#include "platform.h"

int jtagtap_init(void)
{
    jtag_tms_set_output();

	/* Go to JTAG mode for SWJ-DP */
	for(int i = 0; i <= 50; i++) jtagtap_next(1, 0); /* Reset SW-DP */
	jtagtap_tms_seq(0xE73C, 16);		/* SWD to JTAG sequence */
	jtagtap_soft_reset();

	return 0;
}

void jtagtap_reset(void)
{
	volatile int i;
	jtag_trst_set(0);
	for(i = 0; i < 10000; i++) asm("nop");
    jtag_trst_set(1);

	jtagtap_soft_reset();
}

void jtagtap_srst(bool assert)
{
	(void)assert;
#if 0
	jtag_srst_set(assert);
	if(assert) {
		int i;
		for(i = 0; i < 10000; i++)
			asm volatile("nop");
	}
#endif
}

inline uint8_t jtagtap_next(uint8_t dTMS, uint8_t dTDO)
{
	uint16_t ret;

	jtag_tms_set(dTMS);
	jtag_tdi_set(dTDO);

	jtag_tck_set(1);
	ret = jtag_tdo_get();
    jtag_tck_set(0);

	DEBUG("jtagtap_next(TMS = %d, TDO = %d) = %d\n", dTMS, dTDO, ret);

	return ret != 0;
}



#define PROVIDE_GENERIC_JTAGTAP_TMS_SEQ
#define PROVIDE_GENERIC_JTAGTAP_TDI_TDO_SEQ
#define PROVIDE_GENERIC_JTAGTAP_TDI_SEQ

#include "jtagtap_generic.c"

