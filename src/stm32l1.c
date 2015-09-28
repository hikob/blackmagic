/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012 Vegard Storheil Eriksen <zyp@jvnv.net>
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

/* This file implements STM32L1 target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 *
 * Refereces:
 * ST doc - RM0038
 *   Reference manual - STM32L151xx, STM32L152xx and STM32L162xx
 *   advanced ARM-based 32-bit MCUs
 * ST doc - PM0062
 *   Programming manual - STM32L151xx, STM32L152xx and STM32L162xx
 *   Flash and EEPROM programming
 */

#include <stdlib.h>
#include <string.h>

#include "general.h"
#include "adiv5.h"
#include "target.h"
#include "command.h"
#include "gdb_packet.h"

static int stm32l1_flash_erase(struct target_s *target, uint32_t addr, int len);
static int stm32l1_flash_write(struct target_s *target, uint32_t dest,
			const uint8_t *src, int len);

static const char stm32l1_driver_str[] = "STM32L1xx";
static const char stm32l0_driver_str[] = "STM32L0xx";

static const char stm32l1_xml_memory_map[] = "<?xml version=\"1.0\"?>"
/*	"<!DOCTYPE memory-map "
	"             PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\""
	"                    \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"*/
	"<memory-map>"
	"  <memory type=\"flash\" start=\"0x8000000\" length=\"0x80000\">"
	"    <property name=\"blocksize\">0x100</property>"
	"  </memory>"
	"  <memory type=\"ram\" start=\"0x20000000\" length=\"0x5000\"/>"
	"</memory-map>";

static const char stm32l0_xml_memory_map[] = "<?xml version=\"1.0\"?>"
/*  "<!DOCTYPE memory-map "
    "             PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\""
    "                    \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"*/
    "<memory-map>"
    "  <memory type=\"flash\" start=\"0x8000000\" length=\"0x10000\">"
    "    <property name=\"blocksize\">0x80</property>"
    "  </memory>"
    "  <memory type=\"ram\" start=\"0x20000000\" length=\"0x5000\"/>"
    "</memory-map>";

#define STM32L1_FLASH_PAGE_SIZE     256
#define STM32L0_FLASH_PAGE_SIZE     128

#define STM32L_FLASH_PAGE_SIZE(t) ((t->xml_mem_map == stm32l1_xml_memory_map)?STM32L1_FLASH_PAGE_SIZE:STM32L0_FLASH_PAGE_SIZE)
#define STM32L_FLASH_HALFPAGE_SIZE(t) (STM32L_FLASH_PAGE_SIZE(t) / 2)

/* Flash Controller Register Map */
#define STM32L1_FLASH_BASE	0x40023C00
#define STM32L0_FLASH_BASE  0x40022000

#define STM32L_FLASH_BASE_ADDRESS(t) ((t->xml_mem_map == stm32l1_xml_memory_map)?STM32L1_FLASH_BASE:STM32L0_FLASH_BASE)

#define STM32L1_FLASH_ACR(t)     (STM32L_FLASH_BASE_ADDRESS(t) + 0x00)
#define STM32L1_FLASH_PECR(t)    (STM32L_FLASH_BASE_ADDRESS(t) + 0x04)
#define STM32L1_FLASH_PDKEYR(t)  (STM32L_FLASH_BASE_ADDRESS(t) + 0x08)
#define STM32L1_FLASH_PEKEYR(t)  (STM32L_FLASH_BASE_ADDRESS(t) + 0x0C)
#define STM32L1_FLASH_PRGKEYR(t) (STM32L_FLASH_BASE_ADDRESS(t) + 0x10)
#define STM32L1_FLASH_OPTKEYR(t) (STM32L_FLASH_BASE_ADDRESS(t) + 0x14)
#define STM32L1_FLASH_SR(t)      (STM32L_FLASH_BASE_ADDRESS(t) + 0x18)
#define STM32L1_FLASH_OBR(t)     (STM32L_FLASH_BASE_ADDRESS(t) + 0x1C)
#define STM32L1_FLASH_WRPR1(t)   (STM32L_FLASH_BASE_ADDRESS(t) + 0x20)
#define STM32L1_FLASH_WRPR2(t)   (STM32L_FLASH_BASE_ADDRESS(t) + 0x80)
#define STM32L1_FLASH_WRPR3(t)   (STM32L_FLASH_BASE_ADDRESS(t) + 0x84)

#define STM32L1_FLASH_PECR_FPRG  (1 << 10)
#define STM32L1_FLASH_PECR_ERASE (1 <<  9)
#define STM32L1_FLASH_PECR_PROG  (1 <<  3)

#define STM32L1_FLASH_SR_BSY (1 << 0)
#define STM32L1_FLASH_SR_EOP (1 << 1)

#define STM32L1_FLASH_SR_ERROR_MASK (0x1f << 8)

#define STM32L1_PEKEY1 0x89ABCDEF
#define STM32L1_PEKEY2 0x02030405
#define STM32L1_PRGKEY1 0x8C9DAEBF
#define STM32L1_PRGKEY2 0x13141516

#define STM32L1_DBGMCU_IDCODE	0xE0042000
#define STM32L0_DBGMCU_IDCODE   0x40015800

bool stm32l1_probe(struct target_s *target)
{
	uint32_t idcode;

	// IDCODE for STM32L1
	idcode = adiv5_ap_mem_read(adiv5_target_ap(target), STM32L1_DBGMCU_IDCODE);

	DEBUG("STM32L1 IDCODE %08x\n", idcode);
	switch(idcode & 0xFFF) {
	case 0x416:  /* Medium density */
	case 0x427:  /* Medium+ density*/
	case 0x436:  /* Medium+/High density */
		target->driver = stm32l1_driver_str;
		target->xml_mem_map = stm32l1_xml_memory_map;
		target->flash_erase = stm32l1_flash_erase;
		target->flash_write = stm32l1_flash_write;

        DEBUG("STM32L1 valid!\n");
		return true;
	}

	// IDCODE for STM32L0
    idcode = adiv5_ap_mem_read(adiv5_target_ap(target), STM32L0_DBGMCU_IDCODE);
    DEBUG("STM32L0 IDCODE %08x\n", idcode);

    switch(idcode & 0xFFF) {
    case 0x417:
        target->driver = stm32l0_driver_str;
        target->xml_mem_map = stm32l0_xml_memory_map;
        target->flash_erase = stm32l1_flash_erase;
        target->flash_write = stm32l1_flash_write;

        DEBUG("STM32L0 valid!\n");
        return true;
    }

	return false;
}

static void stm32l1_flash_unlock(struct target_s *target, ADIv5_AP_t *ap)
{
	adiv5_ap_mem_write(ap, STM32L1_FLASH_PEKEYR(target), STM32L1_PEKEY1);
	adiv5_ap_mem_write(ap, STM32L1_FLASH_PEKEYR(target), STM32L1_PEKEY2);
	adiv5_ap_mem_write(ap, STM32L1_FLASH_PRGKEYR(target), STM32L1_PRGKEY1);
	adiv5_ap_mem_write(ap, STM32L1_FLASH_PRGKEYR(target), STM32L1_PRGKEY2);
}

static int stm32l1_flash_erase(struct target_s *target, uint32_t addr, int len)
{
	ADIv5_AP_t *ap = adiv5_target_ap(target);
	uint16_t sr;

	uint32_t page_size = STM32L_FLASH_PAGE_SIZE(target);

	addr &= ~(page_size - 1);
	len &= ~(page_size - 1);

	stm32l1_flash_unlock(target, ap);

	/* Flash page erase instruction */
	adiv5_ap_mem_write(ap, STM32L1_FLASH_PECR(target), STM32L1_FLASH_PECR_ERASE | STM32L1_FLASH_PECR_PROG);

	DEBUG("STM32L page size %u\n", page_size);

	/* Read FLASH_SR to poll for BSY bit */
	while(adiv5_ap_mem_read(ap, STM32L1_FLASH_SR(target)) & STM32L1_FLASH_SR_BSY)
	{
		if(target_check_error(target))
		{
	        DEBUG("ERROR 0\n");
			return -1;
		}
	}

	while(len) {
	    DEBUG("Write 0 at %08x\n", addr);

		/* Write first word of page to 0 */
		adiv5_ap_mem_write(ap, addr, 0);

		len -= page_size;
		addr += page_size;
	}

	/* Disable programming mode */
	adiv5_ap_mem_write(ap, STM32L1_FLASH_PECR(target), 0);

	/* Check for error */
	sr = adiv5_ap_mem_read(ap, STM32L1_FLASH_SR(target));
	if ((sr & STM32L1_FLASH_SR_ERROR_MASK) || !(sr & STM32L1_FLASH_SR_EOP))
	{
        DEBUG("ERROR 1\n");
		return -1;
	}

	return 0;
}

static int stm32l1_flash_write(struct target_s *target, uint32_t dest,
			  const uint8_t *src, int len)
{
	ADIv5_AP_t *ap = adiv5_target_ap(target);
	uint16_t sr;

	/* Handle non word-aligned start */
	if(dest & 3) {
		uint32_t data = 0;
		uint32_t wlen = 4 - (dest & 3);
		if(wlen > len)
			wlen = len;

		memcpy((uint8_t *)&data + (dest & 3), src, wlen);
		adiv5_ap_mem_write(ap, dest & ~3, data);
		src += wlen;
		dest += wlen;
		len -= wlen;
	}

	/* Handle non half-page-aligned start */
	if((dest & (STM32L_FLASH_HALFPAGE_SIZE(target) - 1)) && (len >= 4)) {
		uint32_t xlen = STM32L_FLASH_HALFPAGE_SIZE(target) - (dest & (STM32L_FLASH_HALFPAGE_SIZE(target) - 1));
		if(xlen > len)
			xlen = len & ~3;

		target_mem_write_words(target, dest, (uint32_t*)src, xlen);
		src += xlen;
		dest += xlen;
		len -= xlen;
	}

	/* Write half-pages */
	if(len > STM32L_FLASH_HALFPAGE_SIZE(target)) {
		/* Enable half page mode */
		adiv5_ap_mem_write(ap, STM32L1_FLASH_PECR(target), STM32L1_FLASH_PECR_FPRG | STM32L1_FLASH_PECR_PROG);

		/* Read FLASH_SR to poll for BSY bit */
		while(adiv5_ap_mem_read(ap, STM32L1_FLASH_SR(target)) & STM32L1_FLASH_SR_BSY)
			if(target_check_error(target))
				return -1;

		uint32_t write_len = len & ~(STM32L_FLASH_HALFPAGE_SIZE(target) - 1);
		target_mem_write_words(target, dest, (uint32_t*)src, write_len);
		src += write_len;
		dest += write_len;
		len -= write_len;

		/* Disable half page mode */
		adiv5_ap_mem_write(ap, STM32L1_FLASH_PECR(target), 0);

		/* Read FLASH_SR to poll for BSY bit */
		while(adiv5_ap_mem_read(ap, STM32L1_FLASH_SR(target)) & STM32L1_FLASH_SR_BSY)
			if(target_check_error(target))
				return -1;
	}

	/* Handle non-full page at the end */
	if(len >= 4) {
		target_mem_write_words(target, dest, (uint32_t*)src, len & ~3);
		src += len & ~3;
		dest += len & ~3;
		len -= len & ~3;
	}

	/* Handle non-full word at the end */
	if(len) {
		uint32_t data = 0;

		memcpy((uint8_t *)&data, src, len);
		adiv5_ap_mem_write(ap, dest, data);
	}

	/* Check for error */
	sr = adiv5_ap_mem_read(ap, STM32L1_FLASH_SR(target));
	if ((sr & STM32L1_FLASH_SR_ERROR_MASK) || !(sr & STM32L1_FLASH_SR_EOP))
		return -1;

	return 0;
}

static bool stm32l_cmd_erase_mass(target *t)
{
    return stm32l1_flash_erase(t, 0x08000000, 65536);
}
