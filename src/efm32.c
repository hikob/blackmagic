/*
 * efm32.c
 *
 *  Created on: 9 juil. 2013
 *      Author: Liam Beguin <liambeguin.at.gmail.com>
 *      Author: Christophe Braillon <christophe.braillon.at.hikob.com>
 */


#include <stdlib.h>
#include <string.h>

#include "general.h"
#include "adiv5.h"
#include "target.h"
#include "command.h"
#include "gdb_packet.h"

/* keep family IDs in decimal */
#define EFM_FAMILY_ID_GECKO          71
#define EFM_FAMILY_ID_GIANT_GECKO    72
#define EFM_FAMILY_ID_TINY_GECKO     73
#define EFM_FAMILY_ID_LEOPARD_GECKO  74

#define DI_BASE_ADDRESS       0x0FE08000
#define DI_PART_NUMBER_OFFSET 0x1FC

#define MSC_BASE_ADDRESS      0x400C0000
#define MSC_WRITECTRL_OFFSET  0x08
#define MSC_WRITECMD_OFFSET   0x0C
#define MSC_ADDRB_OFFSET      0x10
#define MSC_WDATA             0x18
#define MSC_STATUS_OFFSET     0x1C
#define MSC_LOCK_OFFSET       0x3C
#define MSC_MASSLOCK_OFFSET   0x54

uint32_t page_size = 2048;

static bool efm32_cmd_erase_mass(target *t);

const struct command_s efm32_cmd_list[] =
{
	{"erase_mass", (cmd_handler)efm32_cmd_erase_mass, "Erase entire flash memory"},
	{NULL, NULL, NULL}
};

static int efm32_flash_erase(struct target_s *target, uint32_t addr, int len);
static int efm32_flash_write(struct target_s *target, uint32_t dest, const uint8_t *src, int len);

static const char efm32tg_driver_str[]  = "EFM32TG";
static const char efm32g_driver_str[]  = "EFM32G";
static const char efm32lg_driver_str[] = "EFM32LG";
static const char efm32gg_driver_str[] = "EFM32GG";

static const char efm32tg_xml_memory_map[] = "<?xml version=\"1.0\"?>"
		"<memory-map>"
		"  <memory type=\"flash\" start=\"0x00000000\" length=\"0x8000\">"
		"    <property name=\"blocksize\">0x200</property>"
		"  </memory>"
		"  <memory type=\"ram\" start=\"0x20000000\" length=\"0x1000\"/>"
		"</memory-map>";

static const char efm32g_xml_memory_map[] = "<?xml version=\"1.0\"?>"
		"<memory-map>"
		"  <memory type=\"flash\" start=\"0x00000000\" length=\"0x20000\">"
		"    <property name=\"blocksize\">0x200</property>"
		"  </memory>"
		"  <memory type=\"ram\" start=\"0x20000000\" length=\"0x4000\"/>"
		"</memory-map>";

static const char efm32lg_xml_memory_map[] = "<?xml version=\"1.0\"?>"
		"<memory-map>"
		"  <memory type=\"flash\" start=\"0x00000000\" length=\"0x80000\">"
		"    <property name=\"blocksize\">0x800</property>"
		"  </memory>"
		"  <memory type=\"ram\" start=\"0x20000000\" length=\"0x20000\"/>"
		"</memory-map>";

static const char efm32gg_xml_memory_map[] = "<?xml version=\"1.0\"?>"
		"<memory-map>"
		"  <memory type=\"flash\" start=\"0x00000000\" length=\"0x100000\">"
		"    <property name=\"blocksize\">0x800</property>"
		"  </memory>"
		"  <memory type=\"ram\" start=\"0x20000000\" length=\"0x20000\"/>"
		"</memory-map>";

static const uint8_t efm32_flash_write_stub[] =
{
			0x15, 0x4e,    /* ldr     r6, =#0x1b71 */
			0xc6, 0x63,    /* str     r6, [r0, #EFM32_MSC_LOCK_OFFSET] */
			0x01, 0x26,    /* movs    r6, #1 */
			0x86, 0x60,    /* str     r6, [r0, #EFM32_MSC_WRITECTRL_OFFSET] */

		/* wait_fifo: */
			0x16, 0x68,    /* ldr     r6, [r2, #0] */
			0x00, 0x2e,    /* cmp     r6, #0 */
			0x22, 0xd0,    /* beq     exit */
			0x55, 0x68,    /* ldr     r5, [r2, #4] */
			0xb5, 0x42,    /* cmp     r5, r6 */
			0xf9, 0xd0,    /* beq     wait_fifo */

			0x04, 0x61,    /* str     r4, [r0, #EFM32_MSC_ADDRB_OFFSET] */
			0x01, 0x26,    /* movs    r6, #1 */
			0xc6, 0x60,    /* str     r6, [r0, #EFM32_MSC_WRITECMD_OFFSET] */
			0xc6, 0x69,    /* ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET] */
			0x06, 0x27,    /* movs    r7, #6 */
			0x3e, 0x42,    /* tst     r6, r7 */
			0x16, 0xd1,    /* bne     error */

		/* wait_wdataready: */
			0xc6, 0x69,    /* ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET] */
			0x08, 0x27,    /* movs    r7, #8 */
			0x3e, 0x42,    /* tst     r6, r7 */
			0xfb, 0xd0,    /* beq     wait_wdataready */

			0x2e, 0x68,    /* ldr     r6, [r5] */
			0x86, 0x61,    /* str     r6, [r0, #EFM32_MSC_WDATA_OFFSET] */
			0x08, 0x26,    /* movs    r6, #8 */
			0xc6, 0x60,    /* str     r6, [r0, #EFM32_MSC_WRITECMD_OFFSET] */

			0x04, 0x35,    /* adds    r5, #4 */
			0x04, 0x34,    /* adds    r4, #4 */

		/* busy: */
			0xc6, 0x69,    /* ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET] */
			0x01, 0x27,    /* movs    r7, #1 */
			0x3e, 0x42,    /* tst     r6, r7 */
			0xfb, 0xd1,    /* bne     busy */

			0x9d, 0x42,    /* cmp     r5, r3 */
			0x01, 0xd3,    /* bcc     no_wrap */
			0x15, 0x46,    /* mov     r5, r2 */
			0x08, 0x35,    /* adds    r5, #8 */

		/* no_wrap: */
			0x55, 0x60,    /* str     r5, [r2, #4] */
			0x01, 0x39,    /* subs    r1, r1, #1 */
			0x00, 0x29,    /* cmp     r1, #0 */
			0x02, 0xd0,    /* beq     exit */
			0xdb, 0xe7,    /* b       wait_fifo */

		/* error: */
			0x00, 0x20,    /* movs    r0, #0 */
			0x50, 0x60,    /* str     r0, [r2, #4] */

		/* exit: */
			0x30, 0x46,    /* mov     r0, r6 */
			0x00, 0xbe,    /* bkpt    #0 */

		/* LOCKKEY */
			0x71, 0x1b, 0x00, 0x00
};


bool efm32_probe(struct target_s *target)
{
    uint32_t idcode;

    idcode = adiv5_ap_mem_read(adiv5_target_ap(target), DI_BASE_ADDRESS + DI_PART_NUMBER_OFFSET);

    switch((idcode >> 16) & 0xFF)
    {
        case EFM_FAMILY_ID_TINY_GECKO:
            page_size = 512;
            target->driver      = efm32tg_driver_str;
            target->xml_mem_map = efm32tg_xml_memory_map;
            break;

        case EFM_FAMILY_ID_GECKO:
            page_size = 512;
            target->driver      = efm32g_driver_str;
            target->xml_mem_map = efm32g_xml_memory_map;
            break;

        case EFM_FAMILY_ID_LEOPARD_GECKO:
            page_size = 2048;
            target->driver      = efm32lg_driver_str;
            target->xml_mem_map = efm32lg_xml_memory_map;
            break;

        case EFM_FAMILY_ID_GIANT_GECKO:
            page_size = 2048;
            target->driver      = efm32gg_driver_str;
            target->xml_mem_map = efm32gg_xml_memory_map;
            break;

        default:
            return false;
    }

    target->flash_erase = efm32_flash_erase;
    target->flash_write = efm32_flash_write;

    target_add_commands(target, efm32_cmd_list, "EFM32");

    return true;
}

static int efm32_flash_page_erase(struct target_s *target, uint32_t addr)
{
    uint32_t status;

    // Write address to ADDRB
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_ADDRB_OFFSET, addr);
    
    // Set LADDRIM 
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_WRITECMD_OFFSET, 0x1);

    // Read STATUS
    status = adiv5_ap_mem_read(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_STATUS_OFFSET);

    if(status & 0x6)
    {
        return 1;
    }

    // Write ERASEPAGE
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_WRITECMD_OFFSET, 0x2);

    // Wait for erase operation to complete
    do
    {
        status = adiv5_ap_mem_read(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_STATUS_OFFSET);
    } while(status & 0x1);

    return 0;
}

static int efm32_flash_erase(struct target_s *target, uint32_t addr, int len)
{
    uint32_t a;

    // Unlock command registers
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_LOCK_OFFSET, 0x1B71);

    // Enable write
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_WRITECTRL_OFFSET, 0x1);

    // Erase pages
    for(a = addr; a < addr + len; a += page_size)
    {
        if(efm32_flash_page_erase(target, a))
        {
            return 1;
        }
    }

    // Disable write
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_WRITECTRL_OFFSET, 0x0);

    // Lock command registers
	adiv5_ap_mem_write(adiv5_target_ap(target), MSC_BASE_ADDRESS + MSC_LOCK_OFFSET, 0x0);
    
	return 0;
}


static int efm32_flash_write(struct target_s *target, uint32_t dest, const uint8_t *src, int len)
{
	uint32_t offset = dest % 4;
	uint32_t words = (offset + len + 3) / 4;
	uint32_t data[2 + words];
    uint32_t regs[20]; 

	/* Construct data buffer used by stub */
	data[0] = 1;
	data[1] = 0x20000000 + sizeof(efm32_flash_write_stub) + 8;
	data[2] = 0xFFFFFFFF;		/* pad partial words with all 1s to avoid */
	data[words + 1] = 0xFFFFFFFF;	/* damaging overlapping areas */
	memcpy((uint8_t *)&data[2] + offset, src, len);

	/* Write stub and data to target ram and set PC */
	target_mem_write_words(target, 0x20000000, (void*)efm32_flash_write_stub, sizeof(efm32_flash_write_stub));
	target_mem_write_words(target, 0x20000000 + sizeof(efm32_flash_write_stub), data, words * 4 + 8);
	target_pc_write(target, 0x20000000);
	if(target_check_error(target))
		return -1;

    /* Set parameters in registers */
    target_regs_read(target, regs);
    regs[0] = MSC_BASE_ADDRESS;
    regs[1] = words;
    regs[2] = 0x20000000 + sizeof(efm32_flash_write_stub);
    regs[3] = regs[2] + words * 4 + 8;
    regs[4] = dest;
    target_regs_write(target, regs);
    
	/* Execute the stub */
	target_halt_resume(target, 0);
	while(!target_halt_wait(target));

    if(!adiv5_ap_mem_read(adiv5_target_ap(target), 0x20000000 + sizeof(efm32_flash_write_stub) + 4))
    {
        return -1;
    }    

    return 0;
}

static bool efm32_cmd_erase_mass(target *t)
{
    // TODO: not working yet...
    uint32_t status;

    // Unlock command registers
	adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_LOCK_OFFSET, 0x1B71);

    // Enable write
	adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_WRITECTRL_OFFSET, 0x1);
        
    // Unlock mass erase command 
	adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_MASSLOCK_OFFSET, 0x631A);
	
    // Start mass erase
    adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_WRITECMD_OFFSET, 0x300);

    // Wait for mass erase completion
    do
    {
        status = adiv5_ap_mem_read(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_STATUS_OFFSET);
    } while(status & 0x1);

    // Lock mass erase command
	adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_MASSLOCK_OFFSET, 0x0);

    // Disable write
	adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_WRITECTRL_OFFSET, 0x0);

    // Lock command registers
	adiv5_ap_mem_write(adiv5_target_ap(t), MSC_BASE_ADDRESS + MSC_LOCK_OFFSET, 0x0);

    return true;
}
