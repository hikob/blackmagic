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

/* This file implements a transparent channel over which the GDB Remote
 * Serial Debugging protocol is implemented.  This implementation for STM32
 * uses the USB CDC-ACM device bulk endpoints to implement the channel.
 */
#include "platform.h"
#include "gdb_if.h"

#include "em_usb.h"

#define CDCACM_PACKET_SIZE 64

static uint32_t count_out;
static uint32_t count_new;
static uint32_t count_in;
static uint32_t out_ptr;
static uint8_t buffer_out[CDCACM_PACKET_SIZE];
static uint8_t double_buffer_out[CDCACM_PACKET_SIZE];
static uint8_t buffer_in[CDCACM_PACKET_SIZE];

static int UsbDataReceived(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining);
static int UsbDataTransmitted(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining);

int gdb_if_init(void)
{
    /* Start receiving data from USB host. */
    USBD_Read(EP_DATA_OUT, (void*) double_buffer_out, USB_RX_BUF_SIZ,
            UsbDataReceived);

    count_new = 0;
    out_ptr = 0;

    return 0;
}
static volatile int usb_transmitted = 0;
void gdb_if_putchar(unsigned char c, int flush)
{
    buffer_in[count_in++] = c;
    if (flush || (count_in == CDCACM_PACKET_SIZE))
    {
        /* Refuse to send if USB isn't configured, and
         * don't bother if nobody's listening */
        if ((cdcacm_get_config() != 1) || !cdcacm_get_dtr())
        {
            count_in = 0;
            return;
        }
        usb_transmitted = 0;
        USBD_Write(EP_DATA_IN, (void*) buffer_in, count_in, UsbDataTransmitted);
        while (usb_transmitted == 0)
            ;
        count_in = 0;
    }
}

unsigned char gdb_if_getchar(void)
{
    while (!(out_ptr < count_out))
    {
        /* Detach if port closed */
        if (!cdcacm_get_dtr())
        {
            return 0x04;
        }

        while (cdcacm_get_config() != 1)
            ;

        if (count_new)
        {
            memcpy(buffer_out, double_buffer_out, count_new);
            count_out = count_new;
            count_new = 0;
            out_ptr = 0;

            // Start new RX transfer
            USBD_Read(EP_DATA_OUT, (void*) double_buffer_out, USB_RX_BUF_SIZ,
                    UsbDataReceived);
        }
    }

    return buffer_out[out_ptr++];
}

unsigned char gdb_if_getchar_to(int timeout)
{
    timeout_counter = timeout / 100;

    if (!(out_ptr < count_out))
        do
        {
            /* Detach if port closed */
            if (!cdcacm_get_dtr())
            {
                return 0x04;
            }

            while (cdcacm_get_config() != 1)
                ;

            if (count_new)
            {
                memcpy(buffer_out, double_buffer_out, count_new);
                count_out = count_new;
                count_new = 0;
                out_ptr = 0;

                // Start new RX transfer
                USBD_Read(EP_DATA_OUT, (void*) double_buffer_out,
                        USB_RX_BUF_SIZ, UsbDataReceived);
            }
        } while (timeout_counter && !(out_ptr < count_out));

    if (out_ptr < count_out)
        return gdb_if_getchar();

    return -1;
}

/**************************************************************************//**
 * @brief Callback function called whenever a new packet with data is received
 *        on USB.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int UsbDataReceived(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining)
{
    (void) remaining; /* Unused parameter */

    if ((status == USB_STATUS_OK) && (xferred > 0))
    {
        count_new = xferred;
    }

    return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief Callback function called whenever a packet with data has been
 *        transmitted on USB
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int UsbDataTransmitted(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining)
{
    (void) xferred; /* Unused parameter */
    (void) remaining; /* Unused parameter */

    if (status == USB_STATUS_OK)
    {
        /** TX OK */
        usb_transmitted = 1;
    }
    return USB_STATUS_OK;
}

