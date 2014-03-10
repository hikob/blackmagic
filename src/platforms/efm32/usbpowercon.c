/*
 * usbpowercon.c
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#include "em_device.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usb.h"
#include "em_usart.h"

#include "platform.h"
#include "dmactrl.h"

#include "usbpowercon.h"
#include "jaguar.h"

#undef DEBUG
#define DEBUG(...) uart_printf(__VA_ARGS__)

static void powercon_measure(float voltage, float shunt, float current);

cdcLineCoding_TypeDef usbpowercon_cdclinecoding;

static volatile int activated = 0;
static volatile int locked = 0;
static char power_buffer[256];

void usbpowercon_init()
{
    // Register power measure
    jaguar_power_sensing(powercon_measure);
}

void usbpowercon_start()
{
    activated = 1;
}
void usbpowercon_stop()
{
    activated = 0;
}
static int UsbDataTransmitted(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining)
{
    (void) status;
    (void) xferred;
    (void) remaining;

    locked = 0;
    return USB_STATUS_OK;
}
static void powercon_measure(float voltage, float shunt, float current)
{
    if (!activated)
    {
        return;
    }

    if (!locked)
    {
        int len = snprintf(power_buffer, 255, "%f, %f, %f\n", voltage, current,
                shunt);

        if (len > 255)
        {
            len = 255;
        }

        locked = 1;
        USBD_Write(EP_POWER_IN, (void*) power_buffer, len, UsbDataTransmitted);
    }
    else
    {
        DEBUG("Locked");
    }
}
