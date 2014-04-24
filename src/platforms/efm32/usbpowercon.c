/*
 * usbpowercon.c
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#define DO_DEBUG 1

#include "em_device.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usb.h"
#include "em_usart.h"
#include "em_rtc.h"

#include "platform.h"
#include "dmactrl.h"

#include "usbpowercon.h"
#include "jaguar.h"

static void powercon_measure(uint32_t rtc_timestamp, float voltage, float shunt,
        float current);

cdcLineCoding_TypeDef usbpowercon_cdclinecoding;

static volatile int activated = 0;
static volatile int locked = 0;
static char power_buffer[256];

static float rtc_freq = 0;
static uint32_t rtc_loop = 0;

static uint32_t pin_last_state = 0;

void usbpowercon_init()
{
    // Register power measure
    jaguar_power_sensing(powercon_measure);

    RTC_IntDisable(RTC_IF_OF);
}

void usbpowercon_start()
{
    rtc_freq = (float) CMU_ClockFreqGet(cmuClock_RTC);

    RTC_CounterReset();
    rtc_loop = 0;
    pin_last_state = 0;

    /* Clear and enable interrupt */
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);
    RTC_IntEnable(RTC_IF_OF);
    RTC_IntClear(RTC_IFC_OF);

    activated = 1;
}
void usbpowercon_stop()
{
    activated = 0;
    RTC_IntDisable(RTC_IF_OF);
}
void RTC_IRQHandler()
{
    if (RTC_IntGet() & RTC_IF_OF)
    {
        rtc_loop++;
        RTC_IntClear(RTC_IFC_OF);
    }
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
static void powercon_measure(uint32_t rtc_timestamp, float voltage, float shunt,
        float current)
{
    if (!activated)
    {
        return;
    }

    uint32_t now = RTC_CounterGet();
    uint32_t loops_now = rtc_loop;

    if (now < rtc_timestamp)
    {
        DEBUG("Reducing loop => %d (ts %u, now %u)\n", loops_now, rtc_timestamp, now);
        loops_now -= 1;
    }

    float timestamp = loops_now * (1 << 24) / rtc_freq
            + (float) rtc_timestamp / rtc_freq;

    if (!locked)
    {
        int len = snprintf(power_buffer, 255, "POWER, %f, %f, %f, %f\n", timestamp,
                voltage, current, shunt);

        if (len > 255)
        {
            len = 255;
        }

        locked = 1;
        USBD_Write(EP_POWER_IN, (void*) power_buffer, len, UsbDataTransmitted);
    }
    else
    {
        DEBUG("Locked\n");
    }
}
void usbpowercon_pinstatechange(uint32_t rtc_timestamp, uint32_t state)
{
    if (!activated)
    {
        return;
    }
    if (state == pin_last_state)
    {
        return;
    }

    uint32_t now = RTC_CounterGet();
    uint32_t loops_now = rtc_loop;

    if (now < rtc_timestamp)
    {
        DEBUG("Reducing loop => %d (ts %u, now %u)\n", loops_now, rtc_timestamp, now);
        loops_now -= 1;
    }

    float timestamp = loops_now * (1 << 24) / rtc_freq
            + (float) rtc_timestamp / rtc_freq;

    if (!locked)
    {
        int len = snprintf(power_buffer, 255, "STATE, %f, %u\n", timestamp,
                state);

        if (len > 255)
        {
            len = 255;
        }

        locked = 1;
        USBD_Write(EP_POWER_IN, (void*) power_buffer, len, UsbDataTransmitted);

        pin_last_state = state;
    }
    else
    {
        DEBUG("Locked\n");
    }
}
