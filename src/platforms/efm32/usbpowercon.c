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
#include "usbconfig.h"

#include "usbpowercon.h"
#include "jaguar.h"

static void powercon_measure(uint32_t rtc_timestamp, float voltage, float shunt,
        float current);

static int UsbDataTransmitted(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining);
static void send_current_buffer();
static float get_timestamp(uint32_t rtc_timestamp);

cdcLineCoding_TypeDef usbpowercon_cdclinecoding;

static volatile int activated = 0;

enum
{
    BUF_MAX_SIZE = 64
};

/** Double buffered write */
static char power_buffer_0[BUF_MAX_SIZE];
static char power_buffer_1[BUF_MAX_SIZE];

static const char* power_buffers[2] =
{ power_buffer_0, power_buffer_1 };

/** Double buffered sizes */
static int power_buffer_sizes[2] =
{ 0, 0 };

/** Double buffered buffer to fill */
static int power_buffer_write_to = 1;

/** Flag indicating buffer is being transmitted */
static int usb_sending = 0;

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

    usb_sending = 0;
    send_current_buffer();

    return USB_STATUS_OK;
}

static void send_current_buffer()
{
    if (usb_sending || power_buffer_sizes[power_buffer_write_to] == 0)
    {
        return;
    }

    usb_sending = 1;

    const char* buf = power_buffers[power_buffer_write_to];
    int buf_len = power_buffer_sizes[power_buffer_write_to];

    USBD_Write(EP_POWER_IN, (void*) buf, buf_len, UsbDataTransmitted);

    // Clear buffer size and rotate
    power_buffer_sizes[power_buffer_write_to] = 0;
    power_buffer_write_to ^= 1;
}

static float get_timestamp(uint32_t rtc_timestamp)
{
    uint32_t now = RTC_CounterGet();
    uint32_t loops_now = rtc_loop;
    if (now < rtc_timestamp)
    {
        loops_now -= 1;
    }

    float timestamp = loops_now * (1 << 24) / rtc_freq
            + (float) rtc_timestamp / rtc_freq;
    return timestamp;
}

static void powercon_measure(uint32_t rtc_timestamp, float voltage, float shunt,
        float current)
{
    if (!activated)
    {
        return;
    }

    float timestamp = get_timestamp(rtc_timestamp);

    int allowed_space = BUF_MAX_SIZE
            - power_buffer_sizes[power_buffer_write_to];

    // Write data to selected buffer
    int len = snprintf(power_buffers[power_buffer_write_to], allowed_space,
            "POWER, %f, %f, %f, %f\n", timestamp, voltage, current, shunt);

    if (len >= allowed_space)
    {
        DEBUG("P Failed %u %u\n", len, allowed_space);
    }
    else
    {
        power_buffer_sizes[power_buffer_write_to] += len;
        // try to send
        send_current_buffer();
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

    float timestamp = get_timestamp(rtc_timestamp);
    int allowed_space = BUF_MAX_SIZE
            - power_buffer_sizes[power_buffer_write_to];

    int len = snprintf(power_buffers[power_buffer_write_to], allowed_space,
            "STATE, %f, %u\n", timestamp, state);

    if (len >= allowed_space)
    {
        DEBUG("S Failed %u %u\n", len, allowed_space);
    }
    else
    {
        power_buffer_sizes[power_buffer_write_to] += len;
        // try to send
        send_current_buffer();
        pin_last_state = state;
    }
}
