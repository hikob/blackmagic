/**************************************************************************//**
 * @file main.c
 * @brief USB CDC Serial Port adapter example project.
 * @author Energy Micro AS
 * @version 3.20.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include "em_device.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usb.h"
#include "em_chip.h"
#include "em_usart.h"

#include "platform.h"
#include "gdb_if.h"
#include "gdb_main.h"
#include "jtag_scan.h"

/*** Typedef's and defines. ***/

/* Define USB endpoint addresses */
#define EP_DATA_OUT       0x01  /* Endpoint for USB data reception.       */
#define EP_DATA_IN        0x81  /* Endpoint for USB data transmission.    */
#define EP_NOTIFY         0x82  /* The notification endpoint (not used).  */

#define BULK_EP_SIZE     USB_MAX_EP_SIZE  /* This is the max. ep size.    */
#define USB_RX_BUF_SIZ   BULK_EP_SIZE /* Packet size when receiving on USB*/
#define USB_TX_BUF_SIZ   127    /* Packet size when transmitting on USB.  */

/* The serial port LINE CODING data structure, used to carry information  */
/* about serial port baudrate, parity etc. between host and device.       */
EFM32_PACK_START(1)
typedef struct
{
    uint32_t dwDTERate; /** Baudrate                            */
    uint8_t bCharFormat; /** Stop bits, 0=1 1=1.5 2=2            */
    uint8_t bParityType; /** 0=None 1=Odd 2=Even 3=Mark 4=Space  */
    uint8_t bDataBits; /** 5, 6, 7, 8 or 16                    */
    uint8_t dummy; /** To ensure size is a multiple of 4 bytes.*/
}__attribute__ ((packed)) cdcLineCoding_TypeDef;
EFM32_PACK_END()

static int usb_configured = 0;
static int usb_cdc_dtr = 0;
int cdcacm_get_config(void)
{
    return usb_configured;
}
int cdcacm_get_dtr(void)
{
    return usb_cdc_dtr;
}
/*** Function prototypes. ***/
static void SerialPortInit(void);

static int SetupCmd(const USB_Setup_TypeDef *setup);
static void StateChange(USBD_State_TypeDef oldState,
        USBD_State_TypeDef newState);

/*** Include device descriptor definitions. ***/

#include "descriptors.h"

/*** Variables ***/

uint8_t running_status;
volatile uint32_t timeout_counter;

jmp_buf fatal_error_jmpbuf;

const char *morse_msg;
void morse(const char *msg, char repeat)
{
    (void) msg;
    (void) repeat;
}
void platform_delay(uint32_t delay)
{
    timeout_counter = delay;
    while (timeout_counter)
        ;
}
const char *platform_target_voltage(void)
{
    return "???";
}
void SysTick_Handler(void)
{
    GPIO_PinOutToggle(gpioPortE, 3);

    if (running_status)
    {
        GPIO_PinOutToggle(LED_PORT, LED_IDLE_RUN);
        DEBUG(".");
    }

    if (timeout_counter)
    {
        timeout_counter--;
    }
}

/*
 * The LineCoding variable must be 4-byte aligned as it is used as USB
 * transmit and receive buffer
 */
EFM32_ALIGN(4)
EFM32_PACK_START(1)
static cdcLineCoding_TypeDef __attribute__ ((aligned(4))) cdcLineCoding =
{
    115200, 0, 0, 8, 0
};
EFM32_PACK_END()

void uart_write(const char* msg)
{
    const char *c;
    for (c = msg; *c != 0x0; c++)
    {
        USART_Tx(USART2, *c);
    }
}
char print_buf[1024];

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/

int platform_init()
{
    SCB->VTOR=0x4000;

    /* Chip errata */
    CHIP_Init();

    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    GPIO_PinModeSet(ACTIVITY_LED, gpioModePushPull, 0);
    GPIO_PinOutClear(ACTIVITY_LED);
    SerialPortInit();

    uart_printf("CDC Test Started %x\n", 0x1234);

    /* Setup SysTick Timer for 100 msec interrupts  */
    SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 10);

    // Custom LED
    GPIO_PinModeSet(CUSTOM_LED, gpioModePushPull,  0);

    // GPIO prepare
    GPIO_PinModeSet(TRST_PORT, TRST_PIN, gpioModePushPull,  1);
    GPIO_PinModeSet(SRST_PORT, SRST_PIN, gpioModePushPull,  1);

    GPIO_PinModeSet(TMS_PORT, TMS_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(TCK_PORT, TCK_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(TDI_PORT, TDI_PIN, gpioModePushPull, 0);

    // Enable Target Power
    GPIO_PinModeSet(TARGET_EN_PORT, TARGET_EN_PIN, gpioModePushPull,  1);
    GPIO_PinModeSet(TARGET_5V_PORT, TARGET_5V_PIN, gpioModePushPull,  1);

    USBD_Init(&initstruct);

    /*
     * When using a debugger it is practical to uncomment the following three
     * lines to force host to re-enumerate the device.
     */
    USBD_Disconnect();
    USBTIMER_DelayMs(1000);
    USBD_Connect();

    jtag_scan(NULL);

    return 0;
}

/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts CDC operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
static void StateChange(USBD_State_TypeDef oldState,
        USBD_State_TypeDef newState)
{
    uart_printf("USB State changed %x->%x\n", oldState, newState);

    if (newState == USBD_STATE_CONFIGURED)
    {
        /* We have been configured, start CDC functionality ! */

        if (oldState == USBD_STATE_SUSPENDED) /* Resume ?   */
        {
        }

        uart_printf("\t\t->Started!!\n");
        usb_configured = 1;

        gdb_if_init();

    }

    else if ((oldState == USBD_STATE_CONFIGURED)
            && (newState != USBD_STATE_SUSPENDED))
    {
        /* We have been de-configured, stop CDC functionality */
        uart_printf("\t\t->Stopped!!\n");
        usb_configured = 0;
    }

    else if (newState == USBD_STATE_SUSPENDED)
    {
        /* We have been suspended, stop CDC functionality */
        /* Reduce current consumption to below 2.5 mA.    */
        uart_printf("\t\t->Stopped!!\n");
        usb_configured = 0;
    }
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements CDC class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
static int SetupCmd(const USB_Setup_TypeDef *setup)
{
    int retVal = USB_STATUS_REQ_UNHANDLED;

    if ((setup->Type == USB_SETUP_TYPE_CLASS)
            && (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
    {
        switch (setup->bRequest)
        {
            case USB_CDC_GETLINECODING:
                /********************/
                if ((setup->wValue == 0) && (setup->wIndex == 0) && /* Interface no.            */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction == USB_SETUP_DIR_IN))
                {
                    /* Send current settings to USB host. */
                    USBD_Write(0, (void*) &cdcLineCoding, 7, NULL);
                    retVal = USB_STATUS_OK;
                }
                break;

            case USB_CDC_SETLINECODING:
                /********************/
                if ((setup->wValue == 0) && (setup->wIndex == 0) && /* Interface no.            */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction != USB_SETUP_DIR_IN))
                {
                    /* Get new settings from USB host. */
                    USBD_Read(0, (void*) &cdcLineCoding, 7, NULL);
                    retVal = USB_STATUS_OK;
                }
                break;

            case USB_CDC_SETCTRLLINESTATE:
                /********************/
                if ((setup->wIndex == 0) && /* Interface no.  */
                (setup->wLength == 0)) /* No data        */
                {
                    usb_cdc_dtr = setup->wValue & 1;
                    uart_printf("USB CDC DTR = %u\n", usb_cdc_dtr);

                    /* Do nothing ( Non compliant behaviour !! ) */
                    retVal = USB_STATUS_OK;
                }
                break;
        }
    }

    return retVal;
}

/**************************************************************************//**
 * @brief Initialize the UART peripheral.
 *****************************************************************************/
static void SerialPortInit(void)
{
    USART_TypeDef *uart = USART2;
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

    /* Configure GPIO pins */
    CMU_ClockEnable(cmuClock_GPIO, true);
    /* To avoid false start, configure output as high */
    GPIO_PinModeSet(gpioPortB, 3, gpioModePushPull, 1);
    GPIO_PinModeSet(gpioPortB, 4, gpioModeInput, 0);

    /* Enable peripheral clocks */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_USART2, true);

    /* Configure UART for basic async operation */
    init.enable = usartDisable;
    init.baudrate = 500000;
    USART_InitAsync(uart, &init);

    /* Enable pins at USART2 location #1 */
    uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN
            | USART_ROUTE_LOCATION_LOC1;

    /* Finally enable it */
    USART_Enable(uart, usartEnable);
}
