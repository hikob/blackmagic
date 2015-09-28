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
#include "em_gpio.h"
#include "em_int.h"
#include "em_usb.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_rtc.h"

#include "platform.h"

#include "gdb_if.h"
#include "gdb_main.h"
#include "jtag_scan.h"
#include "target.h"

#include "efm_usbuart.h"
#include "usbpowercon.h"

#include "jaguar.h"

/*** Typedef's and defines. ***/

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

#undef SET_RUN_STATE
#undef SET_IDLE_STATE
#undef SET_ERROR_STATE
#undef PLATFORM_FATAL_ERROR

void SET_RUN_STATE(int state)
{
    running_status = state;
}
void SET_IDLE_STATE(int state)
{
    if (state)
    {
        GPIO_PinOutClear(LED_IDLE_RUN_PORT, LED_IDLE_RUN_PIN);
    }
    else
    {
        GPIO_PinOutSet(LED_IDLE_RUN_PORT, LED_IDLE_RUN_PIN);
    }
}
void SET_ERROR_STATE(int state)
{
    if (state)
    {
        GPIO_PinOutClear(LED_ERROR_PORT, LED_ERROR_PIN);
    }
    else
    {
        GPIO_PinOutSet(LED_ERROR_PORT, LED_ERROR_PIN);
    }
}

void PLATFORM_FATAL_ERROR(int error)
{
    SET_ERROR_STATE(1);
    if (running_status)
        gdb_putpacketz("X1D");
    else
        gdb_putpacketz("EFF");
    running_status = 0;
    target_list_free();
    morse("TARGET LOST.", 1);

    uart_write("xxxxxxxxxxxxxxxxxxx TARGET LOST xxxxxxxxxxxxxxxxxxx\n");

    USBTIMER_DelayMs(500);
    SET_ERROR_STATE(0);

    longjmp(fatal_error_jmpbuf, (error));
}

void led_uart_on()
{
    GPIO_PinOutClear(LED_UART_PORT, LED_UART_PIN);
}
void led_uart_off()
{
    GPIO_PinOutSet(LED_UART_PORT, LED_UART_PIN);
}
void led_uart_toggle()
{
    GPIO_PinOutToggle(LED_UART_PORT, LED_UART_PIN);
}

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
    {
    }
}
const char *platform_target_voltage(void)
{
    return "???";
}
void SysTick_Handler(void)
{
    if (running_status)
    {
        GPIO_PinOutToggle(LED_IDLE_RUN_PORT, LED_IDLE_RUN_PIN);
        DEBUG(".");
    }

    if (timeout_counter)
    {
        timeout_counter--;
    }
}

void jtag_tms_set_output()
{
    GPIO_PinModeSet(TMS_PORT, TMS_PIN, gpioModePushPull, 0);
}

void jtag_tms_set(int val)
{
    if (val)
    {
        GPIO_PinOutSet(TMS_PORT, TMS_PIN);
    }
    else
    {
        GPIO_PinOutClear(TMS_PORT, TMS_PIN);
    }
}
void jtag_tdi_set(int val)
{
    if (val)
    {
        GPIO_PinOutSet(TDI_PORT, TDI_PIN);
    }
    else
    {
        GPIO_PinOutClear(TDI_PORT, TDI_PIN);
    }
}
void jtag_tck_set(int val)
{
    if (val)
    {
        GPIO_PinOutSet(TCK_PORT, TCK_PIN);
    }
    else
    {
        GPIO_PinOutClear(TCK_PORT, TCK_PIN);
    }
}

int jtag_tdo_get()
{
    return GPIO_PinInGet(TDO_PORT, TDO_PIN);
}

void jtag_swdio_set_input()
{
    GPIO_PinModeSet(SWDIO_PORT, SWDIO_PIN, gpioModeInput, 0);
}
void jtag_swdio_set_output()
{
    GPIO_PinModeSet(SWDIO_PORT, SWDIO_PIN, gpioModePushPull, 0);
}

void jtag_swdio_set(int val)
{
    if (val)
    {
        GPIO_PinOutSet(SWDIO_PORT, SWDIO_PIN);
    }
    else
    {
        GPIO_PinOutClear(SWDIO_PORT, SWDIO_PIN);
    }
}
int jtag_swdio_get()
{
    return GPIO_PinInGet(SWDIO_PORT, SWDIO_PIN);
}

void jtag_swclk_set(int val)
{
    if (val)
    {
        GPIO_PinOutSet(SWCLK_PORT, SWCLK_PIN);
    }
    else
    {
        GPIO_PinOutClear(SWCLK_PORT, SWCLK_PIN);
    }
}

void jtag_trst_set(int val)
{
    if (val)
    {
        GPIO_PinOutSet(TRST_PORT, TRST_PIN);
    }
    else
    {
        GPIO_PinOutClear(TRST_PORT, TRST_PIN);
    }
}
void jtag_srst_set(int val)
{
    if (val)
    {
        uart_write("\tSRST set!\n");
        GPIO_PinOutClear(SRST_PORT, SRST_PIN);
    }
    else
    {
        uart_write("\tSRST clear!\n");
        GPIO_PinOutSet(SRST_PORT, SRST_PIN);
    }
}

static inline void pin_reset()
{
    jtag_srst_set(0);
    platform_delay(1);
    jtag_srst_set(1);
    platform_delay(3);
}

static int jtag_pins_are_high_z = 1;
void jtag_pins_high_z()
{
    if (jtag_pins_are_high_z)
    {
        return;
    }

    // Reset!!!
    //pin_reset();

    // GPIO prepare for JTAG INPUT
    GPIO_PinModeSet(TRST_PORT, TRST_PIN, gpioModeInput, 0);
    GPIO_PinModeSet(SRST_PORT, SRST_PIN, gpioModeInput, 0);

    GPIO_PinModeSet(TMS_PORT, TMS_PIN, gpioModeInput, 0);
    GPIO_PinModeSet(TCK_PORT, TCK_PIN, gpioModeInput, 0);

    GPIO_PinModeSet(TDI_PORT, TDI_PIN, gpioModeInput, 0);
    GPIO_PinModeSet(TDO_PORT, TDO_PIN, gpioModeInput, 0);

    // UART TX as input
    GPIO_PinModeSet(USR_UART_TX_PORT, USR_UART_TX_PIN, gpioModeInput, 0);

    uart_write("HIGH Z\n");

    jtag_pins_are_high_z = 1;

    // Enable interrupts on SWD & SWCLK
    GPIO_IntClear(1 << SWCLK_PIN);
    GPIO_IntClear(1 << SWDIO_PIN);
    GPIO_IntConfig(SWCLK_PORT, SWCLK_PIN, true, true, true);
    GPIO_IntConfig(SWDIO_PORT, SWDIO_PIN, true, true, true);
}

void jtag_pins_active()
{
    if (!jtag_pins_are_high_z)
    {
        return;
    }

    // Disable interrupts
    GPIO_IntConfig(SWCLK_PORT, SWCLK_PIN, true, true, false);
    GPIO_IntConfig(SWDIO_PORT, SWDIO_PIN, true, true, false);

    // GPIO prepare for JTAG
    GPIO_PinModeSet(TRST_PORT, TRST_PIN, gpioModePushPull, 1);
    GPIO_PinModeSet(SRST_PORT, SRST_PIN, gpioModePushPull, 0);

    GPIO_PinModeSet(TMS_PORT, TMS_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(TCK_PORT, TCK_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(TDI_PORT, TDI_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(TDO_PORT, TDO_PIN, gpioModeInput, 0);

    // Enable UART TX pin
    GPIO_PinModeSet(USR_UART_TX_PORT, USR_UART_TX_PIN, gpioModePushPull, 1);

//    // Enable target buffer
//    GPIO_PinOutSet(TARGET_BUF_EN_PORT, TARGET_BUF_EN_PIN);

    // Reset!!!
    //pin_reset();

    uart_write("ACTIVE\n");
    jtag_pins_are_high_z = 0;
}

static inline void send_pin_state(uint32_t rtc_timestamp)
{
    uint32_t pin_state = 0;
    pin_state += GPIO_PinInGet(SWCLK_PORT, SWCLK_PIN) ? 1: 0;
    pin_state += GPIO_PinInGet(SWDIO_PORT, SWDIO_PIN) ? 2: 0;

    usbpowercon_pinstatechange(rtc_timestamp, pin_state);
}



static inline void gpio_irq_handler()
{
    uint32_t rtc = RTC_CounterGet();
    if (GPIO_IntGetEnabled() & (1 << POWER_ALERT_PIN))
    {
        /* clear flag for PA6 interrupt */
        GPIO_IntClear(1 << POWER_ALERT_PIN);
        jaguar_power_alert(rtc);
    }

    if (GPIO_IntGetEnabled() & (1 << SWDIO_PIN))
    {
        GPIO_IntClear(1 << SWDIO_PIN);
        send_pin_state(rtc);
    }
    if (GPIO_IntGetEnabled() & (1 << SWCLK_PIN))
    {
        GPIO_IntClear(1 << TCK_PIN);
        send_pin_state(rtc);
    }
}

void GPIO_EVEN_IRQHandler()
{
    gpio_irq_handler();
}

void GPIO_ODD_IRQHandler()
{
    gpio_irq_handler();
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
        USART_Tx(DBG_UART, *c);
    }
}
char print_buf[1024];

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/

int platform_init()
{
    SCB->VTOR = 0x4000;

    /* Chip errata */
    CHIP_Init();
    
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_CORELEDIV2);

    CMU_ClockEnable(cmuClock_GPIO, true);
    
    // Start RTC for power measure timing
    CMU_ClockEnable(cmuClock_RTC, true);
    RTC_Init_TypeDef rtc_init = RTC_INIT_DEFAULT;
    rtc_init.comp0Top = false;
    RTC_Init(&rtc_init);
    
    SerialPortInit();
    
    uart_printf("CDC Test Started rtc freq %uHz, core clock %uHz\n",
            CMU_ClockFreqGet(cmuClock_RTC), CMU_ClockFreqGet(cmuClock_CORE));

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);

    /* Setup SysTick Timer for 100 msec interrupts  */
    SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 10);

    // LEDS GPIO init
    GPIO_PinModeSet(LED_IDLE_RUN_PORT, LED_IDLE_RUN_PIN, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(LED_ERROR_PORT, LED_ERROR_PIN, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(LED_UART_PORT, LED_UART_PIN, gpioModePushPull, 1);
    
//    // Configure target buffer
//    GPIO_PinModeSet(TARGET_BUF_EN_PORT, TARGET_BUF_EN_PIN, gpioModePushPull, 1);
//    GPIO_PinOutClear(TARGET_BUF_EN_PORT, TARGET_BUF_EN_PIN);
    
    // GPIO prepare for JTAG
    jtag_pins_active();
    jaguar_init();

    // Start with 3.3V, 5V activated
    jaguar_target_select_voltage(JAGUAR_VOLTAGE_3p3);
    jaguar_target_select_vdd_voltage(JAGUAR_VDD_VOLTAGE_3p3);
    jaguar_target_5V(1);

    // Initialize USB UART
    usbuart_init(USR_UART, USART_ROUTE_LOCATION_LOC1, DMAREQ_USART1_TXBL,
            DMAREQ_USART1_RXDATAV);

    // Initialize USB Power measure
    usbpowercon_init();

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
        usbuart_stop();
    }

    else if (newState == USBD_STATE_SUSPENDED)
    {
        /* We have been suspended, stop CDC functionality */
        /* Reduce current consumption to below 2.5 mA.    */
        uart_printf("\t\t->Stopped!!\n");
        usb_configured = 0;
        usbuart_stop();
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
                if ((setup->wValue == 0) && (setup->wIndex == 0) && /* Interface no. 0 */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction == USB_SETUP_DIR_IN))
                {
                    /* Send current settings to USB host. */
                    USBD_Write(0, (void*) &cdcLineCoding, 7, NULL);
                    retVal = USB_STATUS_OK;
                }
                else if ((setup->wValue == 0) && (setup->wIndex == 2) && /* Interface no. 2 */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction == USB_SETUP_DIR_IN))
                {
                    /* Send current settings to USB host. */
                    USBD_Write(0, (void*) &usbuart_cdclinecoding, 7, NULL);
                    retVal = USB_STATUS_OK;
                }
                else if ((setup->wValue == 0) && (setup->wIndex == 4) && /* Interface no. 2 */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction == USB_SETUP_DIR_IN))
                {
                    /* Send current settings to USB host. */
                    USBD_Write(0, (void*) &usbpowercon_cdclinecoding, 7, NULL);
                    retVal = USB_STATUS_OK;
                }
                break;

            case USB_CDC_SETLINECODING:
                /********************/
                if ((setup->wValue == 0) && (setup->wIndex == 0) && /* Interface no. */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction != USB_SETUP_DIR_IN))
                {
                    /* Get new settings from USB host. */
                    USBD_Read(0, (void*) &cdcLineCoding, 7, NULL);
                    retVal = USB_STATUS_OK;
                }
                else if ((setup->wValue == 0) && (setup->wIndex == 2) && /* Interface no. */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction != USB_SETUP_DIR_IN))
                {
                    /* Get new settings from USB host. */
                    USBD_Read(0, (void*) &usbuart_cdclinecoding, 7,
                            usbuart_LineCodingReceived);
                    retVal = USB_STATUS_OK;
                }
                else if ((setup->wValue == 0) && (setup->wIndex == 4) && /* Interface no. */
                (setup->wLength == 7) && /* Length of cdcLineCoding  */
                (setup->Direction != USB_SETUP_DIR_IN))
                {
                    /* Get new settings from USB host. */
                    USBD_Read(0, (void*) &usbpowercon_cdclinecoding, 7, NULL);
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

                    retVal = USB_STATUS_OK;
                }
                else if ((setup->wIndex == 2) && /* Interface no.  */
                (setup->wLength == 0)) /* No data        */
                {
                    int uart_dtr = setup->wValue & 1;
                    uart_printf("USB UART DTR = %u\n", uart_dtr);

                    if (uart_dtr)
                    {
                        usbuart_start();
                    }
                    else
                    {
                        usbuart_stop();
                    }

                    retVal = USB_STATUS_OK;
                }
                else if ((setup->wIndex == 4) && /* Interface no.  */
                (setup->wLength == 0)) /* No data        */
                {
                    int dtr = setup->wValue & 1;
                    uart_printf("USB POWERCON DTR = %u\n", dtr);

                    if (dtr)
                    {
                        usbpowercon_start();
                    }
                    else
                    {
                        usbpowercon_stop();
                    }

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
    // Init Debug UART

    USART_TypeDef *uart = DBG_UART;
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

    /* Configure GPIO pins */
    CMU_ClockEnable(cmuClock_GPIO, true);
    /* To avoid false start, configure output as high */
    GPIO_PinModeSet(DBG_UART_TX_PORT, DBG_UART_TX_PIN, gpioModePushPull, 1);
    GPIO_PinModeSet(DBG_UART_RX_PORT, DBG_UART_RX_PIN, gpioModeInput, 0);

    /* Enable peripheral clocks */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_USART0, true);

    /* Configure UART for basic async operation */
    init.enable = usartDisable;
    init.baudrate = 500000;
    USART_InitAsync(uart, &init);

    /* Enable pins at USART0 location #5 */
    uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN
            | USART_ROUTE_LOCATION_LOC5;

    /* Finally enable it */
    USART_Enable(uart, usartEnable);

    // Init pins and clock for JTAG uart
    GPIO_PinModeSet(USR_UART_TX_PORT, USR_UART_TX_PIN, gpioModePushPull, 1);
    GPIO_PinModeSet(USR_UART_RX_PORT, USR_UART_RX_PIN, gpioModeInput, 0);
    CMU_ClockEnable(cmuClock_USART1, true);
}
