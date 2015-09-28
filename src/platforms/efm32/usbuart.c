/*
 * usbuart.c
 *
 *  Created on: Feb 10, 2014
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

#include "platform.h"
#include "dmactrl.h"
#include "efm_usbuart.h"


/*** Function prototypes. ***/

static int UsbDataReceived(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining);
static void DmaSetup(void);
static void UartRxTimeout(void);

/*
 * The LineCoding variable must be 4-byte aligned as it is used as USB
 * transmit and receive buffer
 */
EFM32_ALIGN(4)
EFM32_PACK_START(1)
cdcLineCoding_TypeDef __attribute__ ((aligned(4))) usbuart_cdclinecoding =
{
    9600, 0, 0, 8, 0
};
EFM32_PACK_END()

/* Calculate a timeout in ms corresponding to 5 char times on current     */
/* baudrate. Minimum timeout is set to 10 ms.                             */
#define RX_TIMEOUT    EFM32_MAX(10U, 50000 / (usbuart_cdclinecoding.dwDTERate))

STATIC_UBUF( usbRxBuffer0, USB_RX_BUF_SIZ); /* USB receive buffers.   */
STATIC_UBUF( usbRxBuffer1, USB_RX_BUF_SIZ);
STATIC_UBUF( uartRxBuffer0, USB_TX_BUF_SIZ); /* UART receive buffers.  */
STATIC_UBUF( uartRxBuffer1, USB_TX_BUF_SIZ);

static const uint8_t *usbRxBuffer[2] =
{ usbRxBuffer0, usbRxBuffer1 };
static const uint8_t *uartRxBuffer[2] =
{ uartRxBuffer0, uartRxBuffer1 };

static int usbRxIndex, usbBytesReceived;
static int uartRxIndex, uartRxCount;
static int LastUsbTxCnt;

static bool dmaRxCompleted;
static bool usbRxActive, dmaTxActive;
static bool usbTxActive, dmaRxActive;

static DMA_CB_TypeDef DmaTxCallBack; /** DMA callback structures */
static DMA_CB_TypeDef DmaRxCallBack;

static USART_TypeDef *usbuart_uart;
static uint32_t usbuart_dmareq_tx;
static uint32_t usbuart_dmareq_rx;
static uint32_t usbuart_route;

void usbuart_init(USART_TypeDef *uart, uint32_t route, uint32_t dmareq_tx,
        uint32_t dmareq_rx)
{
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

    /* Configure UART for basic async operation */
    init.enable = usartDisable;
    init.baudrate = usbuart_cdclinecoding.dwDTERate;
    DEBUG("init: Setting JTAG UART %u bauds\n", init.baudrate);
    USART_InitAsync(uart, &init);

    /* Enable pins at USART1 location #1 */
    uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | route;

    /* Finally enable it */
    USART_Enable(uart, usartEnable);

    // Store
    usbuart_uart = uart;
    usbuart_dmareq_tx = dmareq_tx;
    usbuart_dmareq_rx = dmareq_rx;
    usbuart_route = route;

    // Setup
    DmaSetup();
}

void usbuart_start()
{
    /* Start receiving data from USB host. */
    usbRxIndex = 0;
    usbRxActive = true;
    dmaTxActive = false;
    USBD_Read(EP_UART_OUT, (void*) usbRxBuffer[usbRxIndex], USB_RX_BUF_SIZ,
            UsbDataReceived);

    /* Start receiving data on UART. */
    uartRxIndex = 0;
    LastUsbTxCnt = 0;
    uartRxCount = 0;
    dmaRxActive = true;
    usbTxActive = false;
    dmaRxCompleted = true;
    DMA_ActivateBasic(1, true, false, (void *) uartRxBuffer[uartRxIndex],
            (void *) &(usbuart_uart->RXDATA), USB_TX_BUF_SIZ - 1);
    USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);

    led_uart_on();
}

void usbuart_stop()
{
    /* Stop CDC functionality */
    USBTIMER_Stop(0);
    DMA->CHENC = 3; /* Stop DMA channels 0 and 1. */

    led_uart_off();
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
        usbRxIndex ^= 1;

        if (!dmaTxActive)
        {
            /* dmaTxActive = false means that a new UART Tx DMA can be started. */
            dmaTxActive = true;
            DMA_ActivateBasic(0, true, false, (void *) &(usbuart_uart->TXDATA),
                    (void *) usbRxBuffer[usbRxIndex ^ 1], xferred - 1);

            /* Start a new USB receive transfer. */
            USBD_Read(EP_UART_OUT, (void*) usbRxBuffer[usbRxIndex],
                    USB_RX_BUF_SIZ, UsbDataReceived);
        }
        else
        {
            /* The UART transmit DMA callback function will start a new DMA. */
            usbRxActive = false;
            usbBytesReceived = xferred;
        }
    }
    return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief Callback function called whenever a UART transmit DMA has completed.
 *
 * @param[in] channel DMA channel number.
 * @param[in] primary True if this is the primary DMA channel.
 * @param[in] user    Optional user supplied parameter.
 *****************************************************************************/
static void DmaTxComplete(unsigned int channel, bool primary, void *user)
{
    (void) channel; /* Unused parameter */
    (void) primary; /* Unused parameter */
    (void) user; /* Unused parameter */

    /*
     * As nested interrupts may occur and we rely on variables usbRxActive
     * and dmaTxActive etc, we must handle this function as a critical region.
     */
    INT_Disable();

    if (!usbRxActive)
    {
        /* usbRxActive = false means that an USB receive packet has been received.*/
        DMA_ActivateBasic(0, true, false, (void *) &(usbuart_uart->TXDATA),
                (void *) usbRxBuffer[usbRxIndex ^ 1], usbBytesReceived - 1);

        /* Start a new USB receive transfer. */
        usbRxActive = true;
        USBD_Read(EP_UART_OUT, (void*) usbRxBuffer[usbRxIndex], USB_RX_BUF_SIZ,
                UsbDataReceived);
    }
    else
    {
        /* The USB receive complete callback function will start a new DMA. */
        dmaTxActive = false;
    }

    INT_Enable();
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
        if (!dmaRxActive)
        {
            /* dmaRxActive = false means that a new UART Rx DMA can be started. */

            USBD_Write(EP_UART_IN, (void*) uartRxBuffer[uartRxIndex ^ 1],
                    uartRxCount, UsbDataTransmitted);
            LastUsbTxCnt = uartRxCount;

            dmaRxActive = true;
            dmaRxCompleted = true;
            DMA_ActivateBasic(1, true, false,
                    (void *) uartRxBuffer[uartRxIndex],
                    (void *) &(usbuart_uart->RXDATA), USB_TX_BUF_SIZ - 1);
            uartRxCount = 0;
            USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);
        }
        else
        {
            /* The UART receive DMA callback function will start a new DMA. */
            usbTxActive = false;
        }
    }
    return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief Callback function called whenever a UART receive DMA has completed.
 *
 * @param[in] channel DMA channel number.
 * @param[in] primary True if this is the primary DMA channel.
 * @param[in] user    Optional user supplied parameter.
 *****************************************************************************/
static void DmaRxComplete(unsigned int channel, bool primary, void *user)
{
    (void) channel; /* Unused parameter */
    (void) primary; /* Unused parameter */
    (void) user; /* Unused parameter */

    /*
     * As nested interrupts may occur and we rely on variables usbTxActive
     * and dmaRxActive etc, we must handle this function as a critical region.
     */
    INT_Disable();

    uartRxIndex ^= 1;

    if (dmaRxCompleted)
    {
        uartRxCount = USB_TX_BUF_SIZ;
    }
    else
    {
        uartRxCount = USB_TX_BUF_SIZ - 1
                - ((dmaControlBlock[1].CTRL & _DMA_CTRL_N_MINUS_1_MASK)
                        >> _DMA_CTRL_N_MINUS_1_SHIFT);
    }

    if (!usbTxActive)
    {
        /* usbTxActive = false means that a new USB packet can be transferred. */
        usbTxActive = true;
        USBD_Write(EP_UART_IN, (void*) uartRxBuffer[uartRxIndex ^ 1],
                uartRxCount, UsbDataTransmitted);
        LastUsbTxCnt = uartRxCount;
        led_uart_toggle();

        /* Start a new UART receive DMA. */
        dmaRxCompleted = true;
        DMA_ActivateBasic(1, true, false, (void *) uartRxBuffer[uartRxIndex],
                (void *) &(usbuart_uart->RXDATA), USB_TX_BUF_SIZ - 1);
        uartRxCount = 0;
        USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);
    }
    else
    {
        /* The USB transmit complete callback function will start a new DMA. */
        dmaRxActive = false;
        USBTIMER_Stop(0);
    }

    INT_Enable();
}

/**************************************************************************//**
 * @brief
 *   Called each time UART Rx timeout period elapses.
 *   Implements UART Rx rate monitoring, i.e. we must behave differently when
 *   UART Rx rate is slow e.g. when a person is typing characters, and when UART
 *   Rx rate is maximum.
 *****************************************************************************/
static void UartRxTimeout(void)
{
    int cnt;
    uint32_t dmaCtrl;

    dmaCtrl = dmaControlBlock[1].CTRL;

    /* Has the DMA just completed ? */
    if ((dmaCtrl & _DMA_CTRL_CYCLE_CTRL_MASK) == DMA_CTRL_CYCLE_CTRL_INVALID)
    {
        return;
    }

    cnt =
            USB_TX_BUF_SIZ - 1
                    - ((dmaCtrl & _DMA_CTRL_N_MINUS_1_MASK)
                            >> _DMA_CTRL_N_MINUS_1_SHIFT);

    if ((cnt == 0) && (LastUsbTxCnt == BULK_EP_SIZE))
    {
        /*
         * No activity on UART Rx, send a ZERO length USB package if last USB
         * USB package sent was BULK_EP_SIZE (max. EP size) long.
         */
        DMA->CHENC = 2; /* Stop DMA channel 1 (2 = 1 << 1). */
        dmaRxCompleted = false;
        DmaRxComplete(1, true, NULL); /* Call DMA completion callback.    */
        return;
    }

    if ((cnt > 0) && (cnt == uartRxCount))
    {
        /*
         * There is curently no activity on UART Rx but some chars have been
         * received. Stop DMA and transmit the chars we have got so far on USB.
         */
        DMA->CHENC = 2; /* Stop DMA channel 1 (2 = 1 << 1). */
        dmaRxCompleted = false;
        DmaRxComplete(1, true, NULL); /* Call DMA completion callback.    */
        return;
    }

    /* Restart timer to continue monitoring. */
    uartRxCount = cnt;
    USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a CDC_SET_LINECODING
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK if data accepted.
 *         USB_STATUS_REQ_ERR if data calls for modes we can not support.
 *****************************************************************************/
int usbuart_LineCodingReceived(USB_Status_TypeDef status, uint32_t xferred,
        uint32_t remaining)
{
    (void) remaining;

    /* We have received new serial port communication settings from USB host */
    if ((status == USB_STATUS_OK) && (xferred == 7))
    {
        USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

        /* Configure UART for basic async operation */
        init.enable = usartDisable;
        init.baudrate = usbuart_cdclinecoding.dwDTERate;

        /* Check bDataBits, valid values are: 5, 6, 7, 8 or 16 bits */
        if (usbuart_cdclinecoding.bDataBits == 5)
            init.databits = usartDatabits5;

        else if (usbuart_cdclinecoding.bDataBits == 6)
            init.databits = usartDatabits6;

        else if (usbuart_cdclinecoding.bDataBits == 7)
            init.databits = usartDatabits7;

        else if (usbuart_cdclinecoding.bDataBits == 8)
        {
            init.databits = usartDatabits8;
        }

        else if (usbuart_cdclinecoding.bDataBits == 16)
            init.databits = usartDatabits16;

        else
            return USB_STATUS_REQ_ERR;

        /* Check bParityType, valid values are: 0=None 1=Odd 2=Even 3=Mark 4=Space  */
        if (usbuart_cdclinecoding.bParityType == 0)
        {
            init.parity = usartNoParity;
        }

        else if (usbuart_cdclinecoding.bParityType == 1)
        {
            init.parity = usartOddParity;
        }

        else if (usbuart_cdclinecoding.bParityType == 2)
        {
            init.parity = usartEvenParity;
        }

        else if (usbuart_cdclinecoding.bParityType == 3)
        {
            return USB_STATUS_REQ_ERR;
        }

        else if (usbuart_cdclinecoding.bParityType == 4)
        {
            return USB_STATUS_REQ_ERR;
        }

        else
        {
            return USB_STATUS_REQ_ERR;
        }

        /* Check bCharFormat, valid values are: 0=1 1=1.5 2=2 stop bits */
        if (usbuart_cdclinecoding.bCharFormat == 0)
        {
            init.stopbits = usartStopbits1;
        }
        else if (usbuart_cdclinecoding.bCharFormat == 1)
        {
            init.stopbits = usartStopbits1p5;
        }

        else if (usbuart_cdclinecoding.bCharFormat == 2)
        {
            init.stopbits = usartStopbits2;
        }

        else
        {
            return USB_STATUS_REQ_ERR;
        }

        usbuart_stop();

        DEBUG("set: Setting JTAG UART %u bauds\n", init.baudrate);
        USART_InitAsync(usbuart_uart, &init);

        /* Enable pins at USART1 location #1 */
        usbuart_uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | usbuart_route;

        /* Finally enable it */
        USART_Enable(usbuart_uart, usartEnable);

        usbuart_start();

        return USB_STATUS_OK;
    }
    return USB_STATUS_REQ_ERR;
}

/**************************************************************************//**
 * @brief Initialize the DMA peripheral.
 *****************************************************************************/
static void DmaSetup(void)
{
    /* DMA configuration structs */
    DMA_Init_TypeDef dmaInit;
    DMA_CfgChannel_TypeDef chnlCfgTx, chnlCfgRx;
    DMA_CfgDescr_TypeDef descrCfgTx, descrCfgRx;

    /* Initialize the DMA */
    dmaInit.hprot = 0;
    dmaInit.controlBlock = dmaControlBlock;
    DMA_Init(&dmaInit);

    /*---------- Configure DMA channel 0 for UART Tx. ----------*/

    /* Setup the interrupt callback routine */
    DmaTxCallBack.cbFunc = DmaTxComplete;
    DmaTxCallBack.userPtr = NULL;

    /* Setup the channel */
    chnlCfgTx.highPri = false; /* Can't use with peripherals */
    chnlCfgTx.enableInt = true; /* Interrupt needed when buffers are used */
    chnlCfgTx.select = usbuart_dmareq_tx; //DMAREQ_UART1_TXBL;
    chnlCfgTx.cb = &DmaTxCallBack;
    DMA_CfgChannel(0, &chnlCfgTx);

    /* Setup channel descriptor */
    /* Destination is UART Tx data register and doesn't move */
    descrCfgTx.dstInc = dmaDataIncNone;
    descrCfgTx.srcInc = dmaDataInc1;
    descrCfgTx.size = dmaDataSize1;

    /* We have time to arbitrate again for each sample */
    descrCfgTx.arbRate = dmaArbitrate1;
    descrCfgTx.hprot = 0;

    /* Configure primary descriptor. */
    DMA_CfgDescr(0, true, &descrCfgTx);

    /*---------- Configure DMA channel 1 for UART Rx. ----------*/

    /* Setup the interrupt callback routine */
    DmaRxCallBack.cbFunc = DmaRxComplete;
    DmaRxCallBack.userPtr = NULL;

    /* Setup the channel */
    chnlCfgRx.highPri = false; /* Can't use with peripherals */
    chnlCfgRx.enableInt = true; /* Interrupt needed when buffers are used */
    chnlCfgRx.select = usbuart_dmareq_rx; //DMAREQ_UART1_RXDATAV;
    chnlCfgRx.cb = &DmaRxCallBack;
    DMA_CfgChannel(1, &chnlCfgRx);

    /* Setup channel descriptor */
    /* Source is UART Rx data register and doesn't move */
    descrCfgRx.dstInc = dmaDataInc1;
    descrCfgRx.srcInc = dmaDataIncNone;
    descrCfgRx.size = dmaDataSize1;

    /* We have time to arbitrate again for each sample */
    descrCfgRx.arbRate = dmaArbitrate1;
    descrCfgRx.hprot = 0;

    /* Configure primary descriptor. */
    DMA_CfgDescr(1, true, &descrCfgRx);
}

