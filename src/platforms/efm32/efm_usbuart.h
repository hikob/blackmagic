/*
 * efm_usbuart.h
 *
 *  Created on: Feb 10, 2014
 *      Author: burindes
 */

#ifndef EFM_USBUART_H_
#define EFM_USBUART_H_

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

/** External definition of the line coding */
extern cdcLineCoding_TypeDef usbuart_cdclinecoding;


/** External handling of Line Coding update */
int usbuart_LineCodingReceived(USB_Status_TypeDef status,
                              uint32_t xferred,
                              uint32_t remaining);

/** Initialize */
void usbuart_init(USART_TypeDef *uart, uint32_t dmareq_tx, uint32_t dmareq_rx);

/** Start USB/UART forward */
void usbuart_start();
/** Stop USB/UART forward */
void usbuart_stop();

#endif /* EFM_USBUART_H_ */
