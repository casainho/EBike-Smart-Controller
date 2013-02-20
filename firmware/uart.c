/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012, 2013.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "throttle.h"

void uart_init (void)
{
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);

  USART_Cmd(USART3, ENABLE);
}

void uart_send (unsigned char *buf, unsigned int len)
{
  unsigned int i;

  for (i = 0; i < len; i++)
  {
    /* Send one byte from USARTy to USARTz */
    USART_SendData(USART3, buf[i]);

    /* Loop until USARTy DR register is empty */
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) ;
  }
}
