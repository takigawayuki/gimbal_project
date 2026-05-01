#ifndef __USER_USART_H
#define __USER_USART_H

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

void ZDT_UART_RxStart(void);

#endif
