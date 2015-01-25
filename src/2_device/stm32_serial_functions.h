/*
 * stm32_serial_functions.h
 *
 *  Created on: Mar 20, 2013
 *      Author: franz
 */

#ifndef STM32_SERIAL_FUNCTIONS_H_
#define STM32_SERIAL_FUNCTIONS_H_

#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <task.h>

#include <stdarg.h>
#include <stdio.h>

void USART_send(USART_TypeDef *usartNr, const char *msgPtr);
void USART_sendSuspend(USART_TypeDef *usartNr, const char *msgPtr, xTaskHandle taskHandle);
void USART_printf(USART_TypeDef *usartNr, char *buff,...);
void debug(USART_TypeDef *usartNr, const char *s,...);

#endif /* STM32_SERIAL_FUNCTIONS_H_ */
