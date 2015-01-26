/*
 * stm32_serial_functions.c
 *
 *  Created on: Mar 20, 2013
 *      Author: franz
 */

#include <stm32_serial_functions.h>


void USART_send(USART_TypeDef *usartNr, const char *msgPtr)
{
	while(*msgPtr)
	{
		USART_SendData(usartNr, *msgPtr++);
		while (USART_GetFlagStatus(usartNr, USART_FLAG_TC) != SET);
	}
}

/*
void USART_sendSuspend(USART_TypeDef *usartNr, const char *msgPtr, xTaskHandle taskHandle)
{
	while(*msgPtr)
	{
		// Enable Interrupt for USART TX. USART2 MUST NOT be used by another function or task!!!
		USART_ITConfig(usartNr, USART_IT_TC, ENABLE);
		USART_SendData(usartNr, *msgPtr++);
		vTaskSuspend( taskHandle );
	}
}
*/

void USART_printf(USART_TypeDef *usartNr, char *buff,...)
{
	char printBuffer[30];
	va_list arglist;

	va_start(arglist,buff);

	snprintf(printBuffer,30, buff, arglist);
	USART_send(usartNr, printBuffer);
	printf("Hallo%d\n", arglist);

	va_end(arglist);
}

void debug(USART_TypeDef *usartNr, const char *s,...)
{
    va_list va;
    va_start(va,s);

    char buffer[30];

    //sprintf(max,"error: "); vsprintf(max,s,va); sprintf(max,"\n");

    va_end(va);
    vsnprintf(buffer,30, s, va);
	USART_send(usartNr, buffer);
}


