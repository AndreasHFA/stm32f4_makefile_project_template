/*    Copyright (C) 2013 Hofer-Automation
 *
 *	  Author: Franz Flasch, Hofer Andreas
 *
 *    This file is part of Universal CNC-Controller AF-Series.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
//#include <stm32_configuration.h>
//#include <stm32_serial_functions.h>


/*
 *  STM4F407 Discoveryboard LED Pins
 *  LED Green 	->	PD12
 *  LED Orange	->	PD13
 *  LED Red		-> 	PD14
 *  LED Blue	->	PD15
 */


uint8_t SPIRxBuffer[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
//FIXME: Don't know why the TX buffersize has to 9 here? The DMA does not send the
//		 first byte. It begins with the second one. Maybe something is not correctly setup,
//		 or maybe I have to look in the errata sheet of the STM32F4.
uint8_t SPITxBuffer[] = {0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41};

/*
  GPIO_InitTypeDef  GPIO_InitStructure;
*/
  /* GPIOD Periph clock enable */
/*
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
*/

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
/*
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
*/

void Delay(__IO uint32_t nCount);

int main()
{

	RCC_Configuration();
	/*
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	CLI_Configuration();
*/
	//stepgen_reset();

//	SPI_Configuration(SPIRxBuffer, SPITxBuffer, sizeof(SPIRxBuffer));


	while(1)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		Delay(0x03FFFF);

		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		Delay(0x03FFFF);

		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		Delay(0x03FFFF);

		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		Delay(0x03FFFF);

		GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
		Delay(0x3FFFFF);
	}
}


void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
