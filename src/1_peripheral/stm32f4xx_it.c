/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32_configuration.h"
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  //TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

void USART2_IRQHandler (void)
{
//	static int8_t cRxedChar;
//	portBASE_TYPE xYieldRequired = pdFALSE;
//
//	if(USART_GetITStatus(USART2, USART_IT_TC) == SET)
//	{
//		GPIO_SetBits(GPIOD, GPIO_Pin_15);
//		USART_ITConfig(USART2, USART_IT_TC, DISABLE);
//	     // Resume the suspended task.
//	     xYieldRequired = xTaskResumeFromISR( usartControl.commandlineHandle );
//	}
//	else if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
//    {
//    	cRxedChar = USART_ReceiveData(USART2);
//    	xQueueSendFromISR( usartControl.xUsartRxQueue, &cRxedChar, &xYieldRequired );
//    	xTaskResumeFromISR( usartControl.commandlineHandle );
//    }
//
//	GPIO_ResetBits(GPIOD, GPIO_Pin_15);
//	USART_ClearITPendingBit(USART2, USART_IT_TC);
//
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//
//    if( xYieldRequired == pdTRUE )
//    {
//        // We should switch context so the ISR returns to a different task.
//        // NOTE:  How this is done depends on the port you are using.  Check
//        // the documentation and examples for your port.
//   	 taskYIELD();
//    }
}

void EXTI9_5_IRQHandler(void)
{

//  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
//  {
//	/* Toggle LED1 */
//	GPIO_SetBits(GPIOD, GPIO_Pin_13);
//
//	/* SPI configuration -------------------------------------------------------*/
//	SPI_I2S_DeInit(SPI2);
//	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
//	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
//	SPI_Cmd(SPI2, ENABLE);
//
//    /* Clear the EXTI line 0 pending bit */
//    EXTI_ClearITPendingBit(EXTI_Line5);
//  }

}

/*
 * SPI in Slave Receiver mode  - NOT USED
 * */
void SPI2_IRQHandler(void)
{
 /*
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
  {
	printf("%d\n\r", SPI_I2S_ReceiveData(SPI2));
  }
  */
}

void DMA1_Stream3_IRQHandler(void)
{
	//uint8_t i = 0;

	/*
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	printf("RXBuf:");
	for(i = 0;i<sizeof(SPIRxBuffer);i++)
	{
		printf(" %0x", SPIRxBuffer[i]);
	}
	printf("\n");
	*/

	TOGGLE_LED_RED();

	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);

	//CAN_sendBuffer(SPIRxBuffer);
}


void CAN1_RX0_IRQHandler(void)
{

}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
