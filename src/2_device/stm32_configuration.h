#ifndef STM32_CONFIGURATION_H
#define STM32_CONFIGURATION_H

#include <stm32f4xx.h>

#define ssc32_baudrate 38400

#define CLOCK_SPEED 100000


void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void EXTI5_Configuration(void);
void USART_Configuration(void);
void I2C1_Configuration(void);
void I2C2_Configuration(void);
void SPI_Configuration(uint8_t *pRXDMABuffer, uint8_t *pTXDMABuffer, uint8_t bufferSize);
void ADC_Configuration(__IO uint16_t *uhADCxConvertedValue);
void TIM3_PWM_Configuration(void);
void TIM12_PWM_Configuration(void);
void TIM_PWM_INPUT_Configuration(void);
void CAN_Configuration(void);
void CLI_Configuration(void);

#endif /*STM32_CONFIGURATION_H */
