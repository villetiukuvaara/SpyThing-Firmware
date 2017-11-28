/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define GPS_I2C_ADDR 0x42

#define SD_CD_Pin GPIO_PIN_13
#define SD_CD_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOC
#define DEBUG_UART_TX_Pin GPIO_PIN_0
#define DEBUG_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_RX_Pin GPIO_PIN_1
#define DEBUG_UART_RX_GPIO_Port GPIOA
#define SWITCH_2_Pin GPIO_PIN_3
#define SWITCH_2_GPIO_Port GPIOA
#define GPS_UART_TX_Pin GPIO_PIN_4
#define GPS_UART_TX_GPIO_Port GPIOC
#define GPS_UART_RX_Pin GPIO_PIN_5
#define GPS_UART_RX_GPIO_Port GPIOC
#define GPS_EXTINT_Pin GPIO_PIN_2
#define GPS_EXTINT_GPIO_Port GPIOB
#define RADIO_DIO_2_Pin GPIO_PIN_10
#define RADIO_DIO_2_GPIO_Port GPIOB
#define RADIO_DIO_3_Pin GPIO_PIN_11
#define RADIO_DIO_3_GPIO_Port GPIOB
#define VUSB_DET_Pin GPIO_PIN_12
#define VUSB_DET_GPIO_Port GPIOB
#define PMIC_I2C_SCL_Pin GPIO_PIN_13
#define PMIC_I2C_SCL_GPIO_Port GPIOB
#define PMIC_I2C_SDA_Pin GPIO_PIN_14
#define PMIC_I2C_SDA_GPIO_Port GPIOB
#define GPS_TIMEPULSE_Pin GPIO_PIN_15
#define GPS_TIMEPULSE_GPIO_Port GPIOB
#define RADIO_DIO_4_Pin GPIO_PIN_6
#define RADIO_DIO_4_GPIO_Port GPIOC
#define RADIO_DIO_5_Pin GPIO_PIN_7
#define RADIO_DIO_5_GPIO_Port GPIOC
#define PMIC_FAST_Pin GPIO_PIN_9
#define PMIC_FAST_GPIO_Port GPIOA
#define GPS_RESET_N_Pin GPIO_PIN_4
#define GPS_RESET_N_GPIO_Port GPIOB
#define RADIO_RESET_N_Pin GPIO_PIN_5
#define RADIO_RESET_N_GPIO_Port GPIOB
#define RADIO_I2C_SCL_Pin GPIO_PIN_6
#define RADIO_I2C_SCL_GPIO_Port GPIOB
#define RADIO_I2C_SDA_Pin GPIO_PIN_7
#define RADIO_I2C_SDA_GPIO_Port GPIOB
#define RADIO_DIO_0_Pin GPIO_PIN_8
#define RADIO_DIO_0_GPIO_Port GPIOB
#define RADIO_DIO_1_Pin GPIO_PIN_9
#define RADIO_DIO_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
