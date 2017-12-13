/**
  ******************************************************************************
  * @file    stm32xx_it.h 
  * @author  Central Labs
  * @version 2.0.1
  * @date    07-July-2017
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32xx_IT_H
#define __STM32xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
//#include "cube_hal.h"
#include "audio_hal.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
   
//void NMI_Handler(void);
//void HardFault_Handler(void);
//void SVC_Handler(void);
//void DebugMon_Handler(void);
//void PendSV_Handler(void);
//void SysTick_Handler(void);
void USB_IRQHandler(void);

void AUDIO_IN_DFSDM_DMA_1st_CH_IRQHandler(void);
void AUDIO_IN_DFSDM_DMA_2nd_CH_IRQHandler(void);
void AUDIO_IN_DFSDM_DMA_3rd_CH_IRQHandler(void);
void AUDIO_IN_DFSDM_DMA_4th_CH_IRQHandler(void);
void AUDIO_IN_I2S_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
