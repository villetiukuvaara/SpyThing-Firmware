/*
 * sdcard.h
 *
 *  Created on: Dec 9, 2017
 *      Author: parallels
 */

#ifndef SD_INCLUDE_SDCARD_H_
#define SD_INCLUDE_SDCARD_H_

#include "stm32l4xx_hal.h"

#define SD_DETECT_PIN                    GPIO_PIN_13
#define SD_DETECT_GPIO_PORT              GPIOC
#define __SD_DETECT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define SD_DETECT_IRQn                   EXTI15_10_IRQn
#define SD_Detect_IRQHandler             EXTI15_10_IRQnHandler

#define SD_DATATIMEOUT           ((uint32_t)100000000)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

/* DMA definitions for SD DMA transfer */
#define __DMAx_TxRx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE
#define SD_DMAx_Tx_STREAM                 DMA2_Channel4
#define SD_DMAx_Rx_STREAM                 DMA2_Channel4
#define SD_DMAx_Tx_IRQn                   DMA2_Channel4_IRQn
#define SD_DMAx_Rx_IRQn                   DMA2_Channel4_IRQn
#define SD_DMAx_Tx_IRQHandler             DMA2_Channel4_IRQHandler
#define SD_DMAx_Rx_IRQHandler             DMA2_Channel4_IRQHandler

HAL_StatusTypeDef sdcard_init(void);
HAL_StatusTypeDef sdcard_deinit(void);
HAL_StatusTypeDef sdcard_io_config(void);
HAL_StatusTypeDef sdcard_read_blocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef sdcard_write_blocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef sdcard_read_blocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
HAL_StatusTypeDef sdcard_Write_blocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
HAL_StatusTypeDef sdcard_erase(uint32_t StartAddr, uint32_t EndAddr);
//void sdcard_IRQHandler(void);
//void sdcard_DMA_Tx_IRQHandler(void);
//void sdcard_DMA_Rx_IRQHandler(void);
HAL_SD_CardStateTypedef sdcard_get_state(void);
void sdcard_get_info(HAL_SD_CardInfoTypeDef *CardInfo);
uint8_t sdcard_detect(void);

#endif /* SD_INCLUDE_SDCARD_H_ */
