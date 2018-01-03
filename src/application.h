/*
 * application.h
 *
 *  Created on: Dec 31, 2017
 *      Author: parallels
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

void application(void);

// Interrupt handlers
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *handle);

#endif /* APPLICATION_H_ */
