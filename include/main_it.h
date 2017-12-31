/*
 * main_it.h
 *
 *  Created on: Dec 28, 2017
 *      Author: parallels
 */

#ifndef MAIN_IT_H_
#define MAIN_IT_H_

#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

void HAL_GPIO_EXTI_Callback(uint16_t pin);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c);


#endif /* MAIN_IT_H_ */
