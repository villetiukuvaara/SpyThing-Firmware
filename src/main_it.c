/*
 * main_it.c
 *
 *  Created on: Dec 27, 2017
 *      Author: parallels
 */

#include "main_it.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include "gps.h"

extern I2C_HandleTypeDef hi2c1;

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if(pin == GPS_PIO6_Pin) gps_dio6_callback();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	if(hi2c == &hi2c1) gps_i2c_txcplt_callback();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	if(hi2c == &hi2c1) gps_i2c_rxcplt_callback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
	if(hi2c == &hi2c1) gps_i2c_error_callback();
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c)
{
	if(hi2c == &hi2c1) gps_i2c_abort_callback();
}
