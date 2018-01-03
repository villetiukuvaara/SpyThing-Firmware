/*
 * application.c
 *
 *  Created on: Dec 31, 2017
 *      Author: parallels
 */

#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"
#include "retarget.h"
#include "logger.h"
#include "audio.h"
#include "gps.h"
#include <stdbool.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern SD_HandleTypeDef hsd1;
extern RTC_HandleTypeDef hrtc;

void stop(void);

void application(void)
{
	RetargetInit(&huart4);

	uint8_t newline = 30;
	while(newline-- > 0)
		printf("\n");

	if(gps_initialize(&huart3, &hi2c1, &hrtc) != GPS_OK) Error_Handler();

	gps_start();

	gps_sol_t sol;
	gps_status_t stat;
	gps_solution_status_t sol_stat;

	uint32_t cnt = 0;

	/*do
	{
		cnt++;
		printf("%u\n", cnt);
		HAL_Delay(1000);
		sol_stat = gps_solution(&sol);
	} while(sol_stat == GPS_SOL_NONE);*/

	FATFS SDFatFs;  /* File system object for SD card logical drive */
	FIL gps_file, audio_file;     /* File object */

	if(f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0) != FR_OK) Error_Handler();
	//if(f_mkfs((TCHAR const*)SD_Path, 0, 0) != FR_OK) Error_Handler();

	if(f_stat("gps.txt", NULL) != FR_OK)
	{
		if(f_open(&gps_file, "gps.txt", FA_CREATE_ALWAYS) != FR_OK) Error_Handler();
		f_close(&gps_file);
	}

	while(1)
	{
		sol_stat = gps_solution(&sol);
		//if(sol_stat != GPS_SOL_NONE)
		//{
			if(f_open(&gps_file, "gps.txt", FA_WRITE) != FR_OK) Error_Handler();
			if(f_lseek(&gps_file, f_size(&gps_file)) != FR_OK) Error_Handler();

			char wr[128];
			sprintf(wr, "(%li,%li) @ %02u:%02u\n",
					sol.lat, sol.lon, sol.hour, sol.min);

			UINT length = strlen(wr), bw = 0;

			if(f_write(&gps_file, wr, length, &bw) != FR_OK) Error_Handler();
			if(bw != length) Error_Handler();

			f_close(&gps_file);

			printf("Going to sleep\n");
			//gps_stop();
			stop();
			HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 5, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
			printf("Wake up");
			gps_start();
			HAL_Delay(5000);
		//}
	}

	/*##-11- Unlink the RAM disk I/O driver ####################################*/
	FATFS_UnLinkDriver(SD_Path);
}

void stop(void)
{
	HAL_DBGMCU_EnableDBGStopMode();

	HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(UART4_IRQn);

	HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
	HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
	HAL_NVIC_ClearPendingIRQ(SDMMC1_IRQn);
	HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	HAL_NVIC_ClearPendingIRQ(UART4_IRQn);


	__HAL_RCC_SDMMC1_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
	__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);
	//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 5, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	HAL_SuspendTick();

	HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);

	SystemClock_Config();

	HAL_ResumeTick();

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	__HAL_RCC_SDMMC1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
}

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


void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef* handle)
{
//	if(handle == &hrtc)
//	{
//		printf("Started back up\n");
//	}
}
