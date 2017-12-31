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

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern SD_HandleTypeDef hsd1;


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

//	RTC_TimeTypeDef time;
//	RTC_DateTypeDef date;
//	HAL_StatusTypeDef stat2;
//
//	stat2 = HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
//	stat2 = HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	uint32_t cnt = 0;

	do
	{
		cnt++;
		printf("%u\n", cnt);
		HAL_Delay(1000);
		sol_stat = gps_solution(&sol);
	} while(sol_stat == GPS_SOL_NONE);


	//	while(1)
	//	{
	//		printf("START\n");
	//		gps_start();
	//		HAL_Delay(10000);
	//		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	//		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
	//		printf("@ %02u:%02u", time.Hours, time.Minutes);
	//		HAL_Delay(5000);
	//		printf("STOP\n");
	//		gps_stop();
	//		HAL_Delay(2000);
	//	}

	FATFS SDFatFs;  /* File system object for SD card logical drive */
	FIL gps_file, audio_file;     /* File object */

	if(f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0) != FR_OK) Error_Handler();

	if(f_mkfs((TCHAR const*)SD_Path, 0, 0) != FR_OK) Error_Handler();

	if(f_open(&gps_file, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) Error_Handler();
	f_close(&gps_file);

	/*##-11- Unlink the RAM disk I/O driver ####################################*/
	FATFS_UnLinkDriver(SD_Path);
}
