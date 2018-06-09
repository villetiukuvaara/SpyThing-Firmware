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
#include "gpx.h"
#include "locale.h"
#include "jsmn.h"
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern SD_HandleTypeDef hsd1;
extern RTC_HandleTypeDef hrtc;

#define SETTINGS_STRING_LENGTH 16
#define LOG_FILE "spything.log"
#define LOG(x) log_write(LOG_FILE "\n", x)
#define ERROR_HANDLER_LOG(x) do { LOG(x); Error_Handler(); } while(0);

typedef struct
{
	int32_t lat;
	int32_t lon;
} coordinate_t;

typedef enum
{
	STRING,
	UINT8,
	UINT16,
	UINT32,
	INT8,
	INT16,
	INT32,
	COORDINATE,
	BOOL
} setting_type_t;

typedef struct
{
	const char *key;
	void *value;
	setting_type_t type;
} setting_t;

typedef struct
{
	uint32_t audio_time;
	uint32_t interval;
	coordinate_t geofence_positions[4];
	uint32_t geofence_radii[4];
	uint8_t n_positions;
	bool assist_time;
	bool assist_pos;
	uint8_t gpx_prefix[16];
	uint8_t wav_prefix[16];
} settings_t;
settings_t settings_loaded;

gps_geofence_t geofences[4];

#define NEW_SETTING(key, value, type) { key, value, type }

const setting_t settings[] =
{
		NEW_SETTING("audio_time", &settings_loaded.audio_time, UINT32),
		NEW_SETTING("interval", &settings_loaded.interval, UINT32),
		NEW_SETTING("pos0", &settings_loaded.geofence_positions[0], COORDINATE),
		NEW_SETTING("pos1", &settings_loaded.geofence_positions[1], COORDINATE),
		NEW_SETTING("pos2", &settings_loaded.geofence_positions[2], COORDINATE),
		NEW_SETTING("pos3", &settings_loaded.geofence_positions[3], COORDINATE),
		NEW_SETTING("radius0", &settings_loaded.geofence_radii[0], UINT32),
		NEW_SETTING("radius1", &settings_loaded.geofence_radii[1], UINT32),
		NEW_SETTING("radius2", &settings_loaded.geofence_radii[2], UINT32),
		NEW_SETTING("radius3", &settings_loaded.geofence_radii[3], UINT32),
		NEW_SETTING("n_pos", &settings_loaded.n_positions, UINT8),
		NEW_SETTING("assist_time", &settings_loaded.assist_time, BOOL),
		NEW_SETTING("assist_pos", &settings_loaded.assist_pos, BOOL),
		NEW_SETTING("gpx_prefix", &settings_loaded.gpx_prefix, STRING),
		NEW_SETTING("wav_prefix", &settings_loaded.wav_prefix, STRING)
};

void stop(void);
bool load_settings(void);
bool log_write(char *path, char *string);

void application(void)
{
	RetargetInit(&huart4);

	FATFS SDFatFs;  /* File system object for SD card logical drive */
	FIL gpx_file, wav_file;     /* File object */
	FRESULT fr;

	while(!BSP_SD_IsDetected()) HAL_Delay(100);

	for(uint8_t i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
	}

	if(f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0) != FR_OK) ERROR_HANDLER_LOG("Failed to mount SD Card");

	if(!load_settings()) ERROR_HANDLER_LOG("Failed to mount load settings");

	if(gps_initialize(&huart3, &hi2c1, &hrtc) != GPS_OK) ERROR_HANDLER_LOG("Failed to initialize GPS");

	if(gps_set_geofences(geofences, settings_loaded.n_positions) != GPS_OK) ERROR_HANDLER_LOG("Failed to set geofences");

	if(gps_save_settings() != GPS_OK) ERROR_HANDLER_LOG("Failed to save GPS settings");

	bool run = true;

	while(1)
	{
		printf("Waiting to start... ");
		while(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin) == GPIO_PIN_RESET);
		HAL_Delay(10); // Debouncing
		if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin) == GPIO_PIN_RESET) continue;
		printf("starting.\n");

		printf("Creating GPX file... ");

		char path[16];
		uint32_t file_cnt = 0;
		do
		{
			path[0] = '\0';
			sprintf(path, "%s%u.gpx", (char *)settings_loaded.gpx_prefix, file_cnt);
			fr = f_stat(path, NULL);
		} while(fr == FR_OK && file_cnt++ < 100);

		if(fr == FR_OK) ERROR_HANDLER_LOG("Too many GPX files");

		fr = f_open(&gpx_file, path, FA_WRITE | FA_CREATE_ALWAYS);
		if(fr != FR_OK) ERROR_HANDLER_LOG("Failed to open new GPX file");

		if(!gpx_start(&gpx_file)) ERROR_HANDLER_LOG("Failed to write GPX header");
		if(!gpx_trkseg_start(&gpx_file)) ERROR_HANDLER_LOG("Failed to start GPX track segment");

		printf("done.\n");

		gps_start();

		bool record_audio = false;
		uint32_t audio_length;

		run = true;

		while(run)
		{
			gps_sol_t sol;
			gps_data_status_t data_stat;

			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

			data_stat = gps_solution(&sol);

			RTC_DateTypeDef date;
			RTC_TimeTypeDef time;
			HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

			if(data_stat == GPS_DATA_NEW)
			{

				printf("(%li,%li) @ %02u:%02u\n",
									sol.lat, sol.lon, time.Hours, time.Minutes);

				gpx_append_trkpt(&gpx_file, &sol);


				while(gps_solution(&sol) == GPS_DATA_NEW)
				{
					gpx_append_trkpt(&gpx_file, &sol);
				}

				int8_t gf;
				if(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin) == GPIO_PIN_SET
						&& settings_loaded.audio_time > 0
						&& gps_get_geofence(&gf) == GPS_DATA_NEW)
				{
					if(gf >= 0 && !record_audio) // Inside geofence
					{
						// Start new audio file
						file_cnt = 0;
						do
						{
							path[0] = '\0';
							sprintf(path, "%s%u-%u.wav", settings_loaded.wav_prefix, gf, file_cnt);
							fr = f_stat(path, NULL);
						} while(fr == FR_OK && file_cnt++ < 100);

						if(fr == FR_OK)
						{
							LOG("Too many WAV files");
							settings_loaded.audio_time = 0;
						}
						else
						{
							fr = f_open(&wav_file, path, FA_WRITE | FA_CREATE_ALWAYS);
							if(fr != FR_OK) LOG("Failed to open new WAV file");
							else
							{
								audio_init();
								audio_record_init(&wav_file);
								audio_length = 0;
 								record_audio = true;
							}
						}
					}
				}
			}
			else printf("(---,---) @ %02u:%02u\n", time.Hours, time.Minutes);

			if(record_audio)
			{
				audio_record(settings_loaded.interval);
				audio_length += settings_loaded.interval;

				if(audio_length > settings_loaded.audio_time)
				{
					audio_record_end();
					record_audio = false;
					f_close(&wav_file);
				}
			}
			else
			{
				HAL_Delay(settings_loaded.interval/2);
				HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
				HAL_Delay(settings_loaded.interval/2);
			}

			if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin) == GPIO_PIN_RESET)
			{
				HAL_Delay(10); // Debouncing
				if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin) == GPIO_PIN_RESET)
				{
					if(!gpx_trkseg_end(&gpx_file)) ERROR_HANDLER_LOG("Failed to end track segment");

					if(!gpx_end(&gpx_file)) ERROR_HANDLER_LOG("Failed to end GPX file");

					f_close(&gpx_file);

					if(record_audio)
					{
						audio_record_end();
						record_audio = false;
						f_close(&wav_file);
					}

					HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
					run = false;
				}
			}
		}
	}

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

bool load_settings(void)
{
	memset(&settings_loaded, 0, sizeof(settings_t));
	settings_loaded.n_positions = 0;
	settings_loaded.audio_time = 0;
	settings_loaded.assist_pos = false;
	settings_loaded.assist_time = false;

	FRESULT stat;
	FIL file;

	stat = f_open(&file, "settings.jsn", FA_READ);
	if(stat != FR_OK) return false;

	stat = f_lseek(&file, 0);
	if(stat != FR_OK) return false;

	char json[512];
	UINT br;

	stat = f_read(&file, &json, 511, &br);
	if(stat != FR_OK) return false;
	json[br] = '\0';

	f_close(&file);

	jsmn_parser parser;
	jsmntok_t tokens[128];
	int16_t n_tokens;

	jsmn_init(&parser);
	n_tokens = jsmn_parse(&parser, (char*)json, strlen((char*)json), tokens, 128);

	if(n_tokens < 0) return false;

	uint8_t n_settings = sizeof(settings)/sizeof(setting_t);

	for(uint8_t i = 0; i < n_tokens; i++)
	{
		jsmntok_t *t = &tokens[i];
		if(t->type == JSMN_STRING)
		{
			char *start_ptr = json + t->start, *end_ptr = json + t->end;
			*end_ptr = '\0';
			uint8_t j;
			for(j = 0; j < n_settings; j++)
			{
				if(strcmp(settings[j].key, start_ptr) == 0)
				{
					break;
				}
			}

			if(j < n_settings) // Found match
			{
				if(++i >= n_tokens) Error_Handler();
				t = &tokens[i];
				start_ptr = json + t->start;
				end_ptr = json + t->end;
				*end_ptr = '\0';

				errno = 0;

				switch(settings[j].type)
				{
					case STRING:
						if(t->type != JSMN_STRING) return false;
						strncpy((char*)settings[j].value, json + t->start, SETTINGS_STRING_LENGTH);
						break;

					case UINT32:
						if(t->type != JSMN_PRIMITIVE) return false;
						uint32_t val_u32 = strtoul(start_ptr, &end_ptr, 10);
						if(end_ptr == start_ptr || errno != 0) return false;
						*((uint32_t*)settings[j].value) = val_u32;
						break;

					case INT32:
						if(t->type != JSMN_PRIMITIVE) return false;
						int32_t val_i32 = strtol(start_ptr, &end_ptr, 10);
						if(end_ptr == start_ptr || errno != 0) return false;
						*((int32_t*)settings[j].value) = val_i32;
						break;

					case UINT16:
						if(t->type != JSMN_PRIMITIVE) return false;
						uint16_t val_u16 = strtoul(start_ptr, &end_ptr, 10);
						if(end_ptr == start_ptr || errno != 0) return false;
						*((uint16_t*)settings[j].value) = val_u16;
						break;

					case INT16:
						if(t->type != JSMN_PRIMITIVE) return false;
						int16_t val_i16 = strtol(start_ptr, &end_ptr, 10);
						if(end_ptr == start_ptr || errno != 0) return false;
						*((int16_t*)settings[j].value) = val_i16;
						break;

					case UINT8:
						if(t->type != JSMN_PRIMITIVE) return false;
						uint8_t val_u8 = strtoul(start_ptr, &end_ptr, 10);
						if(end_ptr == start_ptr || errno != 0) return false;
						*((uint8_t*)settings[j].value) = val_u8;
						break;

					case INT8:
						if(t->type != JSMN_PRIMITIVE) return false;
						int8_t val_i8 = strtol(start_ptr, &end_ptr, 10);
						if(end_ptr == start_ptr || errno != 0) return false;
						*((int8_t*)settings[j].value) = val_i8;
						break;

					case BOOL:
						if(t->type != JSMN_PRIMITIVE) return false;
						if(*start_ptr == 't') *((bool*)settings[j].value) = true;
						else *((bool*)settings[j].value) = false;
						break;

					case COORDINATE:
						if(t->type != JSMN_ARRAY) return false;

						double lat_d, lon_d;
						int32_t lat_i, lon_i;

						if(++i >= n_tokens) return false;
						t = &tokens[i];
						start_ptr = json + t->start;
						end_ptr = json + t->end;
						if(t->type != JSMN_PRIMITIVE) return false;
						errno = 0;
						lat_d = strtod(start_ptr, &end_ptr);
						if(end_ptr == start_ptr || errno != 0) return false;
						lat_i = lat_d*1e7;
						((coordinate_t*)settings[j].value)->lat = lat_i;

						if(++i >= n_tokens) return false;
						t = &tokens[i];
						start_ptr = json + t->start;
						end_ptr = json + t->end;
						if(t->type != JSMN_PRIMITIVE) return false;
						errno = 0;
						lon_d = strtod(start_ptr, &end_ptr);
						if(end_ptr == start_ptr || errno != 0) return false;
						lon_i = lon_d*1e7;
						((coordinate_t*)settings[j].value)->lon = lon_i;
						break;

					default:
						return false;
				}
			}
		}
	}


	// Configure geofences
	for(uint8_t i = 0; i < settings_loaded.n_positions; i++)
	{
		geofences[i].lat = settings_loaded.geofence_positions[i].lat;
		geofences[i].lon = settings_loaded.geofence_positions[i].lon;
		geofences[i].radius = settings_loaded.geofence_radii[i]*1e2;
	}

	return true;
}

bool log_write(char *path, char *string)
{
	FIL file;
	FRESULT stat;
	bool ret_val = true;

	if(f_stat(path, NULL) != FR_OK)
	{
		stat = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
		if(stat != FR_OK) return false;
	}
	else
	{
		stat = f_open(&file, path, FA_WRITE);
		if(stat != FR_OK) ret_val = false;

		stat = f_lseek(&file, f_size(&file));
		if(stat != FR_OK)
		{
			f_close(&file);
			return false;
		}
	}

	UINT btw = strlen(string);
	UINT bw;

	stat = f_write(&file, string, btw, &bw);
	if(stat != FR_OK || bw != btw) ret_val = false;

	f_close(&file);
	return ret_val;
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
