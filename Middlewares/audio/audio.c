/*
 * audio.c
 *
 *  Created on: Dec 19, 2017
 *      Author: parallels
 */

#include "audio.h"
#include "logger.h"
#include "string.h"

#define AUDIO_PCM_BUFFER_LENGTH ((AUDIO_SAMPLING_FREQUENCY/1000)*AUDIO_CHANNELS * N_MS)
#define AUDIO_SD_BUFFER_LENGTH 8192

wave_sample_t PCM_buffer[AUDIO_PCM_BUFFER_LENGTH];
wave_sample_t SD_buffers[2][AUDIO_SD_BUFFER_LENGTH];
bool audio_ready = false;
uint32_t SD_buffer_pos = 0;
uint32_t SD_buffer_num = 0;


uint8_t audio_init(void)
{
	return BSP_AUDIO_IN_Init(AUDIO_SAMPLING_FREQUENCY, 16, 1);
}

uint8_t audio_record(FIL *file, uint32_t millis, bool append)
{
	// Expand file in advance to make writes faster
	FRESULT res = f_lseek(file, (AUDIO_SAMPLING_FREQUENCY/1000)*millis);
	if(res != FR_OK) return AUDIO_ERROR;

	res = f_lseek(file, 0);
	if(res != FR_OK) return AUDIO_ERROR;

	if(BSP_AUDIO_IN_Record((uint16_t*)PCM_buffer, 0) != AUDIO_OK) return AUDIO_ERROR;

	uint32_t start = HAL_GetTick();
	uint32_t total_samples = 0, current_samples = 0;

	while(HAL_GetTick() < start + millis)
	{
		if(audio_ready)
		{
			current_samples = SD_buffer_pos;
			SD_buffer_pos = 0;
			total_samples += current_samples;

			// Switch to the other buffer while writing out this buffer
			SD_buffer_num = (SD_buffer_num + 1)%2;
			audio_ready = false;

			if(logger_wav_append(file, current_samples, SD_buffers[SD_buffer_num]) != LOGGER_OK)
			{
				BSP_AUDIO_IN_Stop();
				return AUDIO_ERROR;
			}
		}
	}

	BSP_AUDIO_IN_Stop();
	if(logger_wav_write_header(file, AUDIO_SAMPLING_FREQUENCY, 1, total_samples) != LOGGER_OK) return AUDIO_ERROR;

	return AUDIO_OK;
}

/**
 * @brief  Half Transfer user callback, called by BSP functions.
 * @param  None
 * @retval None
 */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
	BSP_AUDIO_IN_TransferComplete_CallBack();
}

/**
 * @brief  Transfer Complete user callback, called by BSP functions.
 * @param  None
 * @retval None
 */
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
	if(audio_ready) return;

	if(SD_buffer_pos + AUDIO_PCM_BUFFER_LENGTH < AUDIO_SD_BUFFER_LENGTH)
	{
		memcpy(SD_buffers[SD_buffer_num] + SD_buffer_pos, PCM_buffer, AUDIO_PCM_BUFFER_LENGTH*sizeof(wave_sample_t));
		SD_buffer_pos += AUDIO_PCM_BUFFER_LENGTH;
	}

	if(SD_buffer_pos + AUDIO_PCM_BUFFER_LENGTH >= AUDIO_SD_BUFFER_LENGTH) audio_ready = true;
}

void BSP_AUDIO_IN_Error_Callback(void)
{
	Error_Handler();
}
