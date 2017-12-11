/*
 * sd_log.h
 *
 *  Created on: Dec 8, 2017
 *      Author: parallels
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "ff_gen_drv.h"
#include "stdbool.h"

typedef uint16_t wave_sample_t;

typedef enum
{
	LOGGER_OK = 0,
	LOGGER_ERR = 1,
	LOGGER_FATFS_ERR =2
} logger_result_t;

logger_result_t logger_wav_write_header(FIL *file, uint32_t sample_rate, uint16_t num_channels, uint32_t num_samples);
logger_result_t logger_wav_append_nchannels(FIL* file, uint16_t num_channels, uint32_t num_samples, wave_sample_t* samples[]);


#endif /* LOGGER_H_ */
