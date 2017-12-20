/*
 * audio.h
 *
 *  Created on: Dec 19, 2017
 *      Author: parallels
 */

#ifndef AUDIO_AUDIO_H_
#define AUDIO_AUDIO_H_

#define N_MS N_MS_PER_INTERRUPT

#define AUDIO_CHANNELS 1
#define AUDIO_SAMPLING_FREQUENCY 32000

#if (AUDIO_SAMPLING_FREQUENCY == 8000)
#define MAX_DECIMATION_FACTOR 160
#else
#define MAX_DECIMATION_FACTOR 128
#endif

#include "stm32l4xx_hal.h"
#include "x_nucleo_cca02m1_audio_l4.h"
#include "fatfs.h"
#include "stdbool.h"

uint8_t audio_init(void);
uint8_t audio_record(FIL *file, uint32_t millis, bool append);

#endif /* AUDIO_AUDIO_H_ */
