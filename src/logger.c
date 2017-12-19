#include "logger.h"

#define WAV_DATA_OFFSET (uint32_t)44
#define BYTES_PER_SAMPLE (uint16_t)2
#define BITS_PER_SAMPLE (uint16_t)16

static const uint8_t WAV_HEADER_1[] = {0x52, 0x49, 0x46, 0x46};
static const uint8_t WAV_HEADER_2[] = {0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20};
static const uint8_t WAV_HEADER_3[] = {0x10, 0x00, 0x00, 0x00, 0x01, 0x00};
static const uint8_t WAV_HEADER_4[] = {0x64, 0x61, 0x74, 0x61};

logger_result_t logger_wav_write_header(FIL *file, uint32_t sample_rate, uint16_t num_channels, uint32_t num_samples)
{
	FRESULT fr;
	UINT bw;

	uint32_t subchunk_2_size = num_samples*num_channels*BYTES_PER_SAMPLE;
	uint32_t chunk_size = 36 + subchunk_2_size;
	uint32_t byte_rate = sample_rate*num_channels*BYTES_PER_SAMPLE;
	uint16_t block_align = num_channels*BYTES_PER_SAMPLE;
	uint32_t bits_per_sample = BITS_PER_SAMPLE;

	// Move to start of file
	fr = f_lseek(file, 0);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;

	// Write "RIFF"
	fr = f_write(file, WAV_HEADER_1, 4, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 4) return LOGGER_ERR;

	// Write chunk size (little endian)
	fr = f_write(file, &chunk_size, 4, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 4) return LOGGER_ERR;

	// Write "WAVE" & "FMT "
	fr = f_write(file, WAV_HEADER_2, 8, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 8) return LOGGER_ERR;

	// Write sub 1 chunk size (16 for PCM) & audio format (1 for PCM)
	fr = f_write(file, WAV_HEADER_3, 6, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 6) return LOGGER_ERR;

	// Write number of channels (little endian)
	fr = f_write(file, &num_channels, 2, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 2) return LOGGER_ERR;

	// Write sample rate (little endian)
	fr = f_write(file, &sample_rate, 4, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 4) return LOGGER_ERR;

	// Write byte rate (little endian)
	fr = f_write(file, &byte_rate, 4, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 4) return LOGGER_ERR;

	// Write block align (little endian)
	fr = f_write(file, &block_align, 2, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 2) return LOGGER_ERR;

	// Write bits per sample (little endian)
	fr = f_write(file, &bits_per_sample, 2, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 2) return LOGGER_ERR;

	// Write "data"
	fr = f_write(file, WAV_HEADER_4, 4, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 4) return LOGGER_ERR;

	// Write subchunk 2 size
	fr = f_write(file, &subchunk_2_size, 4, &bw);
	if(fr != FR_OK) return LOGGER_FATFS_ERR;
	if(bw != 4) return LOGGER_ERR;

	return LOGGER_OK;
}

logger_result_t logger_wav_append_nchannels(FIL* file, uint16_t num_channels, uint32_t num_samples, wave_sample_t* samples[])
{
	FRESULT fr;
	UINT bw;

	// Go to the end of the file
	//uint32_t file_size = f_size(file);
	//fr = f_lseek(file, file_size);
	//if(f_tell(file) != file_size) return LOGGER_FATFS_ERR;

//	for(uint32_t samp = 0; samp < num_samples; samp++)
//	{
//		for(uint16_t ch = 0; ch < num_channels; ch++)
//		{
			//fr = f_write(file, samples[ch] + samp, BYTES_PER_SAMPLE, &bw);
			fr = f_write(file, samples[0], num_samples*BYTES_PER_SAMPLE, &bw);
			if(fr != FR_OK) return LOGGER_FATFS_ERR;
			//if(bw != BYTES_PER_SAMPLE) return LOGGER_ERR;
			if(bw != num_samples*BYTES_PER_SAMPLE) return LOGGER_ERR;
//		}
//	}

	return FR_OK;
}
