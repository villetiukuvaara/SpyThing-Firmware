/*
 * gps.h
 *
 *  Created on: Aug 5, 2017
 *      Author: parallels
 */

#ifndef GPS_H_
#define GPS_H_

#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "minmea.h"

#define FAKE_FIX

#ifndef GPS_RESET_N_GPIO_Port
#define GPS_RESET_N_GPIO_Port GPIOB
#endif

#ifndef GPS_RESET_N_Pin
#define GPS_RESET_N_Pin GPIO_PIN_5
#endif

#ifndef GPS_TIMEPULSE_Pin
#define GPS_TIMEPULSE_Pin GPIO_PIN_15
#endif

#ifndef GPS_TIMEPULSE_GPIO_Port
#define GPS_TIMEPULSE_GPIO_Port GPIOB
#endif

#ifndef GPS_EXTINT_Pin
#define GPS_EXTINT_GPIO_Pin GPIO_PIN_2
#endif

#ifndef GPS_EXTINT_GPIO_Port
#define GPS_EXTINT_GPIO_Port GPIOB
#endif

typedef struct minmea_sentence_rmc minmea_sentence_rmc_t;

typedef enum
{
	GPS_OK = 0,
	GPS_ERR = 1,
	GPS_TIMEOUT = 2
} gps_status_t;

typedef union
{
	struct
	{
		volatile unsigned char b0:1;
		volatile unsigned char b1:1;
		volatile unsigned char b2:1;
		volatile unsigned char b3:1;
		volatile unsigned char b4:1;
		volatile unsigned char b5:1;
		volatile unsigned char b6:1;
		volatile unsigned char b7:1;
	};
	uint8_t byte;
} bitfield8_t;

typedef union
{
	struct
	{
		volatile unsigned char b0:1;
		volatile unsigned char b1:1;
		volatile unsigned char b2:1;
		volatile unsigned char b3:1;
		volatile unsigned char b4:1;
		volatile unsigned char b5:1;
		volatile unsigned char b6:1;
		volatile unsigned char b7:1;
		volatile unsigned char b8:1;
		volatile unsigned char b9:1;
		volatile unsigned char b10:1;
		volatile unsigned char b11:1;
		volatile unsigned char b12:1;
		volatile unsigned char b13:1;
		volatile unsigned char b14:1;
		volatile unsigned char b15:1;
	};
	uint16_t bytes;
} bitfield16_t;

typedef struct
{
	uint32_t iTOW;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	bitfield8_t valid;
	uint32_t tAcc;
	int32_t nano;
	uint8_t fixType;
	bitfield8_t flags;
	bitfield8_t flags2;
	uint8_t numSV;
	int32_t lon;
	int32_t lat;
	int32_t height;
	int32_t hMSL;
	uint32_t hAcc;
	uint32_t vAcc;
	int32_t velN;
	int32_t velE;
	int32_t velD;
	int32_t gSpeed;
	int32_t headMot;
	uint32_t sAcc;
	uint32_t headAcc;
	uint16_t pDOP;
	uint8_t reserved1[6];
	int32_t headVeh;
	int16_t magDec;
	uint16_t magAcc;
} gps_sol_t;

gps_status_t gps_initialize(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, uint32_t refresh_period);
gps_status_t gps_solution(gps_sol_t* solution, uint32_t timeout);
gps_status_t gps_start();
gps_status_t gps_stop();
void gps_i2c_rxcplt_callback();
void gps_i2c_txcplt_callback();
void gps_i2c_error_callback();
void gps_i2c_abort_callback();
void gps_timepulse_callback();
void gps_extint_callback();

#endif /* GPS_H_ */
