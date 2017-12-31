/*
 * gps.c
 *
 *  Created on: Aug 5, 2017
 *      Author: parallels
 */

#include "gps.h"
#include "ublox.h"
#include <string.h>
#include <stdlib.h>

#define BUFFER_SIZE 512
#define DEFAULT_DATA_SIZE 32
#define TIME_LEFT(timeout, start) (timeout == HAL_MAX_DELAY ? HAL_MAX_DELAY : (timeout - (HAL_GetTick() - start)))
#define NUM_RETRIES 5
#define TX_TIMEOUT 1000
#define RX_TIMEOUT 2000
#define ATTEMPT_TX(packet, tries)
#define RTC_UPDATE (uint32_t)0xFFFFFFFF
#define RTC_SYNCH_TIME 360000 // Update every hour

enum
{
	GPS_I2C_STOP,
	GPS_I2C_WAIT,
	GPS_I2C_SYNC1,
	GPS_I2C_SYNC2,
	GPS_I2C_HEADER,
	GPS_I2C_DATA
} gps_i2c_state;

volatile bool running = false;

UART_HandleTypeDef* gps_huart;
I2C_HandleTypeDef* gps_hi2c;
RTC_HandleTypeDef* gps_hrtc;
uint8_t gps_buffer[128];
ubx_packet_t rx_packet;
gps_sol_t last_solution;
RTC_TimeTypeDef gps_solution_timestamp;
RTC_DateTypeDef gps_solution_datestamp;
gps_solution_status_t gps_solution_status;
uint32_t rtc_timestamp = RTC_UPDATE;

//Private functions
void gps_calc_checksum(ubx_packet_t* packet, uint8_t* checksum_a, uint8_t* checksum_b);
gps_status_t gps_ubx_rx(ubx_packet_t* packet, uint32_t timeout);
gps_status_t gps_ubx_tx(ubx_packet_t* packet, uint32_t timeout);
ubx_packet_t gps_ubx_create_packet(uint8_t class, uint8_t id, uint16_t length, uint8_t* data);
gps_status_t gps_ubx_cfg_set(uint8_t id, uint8_t* data, uint16_t length);
gps_status_t gps_ubx_cfg_get(uint8_t id, uint8_t* data, uint16_t poll_length, uint16_t data_length);
gps_status_t gps_ubx_poll(uint8_t class, uint8_t id, uint8_t* data, uint16_t poll_length, uint16_t data_length);
bool gps_tx_ready(void);


gps_status_t gps_initialize(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, RTC_HandleTypeDef* hrtc)
{
	gps_huart = huart;
	gps_hi2c = hi2c;
	gps_hrtc = hrtc;
	gps_status_t stat;
	uint16_t length;
	//uint8_t retry;
	//ubx_packet_t p;
	uint8_t data[128];

	// Disable the IRQ for TX ready indicatino
	HAL_NVIC_DisableIRQ(GPS_PIO6_EXTI_IRQn);

	//Disable I2C interrupts
	HAL_NVIC_DisableIRQ(GPS_I2C_EV_IRQn);
	HAL_NVIC_DisableIRQ(GPS_I2C_ER_IRQn);

	// Hardware reset of the module
	HAL_GPIO_WritePin(GPS_RESET_N_GPIO_Port, GPS_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPS_RESET_N_GPIO_Port, GPS_RESET_N_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	// Clear and load config
	ubx_cfg_cfg_data_t cfg_clear;
	memset(&cfg_clear, 0, sizeof(ubx_cfg_cfg_data_t));
	cfg_clear.clearMask.bytes = 0b1111100011111;
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_CFG, (uint8_t*)&cfg_clear, sizeof(ubx_cfg_cfg_data_t))) != GPS_OK) return stat;
	cfg_clear.clearMask.bytes = 0;
	cfg_clear.loadMask.bytes = 0b1111100011111;
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_CFG, (uint8_t*)&cfg_clear, sizeof(ubx_cfg_cfg_data_t))) != GPS_OK) return stat;

	ubx_mon_hw_data_t hw;
	if((stat = gps_ubx_poll(MSG_CLASS_MON, MSG_ID_MON_HW, (uint8_t*)&hw, 0, sizeof(ubx_mon_hw_data_t))) != GPS_OK) return stat;

	// Disable NMEA output over i2c
	/*ubx_cfg_inf_data_t inf;
	memset(&inf, 0, sizeof(ubx_cfg_inf_data_t)); // No messages at all
	inf.protocolID = 0x01; // NMEA
	inf.infMsgMask[1].byte = 0b00000111; // Enable the default messages over serial port 1
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_INF, (uint8_t*)&inf, sizeof(ubx_cfg_inf_data_t))) != GPS_OK) return stat;*/


	//Disable UART
	ubx_cfg_prt_uart_data_t prt_uart;
	uint8_t* puart = (uint8_t*)&prt_uart;
	*puart = 1;
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PRT, puart, 1, sizeof(ubx_cfg_prt_uart_data_t))) != GPS_OK) return stat;
	//prt_uart.mode.b0 = 0; // Disable
	prt_uart.outProtoMask.bytes &= ~(uint16_t)0b100111;
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_PRT, puart, sizeof(ubx_cfg_prt_uart_data_t))) != GPS_OK) return stat;
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PRT, puart, 1, sizeof(ubx_cfg_prt_uart_data_t))) != GPS_OK) return stat;

	ubx_cfg_prt_ddc_data_t prt_ddc;
	uint8_t* pddc = (uint8_t*)&prt_ddc;
	*pddc = 0;
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PRT, pddc, 1, sizeof(ubx_cfg_prt_ddc_data_t))) != GPS_OK) return stat;
	prt_ddc.flags.b1 = 0; // No extended timeout
	prt_ddc.inProtoMask.bytes = 1; // Only UBX in
	prt_ddc.outProtoMask.bytes = 1; // Only UBX out
	prt_ddc.txReady.bytes = 0;
	prt_ddc.txReady.bytes |= 1<<7; // Threshold of 8 bytes must be ready
	prt_ddc.txReady.bytes |= 6<<2; // Use pin 6 for TX ready indication
	prt_ddc.txReady.b0 = 1; // Enable TX ready GPIO output
	prt_ddc.txReady.b1 = 0; // Active high
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_PRT, (uint8_t*)&prt_ddc, sizeof(ubx_cfg_prt_ddc_data_t))) != GPS_OK) return stat;

	// Check that DDC port settings got applied
	memset(&prt_ddc, 0, sizeof(ubx_cfg_prt_ddc_data_t));
	HAL_Delay(100);
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PRT, (uint8_t*)&prt_ddc, 1, sizeof(ubx_cfg_prt_ddc_data_t))) != GPS_OK) return stat;
	if(prt_ddc.inProtoMask.b0 != 1 || prt_ddc.outProtoMask.b0 != 1 || prt_ddc.txReady.b0 != 1) return GPS_ERR;

	// Set UBX NAV message rates on I2C
	ubx_cfg_msg_data_t msg;
	msg.rate = 1;
	msg.msgClass = MSG_CLASS_NAV;
	msg.msgID = MSG_ID_NAV_PVT;
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_MSG, (uint8_t*)&msg, sizeof(ubx_cfg_msg_data_t))) != GPS_OK) return stat;

	// Set up device nav engine settings
	ubx_cfg_nav5_data_t nav5;
	memset(&nav5, 0, sizeof(ubx_cfg_nav5_data_t));
	nav5.mask.b0 = 1; // Apply dynamic model settings
	nav5.dynModel = GPS_CFG_DYNMODEL; // 0 = portable, 2 = stationary, 3 = pedestrian, 4 = automotive
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_NAV5, (uint8_t*)&nav5, sizeof(ubx_cfg_nav5_data_t))) != GPS_OK) return stat;

	// Turn off SBAS and GLONLASS
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_GNSS, data, 0, 0xFFFF)) != GPS_OK) return stat;
	ubx_cfg_gnss_data_t* gnss = (ubx_cfg_gnss_data_t*)data;
	ubx_cfg_gnss_data_block_t* gnss_block = (ubx_cfg_gnss_data_block_t*)(data + sizeof(ubx_cfg_gnss_data_t));
	for(uint8_t i = 0; i < gnss->numConfigBlocks - 1; i++)
	{
		gnss_block++;
		if(gnss_block->gnssId == 1 || gnss_block->gnssId == 6) gnss_block->flags.b0 = 0;
	}
	length = sizeof(ubx_cfg_gnss_data_block_t) + gnss->numConfigBlocks*sizeof(ubx_cfg_gnss_data_block_t);
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_GNSS, data, length)) != GPS_OK) return stat;

	// Configure power mode
	ubx_cfg_pm2_data_t pm2;
	memset(&pm2, 0, sizeof(ubx_cfg_pm2_data_t));
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PM2, (uint8_t*)&pm2, 0, sizeof(ubx_cfg_pm2_data_t))) != GPS_OK) return stat;
	pm2.flags.b4 = 0; // select EXTINT0
	pm2.flags.b5 = 1; // Keep receiver awake as long as extint is high
	pm2.flags.b6 = 0; // Do not force receiver to go to backup if extint is low
	pm2.flags.b7 = 0; // No extint inactivity timeout
	pm2.flags.b11 = 0; // Add extra wake cycles to update RTC
	pm2.flags.b12 = 1; // Add extra wake cycles to update ephemeris
	pm2.flags.b16 = 0; // receiver enters (Inactive) Awaiting Next Search state after failing acquisition
	pm2.flags.bytes &= ~(uint32_t)(3<<17); // Oon/off power save mode (PSMOO)
	pm2.flags.b17 = 1; //
	pm2.updatePeriod = 10000; // Update every 15 seconds
	pm2.searchPeriod = 10000; // Try to search again every 10 seconds if failed aqcuisition
	pm2.gridOffset = 0;
	pm2.onTime = 1; // Don't stay in tracking mode at all
	pm2.minAcqTime = 2;
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_PM2, (uint8_t*)&pm2, sizeof(ubx_cfg_pm2_data_t))) != GPS_OK) return stat;
	memset(&pm2, 0, sizeof(ubx_cfg_pm2_data_t));
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PM2, (uint8_t*)&pm2, 0, sizeof(ubx_cfg_pm2_data_t))) != GPS_OK) return stat;

	// Enable power save mode
	ubx_cfg_rxm_data_t rxm;
	memset(&rxm, 0, sizeof(ubx_cfg_rxm_data_t));
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_RXM, (uint8_t*)&rxm, 0, sizeof(ubx_cfg_rxm_data_t))) != GPS_OK) return stat;
	rxm.lpMode = GPS_CFG_LOW_POWER_MODE; // Power save mode
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_RXM, (uint8_t*)&rxm, sizeof(ubx_cfg_rxm_data_t))) != GPS_OK) return stat;

	// Save config
	ubx_cfg_cfg_data_t save;
	memset(&save, 0, sizeof(ubx_cfg_cfg_data_t));
	save.saveMask.bytes = cfg_clear.loadMask.bytes = 0b1111100011111;
	//cfg.saveMask.b10 = 1; // Antenna config
	//save.clearMask.bytes = 0b1111100011111; // Clear everything before saving
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_CFG, (uint8_t*)&save, sizeof(ubx_cfg_cfg_data_t))) != GPS_OK) return stat;

	// Load saved config
	//save.saveMask.bytes = 0;
	//save.loadMask.bytes = 0b1111100011111;
	//if((stat = gps_ubx_cfg_set(MSG_ID_CFG_CFG, (uint8_t*)&save, sizeof(ubx_cfg_cfg_data_t))) != GPS_OK) return stat;

	gps_solution_status = GPS_SOL_NONE;
	gps_i2c_state = GPS_I2C_STOP;

	// Enable I2C IRQ
	HAL_NVIC_EnableIRQ(GPS_I2C_EV_IRQn);
	HAL_NVIC_EnableIRQ(GPS_I2C_ER_IRQn);

	return GPS_OK;
}

/*
 * Does not guarantee that the GPS has really started back up!
 */
gps_status_t gps_start()
{
	// Send data on UART 1 RX to wake up
	//uint8_t data = 0b10101010;
	//if(HAL_UART_Transmit(gps_huart, &data, 1, TX_TIMEOUT) != HAL_OK) return GPS_ERR;
	//HAL_Delay(1000);
	HAL_GPIO_WritePin(GPS_EXTINT_GPIO_Port, GPS_EXTINT_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPS_EXTINT_GPIO_Port, GPS_EXTINT_Pin, GPIO_PIN_RESET);

	/*while(HAL_GPIO_ReadPin(GPS_PIO6_GPIO_Port, GPS_PIO6_Pin) == GPIO_PIN_SET)
	{
		uint8_t data;
		HAL_I2C_Master_Receive(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &data, 1, RX_TIMEOUT);
	}*/

	if(gps_i2c_state == GPS_I2C_STOP)
		gps_i2c_state = GPS_I2C_WAIT;

	// Enable I2C IRQ
	uint32_t stat2 = HAL_I2C_GetState(gps_hi2c);
	if(stat2 == HAL_I2C_STATE_RESET
			|| stat2 == HAL_I2C_STATE_ERROR
			|| stat2 == HAL_I2C_STATE_TIMEOUT
			|| stat2 == HAL_I2C_STATE_ABORT
			|| stat2 == HAL_I2C_STATE_BUSY)
	{
		if(HAL_I2C_Init(gps_hi2c) != HAL_OK) Error_Handler();
	}

	HAL_NVIC_EnableIRQ(GPS_I2C_EV_IRQn);
	HAL_NVIC_EnableIRQ(GPS_I2C_ER_IRQn);

	// Cause the interrupt to happen if there is already data ready
	if(gps_tx_ready()) HAL_NVIC_SetPendingIRQ(GPS_PIO6_EXTI_IRQn);
	// Enable the IRQ for TX ready indication
	HAL_NVIC_EnableIRQ(GPS_PIO6_EXTI_IRQn);


	return GPS_OK;
}

gps_status_t gps_stop()
{
	gps_i2c_state = GPS_I2C_STOP;

	//Disable I2C interrupts
	HAL_NVIC_DisableIRQ(GPS_I2C_EV_IRQn);
	HAL_NVIC_DisableIRQ(GPS_I2C_ER_IRQn);

	// Disable the IRQ for TX ready indication
	HAL_NVIC_DisableIRQ(GPS_PIO6_EXTI_IRQn);

	ubx_rxm_pmreq_data_t data;
	memset(&data, 0, sizeof(ubx_rxm_pmreq_data_t));
	data.duration = 0; // Forever
	data.flags.b1 = 1;
	data.flags.b2 = 1;
	data.wakeupSources.b5 = 1; // Wakeup on EXTINT0 edge

	ubx_packet_t p = gps_ubx_create_packet(MSG_CLASS_RXM, MSG_ID_RXM_PMREQ, sizeof(ubx_rxm_pmreq_data_t), (uint8_t*)&data);

	gps_status_t stat;
	uint8_t retry = 0;
	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TX_TIMEOUT);
		if(stat == GPS_OK) break;
	}
	if(stat != GPS_OK) return stat;

	return GPS_OK;
}

gps_solution_status_t gps_solution(gps_sol_t* solution)
{
	if(gps_solution_status == GPS_SOL_NONE) return GPS_SOL_NONE;

	memcpy(solution, &last_solution, sizeof(gps_sol_t));

	if(gps_solution_status == GPS_SOL_NEW)
	{
		gps_solution_status = GPS_SOL_OLD;
		return GPS_SOL_NEW;
	}
	else return GPS_SOL_OLD;
}

/*
 * Calculates the checksum for the packet
 */
void gps_calc_checksum(ubx_packet_t* packet, uint8_t* checksum_a, uint8_t* checksum_b)
{
	*checksum_a = 0;
	*checksum_b = 0;
	uint8_t* ptr = &(packet->pkt_class);

	for(uint16_t i = 0; i < 4; i++)
	{
		*checksum_a += *ptr;
		*checksum_b += *checksum_a;
		ptr++;
	}

	ptr = packet->data;
	for(uint16_t i = 0; i < packet->length; i++)
	{
		*checksum_a += *ptr;
		*checksum_b += *checksum_a;
		ptr++;
	}
}

/*
 * Blocking read for UBX packet. Packet must be prepared with data field allocated for correct length of data
 * If length is set to 0xFFFF then the length is determined automatically, but data should still point to a buffer
 * with enough space!
 */
gps_status_t gps_ubx_rx(ubx_packet_t* packet, uint32_t timeout)
{
	ubx_packet_t rx;
	uint32_t start = HAL_GetTick();
	HAL_StatusTypeDef stat;
	bool find_length = false;

	if(packet->length == 0xFFFF) find_length = true;

	// Write 0xFF to indicate reading from data stream register 0xFF
	uint8_t address = 0xFF;
	stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &address, 1, TIME_LEFT(timeout, start));
	if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	if(stat != HAL_OK) return GPS_ERR;

	while(1)
	{
		// Wait for sync chars
		do
		{
			if(timeout != HAL_MAX_DELAY && HAL_GetTick() - start >= timeout) return GPS_TIMEOUT;
			stat = HAL_I2C_Master_Receive(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &rx.sync_char_1, 1, TIME_LEFT(timeout, start));
			if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
			if(stat != HAL_OK) return GPS_ERR;
		} while(rx.sync_char_1 != UBX_SYNC_BYTE_1);

		stat = HAL_I2C_Master_Receive(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &rx.sync_char_2, 1, TIME_LEFT(timeout, start));
		if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
		if(stat != HAL_OK) return GPS_ERR;
		if(rx.sync_char_2 != UBX_SYNC_BYTE_2) continue;

		// Read class, id, and length
		stat = HAL_I2C_Master_Receive(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &rx.pkt_class, 4, TIME_LEFT(timeout, start));
		if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
		if(stat != HAL_OK) return GPS_ERR;

		if(rx.pkt_class != packet->pkt_class || rx.id != packet->id) continue; // Wrong packet. Wait for the next one

		if(find_length) packet->length = rx.length;
		else if(rx.length != packet->length) return GPS_ERR; // Expecting the wrong amount of data

		// Receive data
		if(packet->length > 0)
		{
			if(packet->data == NULL) return GPS_ERR;

			stat = HAL_I2C_Master_Receive(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, packet->data, packet->length, TIME_LEFT(timeout, start));
			if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
			if(stat != HAL_OK) return GPS_ERR;
		}

		// Receive checksum
		stat = HAL_I2C_Master_Receive(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &(packet->checksum_a), 2, TIME_LEFT(timeout, start));
		if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
		if(stat != HAL_OK) return GPS_ERR;

		// Verify checksum
		gps_calc_checksum(packet, &rx.checksum_a, &rx.checksum_b);
		if(packet->checksum_a == rx.checksum_a && packet->checksum_b == rx.checksum_b) break;
		else return GPS_ERR;
	}

	return GPS_OK;
}

/*
 * Blocking write of the usb packet. The checksum gets calculated.
 */
gps_status_t gps_ubx_tx(ubx_packet_t* packet, uint32_t timeout)
{
	//ubx_packet_t rx;
	uint32_t start = HAL_GetTick();
	HAL_StatusTypeDef stat;

	if(packet->length > 0 && packet->data == NULL) return GPS_ERR;

	packet->sync_char_1 = UBX_SYNC_BYTE_1;
	packet->sync_char_2 = UBX_SYNC_BYTE_2;
	gps_calc_checksum(packet, &(packet->checksum_a), &(packet->checksum_b));

	// Create buffer to write it all out at once
	// 8 extra bytes for header (6) and checksum (2)
	uint16_t length = packet->length + 8;
	uint8_t* buffer = malloc(length);
	if(buffer == NULL) return GPS_ERR;

	// Copy sync charc, class, id, and length into buffer
	memcpy(buffer, &(packet->sync_char_1), 6);
	// Copy data
	if(packet->length != 0) memcpy(buffer + 6, packet->data, packet->length);
	// Copy checksum
	memcpy(buffer + 6 + packet->length, &(packet->checksum_a), 2);

	stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, buffer, length, TIME_LEFT(timeout, start));

	free(buffer);

	if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	else if(stat != HAL_OK) return GPS_ERR;
	else return GPS_OK;

	//	// Write sync chars, class, id, and length
	//	stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &(packet->sync_char_1), 6, TIME_LEFT(timeout, start));
	//	if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	//	if(stat != HAL_OK) return GPS_ERR;
	//
	//	// Write data payload
	//	if(packet->length > 0)
	//	{
	//		stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, packet->data, packet->length, TIME_LEFT(timeout, start));
	//		if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	//		if(stat != HAL_OK) return GPS_ERR;
	//	}
	//
	//	// Write checksum
	//	stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &(packet->checksum_a), 2, TIME_LEFT(timeout, start));
	//	if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	//	if(stat != HAL_OK) return GPS_ERR;
	//
	//	return GPS_OK;
}

gps_status_t gps_ubx_cfg_set(uint8_t id, uint8_t* data, uint16_t length)
{
	ubx_packet_t p = gps_ubx_create_packet(MSG_CLASS_CFG, id, length, data);
	gps_status_t stat = GPS_ERR;
	uint8_t retry = 0;

	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TX_TIMEOUT);
		if(stat != GPS_OK) continue;

		// Get an ACK
		ubx_packet_t p2;
		p2.pkt_class = MSG_CLASS_ACK;
		p2.id = MSG_ID_ACK_ACK;
		uint8_t data[2];
		p2.data = data;
		p2.length = 2;
		if((stat = gps_ubx_rx(&p2, RX_TIMEOUT)) == GPS_OK
				&& data[0] == p.pkt_class
				&& data[1] == p.id)
			return GPS_OK;
	}
	return stat;
}

gps_status_t gps_ubx_cfg_get(uint8_t id, uint8_t* data, uint16_t poll_length, uint16_t data_length)
{
	ubx_packet_t p;
	if(poll_length != 0) p = gps_ubx_create_packet(MSG_CLASS_CFG, id, poll_length, data);
	else p = gps_ubx_create_packet(MSG_CLASS_CFG, id, 0, NULL);

	ubx_packet_t p2 = gps_ubx_create_packet(MSG_CLASS_CFG, id, data_length, data);
	gps_status_t stat = GPS_ERR;
	uint8_t retry = 0;

	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TX_TIMEOUT);
		if(stat != GPS_OK) continue;
		if((stat = gps_ubx_rx(&p2, RX_TIMEOUT)) == GPS_OK) return GPS_OK;
	}

	return stat;
}

gps_status_t gps_ubx_poll(uint8_t class, uint8_t id, uint8_t* data, uint16_t poll_length, uint16_t data_length)
{
	ubx_packet_t p;
	if(poll_length != 0) p = gps_ubx_create_packet(class, id, poll_length, data);
	else p = gps_ubx_create_packet(class, id, 0, NULL);

	ubx_packet_t p2 = gps_ubx_create_packet(class, id, data_length, data);
	gps_status_t stat = GPS_ERR;
	uint8_t retry = 0;

	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TX_TIMEOUT);
		if(stat != GPS_OK) continue;
		if((stat = gps_ubx_rx(&p2, RX_TIMEOUT)) == GPS_OK) return GPS_OK;
	}

	return stat;
}

ubx_packet_t gps_ubx_create_packet(uint8_t class, uint8_t id, uint16_t length, uint8_t* data)
{
	// Disable NMEA output over i2c
	ubx_packet_t p;
	p.pkt_class = class;
	p.id = id;
	p.length = length;
	p.data = data;

	return p;
}

bool gps_tx_ready(void)
{
	return HAL_GPIO_ReadPin(GPS_PIO6_GPIO_Port, GPS_PIO6_Pin);
}

void gps_i2c_rxcplt_callback()
{
	HAL_StatusTypeDef stat;

	switch(gps_i2c_state)
	{
	case GPS_I2C_STOP: return;
	case GPS_I2C_WAIT: // This shouldn't happen, but fall through
	case GPS_I2C_SYNC1:
		if(gps_buffer[0] == UBX_SYNC_BYTE_1) gps_i2c_state = GPS_I2C_SYNC2;
		else if(gps_buffer[0] != 0xFF)
		{
			gps_i2c_state = GPS_I2C_SYNC1;
		}
		HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, 1);
		break;

	case GPS_I2C_SYNC2:
		if(gps_buffer[0] == UBX_SYNC_BYTE_2)
		{
			gps_i2c_state = GPS_I2C_HEADER;
			HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, 4);
		}
		else // Didn't get second sync byte
		{
			if(gps_tx_ready())
			{
				gps_i2c_state = GPS_I2C_SYNC1;
				HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, 1);
			}
			else
			{
				gps_i2c_state = GPS_I2C_WAIT;
			}
		}
		break;

	case GPS_I2C_HEADER:
		gps_i2c_state = GPS_I2C_DATA;
		rx_packet.pkt_class = gps_buffer[0];
		rx_packet.id = gps_buffer[1];
		rx_packet.length = *((uint16_t*)(gps_buffer + 2));
		rx_packet.data = gps_buffer;
		// 2 extra bytes to recieve checksum bytes
		HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, rx_packet.length + 2);
		break;
	case GPS_I2C_DATA:
		rx_packet.checksum_a = gps_buffer[rx_packet.length];
		rx_packet.checksum_b = gps_buffer[rx_packet.length + 1];
		uint8_t ck_a, ck_b;
		gps_calc_checksum(&rx_packet, &ck_a, &ck_b);
		if(rx_packet.checksum_a == ck_a && rx_packet.checksum_b == ck_b)
		{
			if(rx_packet.pkt_class == MSG_CLASS_NAV && rx_packet.id == MSG_ID_NAV_PVT)
			{
				gps_sol_t* sol = (gps_sol_t*)gps_buffer;
				if(sol->valid.b0 == 1 && sol->valid.b1 == 1) // Valid time
				{
					RTC_TimeTypeDef time;
					RTC_DateTypeDef date;

					time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					time.Hours = sol->hour;
					time.Minutes = sol->min;
					time.Seconds = sol->sec < 60 ? sol->sec : 60; // Can be longer than 60
					time.TimeFormat = RTC_HOURFORMAT_24;

					date.Year = (uint8_t)(sol->year - 2000);
					date.Month = sol->month;
					date.Date = sol->day;

					if(rtc_timestamp == RTC_UPDATE || HAL_GetTick() > rtc_timestamp + RTC_SYNCH_TIME)
					{
						stat = HAL_RTC_SetTime(gps_hrtc, &time, RTC_FORMAT_BIN);
						if(stat != HAL_OK) Error_Handler();

						stat = HAL_RTC_SetDate(gps_hrtc, &date, RTC_FORMAT_BIN);
						if(stat != HAL_OK) Error_Handler();

						rtc_timestamp = HAL_GetTick();
					}

					if(sol->fixType != 0) // Location fix
					{
						gps_solution_timestamp = time;
						gps_solution_datestamp = date;
						memcpy(&last_solution, gps_buffer, sizeof(gps_sol_t));

						printf("(%li,%li) @ %02u:%02u\n", last_solution.lat, last_solution.lon, last_solution.hour, last_solution.min);
						gps_solution_status = GPS_SOL_NEW;
					}
				}
			}
		}

		// Look for next packet if data still ready
		if(gps_tx_ready())
		{
			gps_i2c_state = GPS_I2C_SYNC1;
			HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, 1);
		}
		else gps_i2c_state = GPS_I2C_WAIT;
		break;
	}
}

void gps_i2c_txcplt_callback()
{

}

void gps_i2c_error_callback()
{
	uint32_t err = HAL_I2C_GetError(gps_hi2c);
	if(err == HAL_I2C_ERROR_OVR)
	{
		printf("error\n");
	}
	//else Error_Handler();
	//	// Reinitialize I2C
	//	HAL_I2C_Init(gps_hi2c);
	//
	//	if(gps_i2c_state != GPS_I2C_STOP)
	//	{
	//		if(gps_tx_ready())
	//		{
	//			gps_i2c_state = GPS_I2C_SYNC1;
	//			HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, 1);
	//		}
	//		else gps_i2c_state = GPS_I2C_WAIT;
	//	}
}

void gps_i2c_abort_callback()
{
	Error_Handler();
}

void gps_timepulse_callback()
{

}

void gps_dio6_callback()
{
	switch(gps_i2c_state)
	{
	case GPS_I2C_WAIT:
		//if(HAL_I2C_GetState())
		//	HAL_I2C_Init();
		gps_i2c_state = GPS_I2C_SYNC1;
		HAL_I2C_Master_Receive_IT(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, gps_buffer, 1);
		break;
	default:
		break;
	}
}
