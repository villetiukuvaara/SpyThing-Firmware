/*
 * gps.c
 *
 *  Created on: Aug 5, 2017
 *      Author: parallels
 */

#include "gps.h"
#include "ublox.h"
#include <string.h>

#define BUFFER_SIZE 512
#define DEFAULT_DATA_SIZE 32
#define TIME_LEFT(timeout, start) (timeout == HAL_MAX_DELAY ? HAL_MAX_DELAY : (timeout - (HAL_GetTick() - start)))
#define NUM_RETRIES 5
#define TX_TIMEOUT 1000
#define RX_TIMEOUT 500
#define ATTEMPT_TX(packet, tries)

uint8_t buffer[BUFFER_SIZE];
uint32_t pos;
minmea_sentence_rmc_t gps_data;

UART_HandleTypeDef* gps_huart;
I2C_HandleTypeDef* gps_hi2c;

//Private functions
void gps_calc_checksum(ubx_packet_t* packet, uint8_t* checksum_a, uint8_t* checksum_b);
gps_status_t gps_ubx_rx(ubx_packet_t* packet, uint32_t timeout);
gps_status_t gps_ubx_tx(ubx_packet_t* packet, uint32_t timeout);
ubx_packet_t gps_ubx_create_packet(uint8_t class, uint8_t id, uint16_t length, uint8_t* data);
gps_status_t gps_ubx_cfg_set(uint8_t id, uint8_t* data, uint16_t length);
gps_status_t gps_ubx_cfg_get(uint8_t id, uint8_t* data, uint16_t length);

gps_status_t gps_initialize(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, uint32_t refresh_period)
{
	gps_huart = huart;
	gps_hi2c = hi2c;
	gps_status_t stat;
	uint16_t length;
	//uint8_t retry;
	//ubx_packet_t p;

	// Hardware reset of the module
	HAL_GPIO_WritePin(GPS_RESET_N_GPIO_Port, GPS_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPS_RESET_N_GPIO_Port, GPS_RESET_N_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	// Disable NMEA output over i2c
	ubx_cfg_inf_data_t inf;
	memset(&inf, 0, sizeof(ubx_cfg_inf_data_t)); // No messages at all
	inf.protocolID = 0x01; // NMEA
	inf.infMsgMask[1].byte = 0b00000111; // Enable the default messages over serial port 1
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_INF, (uint8_t*)&inf, sizeof(ubx_cfg_inf_data_t))) != GPS_OK) return stat;

	// Set up device nav engine settings
	ubx_cfg_nav5_data_t nav5;
	memset(&nav5, 0, sizeof(ubx_cfg_nav5_data_t));
	nav5.mask.b0 = 1; // Apply dynamic model settings
	nav5.dynModel = 0; // 0 = portable, 2 = stationary, 3 = pedestrian, 4 = automotive
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_NAV5, (uint8_t*)&nav5, sizeof(ubx_cfg_nav5_data_t))) != GPS_OK) return stat;

	// Turn off SBAS and GLONLASS
	uint8_t data[128];
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_GNSS, data, 0xFFFF)) != GPS_OK) return stat;
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
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_PM2, (uint8_t*)&pm2, sizeof(ubx_cfg_pm2_data_t))) != GPS_OK) return stat;

	// Enable power save mode
	ubx_cfg_rxm_data_t rxm;
	memset(&rxm, 0, sizeof(ubx_cfg_rxm_data_t));
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_RXM, (uint8_t*)&rxm, sizeof(ubx_cfg_rxm_data_t))) != GPS_OK) return stat;
	rxm.lpMode = 1; // Power save mode
	if((stat = gps_ubx_cfg_set(MSG_ID_CFG_RXM, (uint8_t*)&rxm, sizeof(ubx_cfg_rxm_data_t))) != GPS_OK) return stat;

	memset(&rxm, 0, sizeof(ubx_cfg_rxm_data_t));
	if((stat = gps_ubx_cfg_get(MSG_ID_CFG_RXM, (uint8_t*)&rxm, sizeof(ubx_cfg_rxm_data_t))) != GPS_OK) return stat;

	return GPS_OK;
}

/*
 * Does not guarantee that the GPS has really started back up!
 */
gps_status_t gps_start()
{
	// Send data on UART 1 RX to wake up
	uint8_t data = 0b10101010;
	if(HAL_UART_Transmit(gps_huart, &data, 1, TX_TIMEOUT) != HAL_OK) return GPS_ERR;
	HAL_Delay(1000);

	ubx_cfg_nav5_data_t nav5;
	gps_ubx_cfg_get(MSG_ID_CFG_NAV5, (uint8_t*)&nav5, sizeof(ubx_cfg_nav5_data_t));

	return GPS_OK;
}

gps_status_t gps_stop()
{
	ubx_rxm_pmreq_data_t data;
	memset(&data, 0, sizeof(ubx_rxm_pmreq_data_t));
	data.duration = 0; // Forever
	data.flags.b1 = 1;
	data.flags.b2 = 1;
	data.wakeupSources.b3 = 1; // Wakeup on activity on UART 1 RX

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

gps_status_t gps_solution(gps_sol_t* solution, uint32_t timeout)
{
	uint32_t start = HAL_GetTick();
	ubx_packet_t p = gps_ubx_create_packet(MSG_CLASS_NAV, MSG_ID_NAV_PVT, 0, NULL);
	uint8_t retry = 0;
	gps_status_t stat;

	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TIME_LEFT(start, timeout));
		if(stat == GPS_OK) break;
	}
	if(stat != GPS_OK) return stat;

	p.length = sizeof(gps_sol_t);
	p.data = (uint8_t*)solution;
	stat = gps_ubx_rx(&p, TIME_LEFT(timeout, start));

	return stat;
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
	ubx_packet_t rx;
	uint32_t start = HAL_GetTick();
	HAL_StatusTypeDef stat;

	if(packet->length > 0 && packet->data == NULL) return GPS_ERR;

	packet->sync_char_1 = UBX_SYNC_BYTE_1;
	packet->sync_char_2 = UBX_SYNC_BYTE_2;
	gps_calc_checksum(packet, &(packet->checksum_a), &(packet->checksum_b));
	//packet->checksum_a = 9;

	// Write sync chars, class, id, and length
	stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &(packet->sync_char_1), 6, TIME_LEFT(timeout, start));
	if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	if(stat != HAL_OK) return GPS_ERR;

	// Write data payload
	if(packet->length > 0)
	{
		stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, packet->data, packet->length, TIME_LEFT(timeout, start));
		if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
		if(stat != HAL_OK) return GPS_ERR;
	}

	// Write checksum
	stat = HAL_I2C_Master_Transmit(gps_hi2c, GPS_I2C_ADDRESS_SHIFT, &(packet->checksum_a), 2, TIME_LEFT(timeout, start));
	if(stat == HAL_TIMEOUT) return GPS_TIMEOUT;
	if(stat != HAL_OK) return GPS_ERR;

	return GPS_OK;
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

gps_status_t gps_ubx_cfg_get(uint8_t id, uint8_t* data, uint16_t length)
{
	ubx_packet_t p = gps_ubx_create_packet(MSG_CLASS_CFG, id, 0, NULL);
	ubx_packet_t p2 = gps_ubx_create_packet(MSG_CLASS_CFG, id, length, data);
	gps_status_t stat = GPS_ERR;
	uint8_t retry = 0;

	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TX_TIMEOUT);
		if(stat != GPS_OK) continue;
		if(gps_ubx_rx(&p2, RX_TIMEOUT) == GPS_OK) break;
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

void gps_i2c_rxcplt_callback()
{

}

void gps_i2c_txcplt_callback()
{

}

void gps_i2c_error_callback()
{

}

void gps_i2c_abort_callback()
{

}

void gps_timepulse_callback()
{

}

void gps_extint_callback()
{

}
