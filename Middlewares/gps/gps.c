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

uint8_t buffer[BUFFER_SIZE];
uint32_t pos;
minmea_sentence_rmc_t gps_data;

UART_HandleTypeDef* gps_huart;
I2C_HandleTypeDef* gps_hi2c;

//Private functions
void gps_calc_checksum(ubx_packet_t* packet, uint8_t* checksum_a, uint8_t* checksum_b);
gps_status_t gps_ubx_rx(ubx_packet_t* packet, uint32_t timeout);
gps_status_t gps_ubx_tx(ubx_packet_t* packet, uint32_t timeout);
bool gps_ubx_rx_ack(uint32_t timeout);
ubx_packet_t gps_ubx_create_packet(uint8_t class, uint8_t id, uint16_t length, uint8_t* data);

gps_status_t gps_initialize(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, uint32_t refresh_period)
{
	gps_huart = huart;
	gps_hi2c = hi2c;
	gps_status_t stat;
	uint8_t retry;
	ubx_packet_t p;

	// Hardware reset of the module
	HAL_GPIO_WritePin(GPS_RESET_N_GPIO_Port, GPS_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPS_RESET_N_GPIO_Port, GPS_RESET_N_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	// Check version
	uint8_t cnt = 0;
	bool stop = false;

	// Disable NMEA output over i2c
	p.pkt_class = MSG_CLASS_CFG;
	p.id = MSG_ID_CFG_INF;
	ubx_cfg_inf_data_t inf;
	inf.protocolID = 0x01; // NMEA
	//inf.infMsgMask[0].byte = 0; // No messages over I2C
	memset(inf.infMsgMask, 0, 6*sizeof(bitfield8_t));
	inf.infMsgMask[1].byte = 0b00000111; // Enable the default messages over serial port 1
	p.length = sizeof(ubx_cfg_inf_data_t);
	p.data = (uint8_t*)(&inf);
	retry = 0;
	while(retry++ < NUM_RETRIES)
	{
		stat = gps_ubx_tx(&p, TX_TIMEOUT);
		if(stat != GPS_OK) continue;
		if(gps_ubx_rx_ack(RX_TIMEOUT))
		{
			stat = GPS_OK;
			break;
		}
	}
	if(stat != GPS_OK) return stat;

	return GPS_OK;
}

gps_status_t gps_start()
{
	ubx_packet_t p;
	p.pkt_class = MSG_CLASS_CFG;
	p.id = MSG_ID_CFG_RST;

	ubx_cfg_rst_data_t data;
	p.data = (uint8_t*)(&data);
	p.length = sizeof(ubx_cfg_rst_data_t);
	data.resetMode = 0x09; // Controlled GNSS stop
	gps_status_t stat = gps_ubx_tx(&p, 1000);
	return stat;
}

gps_status_t gps_stop()
{
	ubx_packet_t p;
	p.pkt_class = MSG_CLASS_CFG;
	p.id = MSG_ID_CFG_RST;

	ubx_cfg_rst_data_t data;
	p.data = (uint8_t*)(&data);
	p.length = sizeof(ubx_cfg_rst_data_t);
	data.resetMode = 0x08; // Controlled GNSS stop
	gps_status_t stat = gps_ubx_tx(&p, 1000);
	return stat;
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
 */
gps_status_t gps_ubx_rx(ubx_packet_t* packet, uint32_t timeout)
{
	ubx_packet_t rx;
	uint32_t start = HAL_GetTick();
	HAL_StatusTypeDef stat;

	if(packet->length > 0 && packet->data == NULL) return GPS_ERR;

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

		if(rx.length != packet->length) return GPS_ERR; // Expecting the wrong amount of data

		// Receive data
		if(packet->length > 0)
		{
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

/*
 * Returns an ack
 */
bool gps_ubx_rx_ack(uint32_t timeout)
{
	ubx_packet_t p;
	p.pkt_class = MSG_CLASS_ACK;
	p.id = MSG_ID_ACK_ACK;
	p.length = 0;
	if(gps_ubx_rx(&p, timeout) == GPS_OK) return true;
	else return false;
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
