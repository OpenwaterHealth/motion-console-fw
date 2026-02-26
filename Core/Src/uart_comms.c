/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include "usbd_cdc_if.h"
#include "tca9548a.h"
#include "trigger.h"
#include "fan_driver.h"
#include "ads7828.h"
#include "ad5761r.h"
#include "ads7924.h"
#include "max31875.h"
#include "led_driver.h"
#include "if_commands.h"

#include <string.h>

#define PDU_N 16

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];
extern ADS7924_HandleTypeDef tec_ads;
extern ADS7828_HandleTypeDef adc_mon[2];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
volatile uint8_t tx_busy = 0;

/* consoleTemps is owned by the command-processing module */
extern ConsoleTemperatures consoleTemps;

extern FAN_Driver fan;
extern bool _enter_dfu;

extern ad5761r_dev tec_dac;

// Local helper used for sending
void printUartPacket(const UartPacket* packet) {
	if (!packet) {
		printf("Invalid packet (NULL pointer).\n");
		return;
	}

	printf("UartPacket:\r\n");
	printf("  ID: %u\r\n", packet->id);
	printf("  Packet Type: 0x%02X\r\n", packet->packet_type);
	printf("  Command: 0x%02X\r\n", packet->command);
	printf("  Address: 0x%02X\r\n", packet->addr);
	printf("  Reserved: 0x%02X\r\n", packet->reserved);
	printf("  Data Length: %u\r\n", packet->data_len);

	printf("  Data: ");
	if (packet->data && packet->data_len > 0) {
		for (uint16_t i = 0; i < packet->data_len; ++i) {
			printf("0x%02X ", packet->data[i]);
		}
		printf("\r\n");
	} else {
		printf("No data\r\n");
	}

	printf("  CRC: 0x%04X\r\n\r\n", packet->crc);
}

static void UART_INTERFACE_SendDMA(UartPacket* pResp)
{
	if (!pResp) return;
	// Wait for previous transmission to complete
	while (tx_busy) {
		HAL_Delay(1);
	}
	memset(txBuffer, 0, sizeof(txBuffer));
	int bufferIndex = 0;

	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = pResp->packet_type;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;
	if (pResp->data_len > 0) {
		memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	txBuffer[bufferIndex++] = OW_END_BYTE;

	tx_flag = 0;
	tx_busy = 1;
	CDC_Transmit_FS(txBuffer, bufferIndex);
	// Wait for transmit complete (blocking). In no-OS designs this is typical from main loop
	while (!tx_flag) {
		HAL_Delay(1);
	}
	tx_busy = 0;
}

// Process one received packet if available. Call this periodically from main loop.
void comms_process(void)
{
	if (!rx_flag) return;

	UartPacket cmd;
	UartPacket resp;
	uint16_t calculated_crc;
	int bufferIndex = 0;

	if (rxBuffer[bufferIndex++] != OW_START_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;
	cmd.packet_type = rxBuffer[bufferIndex++];
	cmd.command = rxBuffer[bufferIndex++];
	cmd.addr = rxBuffer[bufferIndex++];
	cmd.reserved = rxBuffer[bufferIndex++];

	// Extract payload length
	cmd.data_len = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;

	// Check if data length is valid
	if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex && rxBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	// Extract data pointer
	cmd.data = &rxBuffer[bufferIndex];
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		bufferIndex = COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
	} else {
		bufferIndex += cmd.data_len; // move pointer to end of data
	}

	// Extract received CRC
	cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;

	// Calculate CRC for received data
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE-3);
	} else {
		calculated_crc = util_crc16(&rxBuffer[1], cmd.data_len + 8);
	}

	// Check CRC
	if (cmd.crc != calculated_crc) {
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_BAD_CRC;
		goto NextDataPacket;
	}

	// Check end byte
	if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.addr = 0;
		resp.reserved = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	process_if_command(&resp, &cmd);

NextDataPacket:
	UART_INTERFACE_SendDMA(&resp);
	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;
	rx_flag = 0;
	// Restart reception
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

void comms_init(void) {
	printf("Initilize comms (no-RTOS)\r\n");

	/* initialize console temps via the command module ownership */
	consoleTemps.f.t1 = 0;
	consoleTemps.f.t2 = 0;
	consoleTemps.f.t3 = 0;

	tx_busy = 0;
	tx_flag = 0;
	rx_flag = 0;

	CDC_FlushRxBuffer_FS();
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

// Callback functions
void comms_handle_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t pos) {

    if (huart->Instance == USART1) {
        // Notify the task
    	rx_flag = 1;
    }
}

void CDC_handle_RxCpltCallback(uint16_t len) {
	rx_flag = 1;
}

void CDC_handle_TxCpltCallback() {
	tx_flag = 1;
}

void comms_handle_TxCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		tx_flag = 1;
	}
}

void comms_handle_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        // Handle errors here. Maybe reset DMA reception, etc.
    }
}
