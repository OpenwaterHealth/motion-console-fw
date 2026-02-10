/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
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

SemaphoreHandle_t uartTxSemaphore;
SemaphoreHandle_t xRxSemaphore;
TaskHandle_t commsTaskHandle;

/* consoleTemps is owned by the command-processing module */
extern ConsoleTemperatures consoleTemps;

extern FAN_Driver fan;
extern bool _enter_dfu;

extern ad5761r_dev tec_dac;

const osThreadAttr_t comm_rec_task_attribs = {
  .name = "comRecTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

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
	// Wait for semaphore availability before proceeding
	if (xSemaphoreTake(uartTxSemaphore, portMAX_DELAY) == pdTRUE) {
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
		if(pResp->data_len > 0)
		{
			memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
			bufferIndex += pResp->data_len;
		}
		uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
		txBuffer[bufferIndex++] = crc >> 8;
		txBuffer[bufferIndex++] = crc & 0xFF;

		txBuffer[bufferIndex++] = OW_END_BYTE;

		CDC_Transmit_FS(txBuffer, bufferIndex);
		while(!tx_flag);
		xSemaphoreGive(uartTxSemaphore);
	}
}

void comms_receive_task(void *argument) {

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	CDC_FlushRxBuffer_FS();

	UartPacket cmd;
	UartPacket resp;
	uint16_t calculated_crc;
	rx_flag = 0;
	tx_flag = 0;
	while(1) {
		CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		int bufferIndex = 0;

		if(rxBuffer[bufferIndex++] != OW_START_BYTE) {
			resp.id = cmd.id;
			resp.data_len = 0;
			resp.packet_type = OW_NAK;
			goto NextDataPacket;
		}

		cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
		bufferIndex+=2;
		cmd.packet_type = rxBuffer[bufferIndex++];
		cmd.command = rxBuffer[bufferIndex++];
		cmd.addr = rxBuffer[bufferIndex++];
		cmd.reserved = rxBuffer[bufferIndex++];

		// Extract payload length
		cmd.data_len = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
		bufferIndex+=2;

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
		if (cmd.data_len > COMMAND_MAX_SIZE)
		{
			bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
		}else{
			bufferIndex += cmd.data_len; // move pointer to end of data
		}

		// Extract received CRC
		cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
		bufferIndex+=2;

		// Calculate CRC for received data

		if (cmd.data_len > COMMAND_MAX_SIZE)
		{
			calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE-3);
		}
		else
		{
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
		ptrReceive=0;
		rx_flag = 0;
	}

}

// This is the FreeRTOS task
void comms_init() {
	printf("Initilize comms task\r\n");

	/* initialize console temps via the command module ownership */
	consoleTemps.f.t1 = 0;
	consoleTemps.f.t2 = 0;
	consoleTemps.f.t3 = 0;

	uartTxSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(uartTxSemaphore); // Initially available for transmission
	xRxSemaphore = xSemaphoreCreateBinary();
	if (xRxSemaphore == NULL) {
		// Handle semaphore creation failure
		printf("failed to create xRxSemaphore\r\n");
	}

	commsTaskHandle = osThreadNew(comms_receive_task, NULL, &comm_rec_task_attribs);
	if (commsTaskHandle == NULL) {
		printf("Failed to create comms Task\r\n");
	}
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
	// printf("CDC_handle_RxCpltCallback enter\r\n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(commsTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	// printf("CDC_handle_RxCpltCallback exit\r\n");
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
