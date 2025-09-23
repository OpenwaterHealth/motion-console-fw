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
#include "led_driver.h"

#include <string.h>

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;

SemaphoreHandle_t uartTxSemaphore;
SemaphoreHandle_t xRxSemaphore;
TaskHandle_t commsTaskHandle;

extern FAN_Driver fan;
extern uint8_t FIRMWARE_VERSION_DATA[3];
extern bool _enter_dfu;

static uint8_t last_fan_speed = 0;
static uint32_t id_words[3] = {0};
static uint8_t i2c_list[10] = {0};
static uint8_t i2c_data[0xff] = {0};
static uint32_t last_fsync_count = 0;
static uint32_t last_lsync_count = 0;

static char retTriggerJson[0xFF];

static _Bool process_controller_command(UartPacket *uartResp, UartPacket *cmd);

const osThreadAttr_t comm_rec_task_attribs = {
  .name = "comRecTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

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
		// printf("Sending Response\r\n");
		// printf("send data\r\n");
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
		// HAL_UART_Transmit_DMA(&huart1, txBuffer, bufferIndex);
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

		// printf("data received\r\n");
		int bufferIndex = 0;

		if(rxBuffer[bufferIndex++] != OW_START_BYTE) {
			// Send NACK doesn't have the correct start byte
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
			// Send NACK response due to no end byte
			// data can exceed buffersize but every buffer must have a start and end packet
			// command that will send more data than one buffer will follow with data packets to complete the request
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
			// Send NACK response due to bad CRC
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
		// printUartPacket(&cmd);
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


static _Bool process_controller_command(UartPacket *uartResp, UartPacket *cmd)
{
	_Bool ret = true;
	int iRet = 0;
	uartResp->command = cmd->command;
	switch (cmd->command)
	{
		case OW_CTRL_I2C_SCAN:
			//printf("I2C Scan\r\n");
			uartResp->command = OW_CTRL_I2C_SCAN;
			if(cmd->data_len != 2){
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}else{
				//printf("I2C Scan MUX: 0x%02X CH: 0x%02X \r\n", cmd->data[0], cmd->data[1]);
				memset(i2c_list, 0, 10);
				iRet = TCA9548A_scan_channel(cmd->data[0], cmd->data[1], i2c_list, 10, false);
				if(iRet < 0){
					// error
					uartResp->packet_type = OW_ERROR;
					uartResp->data_len = 0;
					uartResp->data = NULL;
				} else {
					uartResp->data_len = (uint16_t)iRet;
					uartResp->data = i2c_list;
				}
			}
			break;
		case OW_CTRL_SET_IND:
			//printf("Console SET Indicator\r\n");
			uartResp->command = OW_CTRL_SET_IND;
			if(uartResp->reserved > 3){
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}
			else
			{
				LED_RGB_SET(cmd->reserved);
			}
			break;
		case OW_CTRL_GET_IND:
			//printf("Console GET Indicator\r\n");
			uartResp->command = OW_CTRL_GET_IND;
			uartResp->reserved = LED_RGB_GET();
			break;
		case OW_CTRL_SET_FAN:
			//printf("Console Set Fan ADDR: 0x%02X SPEED: 0x%02X\r\n", cmd->addr, cmd->data[0]);
			uartResp->command = OW_CTRL_SET_FAN;
			if(cmd->addr > 1 || cmd->data_len != 1){
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}else{
				printf("Set fan to: %d\r\n", cmd->data[0]);
				FAN_SetManualPWM(&fan, cmd->data[0]);
			}
			break;
		case OW_CTRL_GET_FAN:
			//printf("Console Get Fan ADDR: 0x%02X\r\n", cmd->addr);
			uartResp->command = OW_CTRL_GET_FAN;
			if(cmd->addr > 1){
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}else{
				last_fan_speed = FAN_GetPWMDuty(&fan);
				uartResp->data_len = 1;
				uartResp->data = &last_fan_speed;
			}
			break;
		case OW_CTRL_I2C_RD:
			uartResp->command = OW_CTRL_I2C_RD;
			if(cmd->data_len != 5) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			} else {
				uint8_t mux_index = cmd->data[0];
				uint8_t channel = cmd->data[1];
				uint8_t i2c_addr = cmd->data[2];
				uint8_t reg_addr = cmd->data[3];
				uint8_t data_len = cmd->data[4];
				memset(i2c_data, 0, 0xff); // clear buffer
				// printf("I2C Read MUX: 0x%02X CHANNEL: 0x%02X I2C ADDR: 0x%02X REGISTER: 0x%02X DATA_LEN: 0x%02X\r\n", mux_index, channel, i2c_addr, reg_addr, data_len);
				int8_t ret = TCA9548A_Read_Data(mux_index, channel, i2c_addr, reg_addr, data_len, i2c_data);
				if (ret!= TCA9548A_OK) {
					printf("error selecting channel\r\n");
					uartResp->packet_type = OW_ERROR;
					uartResp->data_len = 0;
					uartResp->data = NULL;
				} else {
					uartResp->data_len = data_len;
					uartResp->data = i2c_data;
				}
			}
			break;
		case OW_CTRL_I2C_WR:
			uartResp->command = OW_CTRL_I2C_WR;
			if(cmd->data_len < 5) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			} else {
				uint8_t mux_index = cmd->data[0];
				uint8_t channel = cmd->data[1];
				uint8_t i2c_addr = cmd->data[2];
				uint8_t reg_addr = cmd->data[3];
				uint8_t data_len = cmd->data[4];
				uint8_t *pData = &cmd->data[5];
				// printf("I2C Write MUX: 0x%02X CHANNEL: 0x%02X I2C ADDR: 0x%02X REGISTER: 0x%02X DATA_LEN: 0x%02X\r\n",mux_index, channel, i2c_addr, reg_addr, data_len);
				// printf("Data to Write\r\n");
				// printBuffer(pData, data_len);
				int8_t ret = TCA9548A_Write_Data(mux_index, channel, i2c_addr, reg_addr, data_len, pData);
				if (ret!= TCA9548A_OK) {
					printf("error selecting channel\r\n");
					uartResp->packet_type = OW_ERROR;
					uartResp->data_len = 0;
					uartResp->data = NULL;
				}
			}
			break;
		case OW_CTRL_SET_TRIG:
			uartResp->command = OW_CTRL_SET_TRIG;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;

			if(Trigger_SetConfigFromJSON((char *)cmd->data, cmd->data_len) != HAL_OK)
			{
				uartResp->packet_type = OW_ERROR;
			}else{
				// refresh state
				memset(retTriggerJson, 0, sizeof(retTriggerJson));
				if(Trigger_GetConfigToJSON(retTriggerJson, 0xFF) != HAL_OK)
				{
					uartResp->packet_type = OW_ERROR;
				}else{
					uartResp->data_len = strlen(retTriggerJson);
					uartResp->data = (uint8_t *)retTriggerJson;
				}
			}

			break;

		case OW_CTRL_GET_TRIG:
			uartResp->command = OW_CTRL_GET_TRIG;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;
			memset(retTriggerJson, 0, sizeof(retTriggerJson));
			if(Trigger_GetConfigToJSON(retTriggerJson, 0xFF) != HAL_OK)
			{
				uartResp->packet_type = OW_ERROR;
			}else{
				uartResp->data_len = strlen(retTriggerJson);
				uartResp->data = (uint8_t *)retTriggerJson;
			}
			break;
		case OW_CTRL_START_TRIG:
			uartResp->command = OW_CTRL_START_TRIG;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;

			LED_RGB_SET(1); // Blue
			Trigger_Start();
			break;
		case OW_CTRL_STOP_TRIG:
			uartResp->command = OW_CTRL_STOP_TRIG;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;

			LED_RGB_SET(2); // Green
			Trigger_Stop();
			break;
		case OW_CTRL_GET_FSYNC:
			uartResp->command = OW_CTRL_GET_FSYNC;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 4;
			last_fsync_count = get_fsync_pulse_count();
			uartResp->data = (uint8_t *)&last_fsync_count;
			break;
		case OW_CTRL_GET_LSYNC:
			uartResp->command = OW_CTRL_GET_LSYNC;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 4;
			last_lsync_count = get_lsync_pulse_count();
			uartResp->data = (uint8_t *)&last_lsync_count;
			break;
		case OW_CTRL_TEC_STATUS:
			uartResp->command = OW_CTRL_TEC_STATUS;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 1;
			*uartResp->data = (uint8_t) is_tec_enabled();
			break;
		default:
			uartResp->data_len = 0;
			uartResp->packet_type = OW_UNKNOWN;
			break;
	}

	return ret;
}

_Bool process_if_command(UartPacket *uartResp, UartPacket *cmd)
{
	uartResp->id = cmd->id;
	uartResp->packet_type = OW_RESP;
	uartResp->addr = 0;
	uartResp->reserved = 0;
	uartResp->data_len = 0;
	uartResp->data = 0;
	switch (cmd->packet_type)
	{
	case OW_CMD:
		uartResp->command = cmd->command;
		switch(cmd->command)
		{
		case OW_CMD_PING:
			//printf("ping response\r\n");
			break;
		case OW_CMD_NOP:
			//printf("NOP response\r\n");
			break;
		case OW_CMD_VERSION:
			//printf("Version response\r\n");
			uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
			uartResp->data = FIRMWARE_VERSION_DATA;
			break;
		case OW_CMD_ECHO:
			// exact copy
			//printf("Echo response\r\n");
			uartResp->data_len = cmd->data_len;
			uartResp->data = cmd->data;
			break;
		case OW_CMD_HWID:
			//printf("HWID response\r\n");
			id_words[0] = HAL_GetUIDw0();
			id_words[1] = HAL_GetUIDw1();
			id_words[2] = HAL_GetUIDw2();
			uartResp->data_len = 16;
			uartResp->data = (uint8_t *)&id_words;
			break;
		case OW_CMD_TOGGLE_LED:
			//printf("Toggle LED\r\n");
			HAL_GPIO_TogglePin(LED_ON_GPIO_Port, LED_ON_Pin);
			break;
		case OW_CMD_RESET:
			//printf("Soft Reset\r\n");
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;

			__HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim15, 0);
			if(HAL_TIM_Base_Start_IT(&htim15) != HAL_OK){
				uartResp->packet_type = OW_ERROR;
			}
			break;

		case OW_CMD_DFU:
			printf("Enter DFU\r\n");
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;

			_enter_dfu = true;

			__HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim15, 0);
			if(HAL_TIM_Base_Start_IT(&htim15) != HAL_OK){
				uartResp->packet_type = OW_ERROR;
			}
			break;
		default:
			break;
		}
		break;
	case OW_CONTROLLER:
		return process_controller_command(uartResp, cmd);
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd->tag;
		break;
	}

	return true;
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        // Handle errors here. Maybe reset DMA reception, etc.
    }
}
