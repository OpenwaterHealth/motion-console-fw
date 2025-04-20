/*
 * common.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define COMMAND_MAX_SIZE 2048

#include <stdio.h>
#include <stdint.h>

/*
 * UART Communication Protocol Packet Structure:
 *
 * | Start Byte | ID | Packet Type | Command | addr | reserved | Length | Payload | CRC16 | End Byte |
 *
 * Definitions:
 *
 * Start Byte:
 *   - A predefined byte to indicate the beginning of a packet.
 *   - Value: 0xAA (as per USTX_ProtocolTypes)
 *
 * ID:
 *   - ID for transaction used for response or continuation data.
 *
 * Packet Type:
 *   - Indicates the type of the packet being sent or received.
 *   - Possible values:
 *     - OW_ACK: Acknowledgment packet (Value: 0xE0)
 *     - OW_NAK: Negative acknowledgment packet (Value: 0xE1)
 *     - OW_CMD: Command packet (Value: 0xE2)
 *     - OW_RESP: Response packet (Value: 0xE3)
 *     - OW_DATA: Data packet (Value: 0xE4)
 *     - OW_JSON: JSON data packet (Value: 0xE5)
 *     - OW_BAD_CRC: Bad CRC error packet (Value: 0xEE)
 *     - OW_ERROR: General error packet (Value: 0xEF)
 *
 * Command:
 *   - Specifies the command or action to be taken.
 *   - Possible values:
 *     - USTX_NOP: No operation command (Value: 0xB0)
 *     - USTX_PING: Ping command (Value: 0xB1)
 *     - USTX_VERSION: Request for version information (Value: 0xB2)
 *     - USTX_ID: Request for ID information (Value: 0xB3)
 *
 * Length:
 *   - Indicates the length of the payload data.
 *
 * Payload:
 *   - Contains the actual data or information being sent or received.
 *   - Size can vary up to a maximum of COMMAND_MAX_SIZE (2048 bytes).
 *
 * CRC16:
 *   - A 16-bit Cyclic Redundancy Check value for error-checking purposes.
 *   - Helps in detecting errors in the transmitted data.
 *
 * End Byte:
 *   - A predefined byte to indicate the end of a packet.
 *   - Value: 0xDD (as per USTX_ProtocolTypes)
 *
 */



typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} OWProtocolTypes;

typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_I2C_PASSTHRU = 0xE9,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} OWPacketTypes;

typedef enum {
	OW_CTRL_SET_TRIG = 0x13,
	OW_CTRL_GET_TRIG = 0x14,
	OW_CTRL_START_TRIG = 0x15,
	OW_CTRL_STOP_TRIG = 0x16,
} MotionControllerCommands;

typedef enum {
	OW_CODE_SUCCESS = 0x00,
	OW_CODE_IDENT_ERROR = 0xFD,
	OW_CODE_DATA_ERROR = 0xFE,
	OW_CODE_ERROR = 0xFF,
} OWErrorCodes;

typedef enum {
	OW_CMD_PING = 0x00,
	OW_CMD_PONG = 0x01,
	OW_CMD_VERSION = 0x02,
	OW_CMD_ECHO = 0x03,
	OW_CMD_TOGGLE_LED = 0x04,
	OW_CMD_HWID = 0x05,
	OW_CMD_NOP = 0x0E,
	OW_CMD_RESET = 0x0F,
} OWGlobalCommands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint8_t addr;
	uint8_t reserved;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} UartPacket;

_Bool process_if_command(UartPacket *uartResp, UartPacket *cmd);

#endif /* INC_COMMON_H_ */
