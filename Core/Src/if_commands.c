/*
 * if_commands.c
 *
 *  Created as refactor target for command processing from uart_comms.c
 */

#include "main.h"
#include "common.h"
#include "uart_comms.h"
#include "utils.h"
#include "tca9548a.h"
#include "trigger.h"
#include "fan_driver.h"
#include "ads7828.h"
#include "ad5761r.h"
#include "ads7924.h"
#include "max31875.h"
#include "led_driver.h"
#include "mcp42u83.h"

#include <string.h>

#define PDU_N 16

ConsoleTemperatures consoleTemps;
static uint8_t board_id;

typedef struct {
    uint16_t raws[PDU_N];
    float    vals[PDU_N];
} PDUFields_t;

typedef union {
    PDUFields_t f;
    uint8_t     bytes[sizeof(PDUFields_t)];
} PDUFrame_t;

static PDUFrame_t pdu_frame;

extern ADS7924_HandleTypeDef tec_ads;
extern ADS7828_HandleTypeDef adc_mon[2];

extern mcp42u83_dev mcp42u83_device;

extern FAN_Driver fan;
extern uint8_t FIRMWARE_VERSION_DATA[3];
extern bool _enter_dfu;

extern ad5761r_dev tec_dac;

static uint8_t last_fan_speed = 0;
static uint32_t id_words[3] = {0};
static uint8_t i2c_list[10] = {0};
static uint8_t i2c_data[0xff] = {0};
static uint32_t last_fsync_count = 0;
static uint32_t last_lsync_count = 0;

static float tecadc_last_volts[4];
static uint16_t tecadc_last_raw[4];
static uint8_t tec_temp_good;
static char retTriggerJson[0xFF];
static float tec_setpoint = 0.0;

static _Bool process_controller_command(UartPacket *uartResp, UartPacket *cmd)
{
    _Bool ret = true;
    int iRet = 0;
    uartResp->command = cmd->command;
    switch (cmd->command)
    {
        case OW_CTRL_I2C_SCAN:
            uartResp->command = OW_CTRL_I2C_SCAN;
            if(cmd->data_len != 2){
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }else{
                memset(i2c_list, 0, 10);
                iRet = TCA9548A_scan_channel(cmd->data[0], cmd->data[1], i2c_list, 10, false);
                if(iRet < 0){
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
            uartResp->command = OW_CTRL_GET_IND;
            uartResp->reserved = LED_RGB_GET();
            break;
        case OW_CTRL_SET_FAN:
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
                memset(i2c_data, 0, 0xff);
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

            LED_RGB_SET(LED_BLUE); // Blue
            Trigger_Start();
            break;
        case OW_CTRL_STOP_TRIG:
            uartResp->command = OW_CTRL_STOP_TRIG;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            uartResp->data_len = 0;

            LED_RGB_SET(LED_GREEN); // Green
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
        case OW_CTRL_TEC_DAC:
            uartResp->command = OW_CTRL_TEC_DAC;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            if(cmd->reserved == 0){
                uint16_t reg_data = 0;
                if(ad5761r_register_readback(&tec_dac, CMD_RD_DAC_REG, &reg_data) == HAL_OK)
                {
                    float temp_val = 0;
                    temp_val = code_to_volts(&tec_dac, reg_data);
                    uartResp->data = (uint8_t *)&temp_val;
                    uartResp->data_len   = sizeof(float);

                }else{
                  uartResp->data_len = 0;
                  uartResp->data = NULL;
                  uartResp->packet_type = OW_ERROR;
                }
            }
            else if(cmd->reserved == 1 && cmd->data_len == 4){
                float set_voltage;
                memcpy(&set_voltage, cmd->data, sizeof(float));
                uint16_t reg_data = 0;
                reg_data = volts_to_code(&tec_dac, set_voltage);
                uartResp->data_len    = 4;
                uartResp->data = (uint8_t *)&tec_setpoint;
                if(ad5761r_write_update_dac_register(&tec_dac, reg_data)!=0){
                  printf("TEC DAC Failed to set DAC Voltage\r\n");
                  uartResp->data_len = 0;
                  uartResp->data = NULL;
                  uartResp->packet_type = OW_ERROR;
                }else{
                    tec_setpoint       = set_voltage;
                    uartResp->data_len    = 4;
                    uartResp->data = (uint8_t *)&tec_setpoint;
                }
            }else{
                uartResp->data_len = 0;
                uartResp->data = NULL;
                uartResp->packet_type = OW_UNKNOWN;
            }

            break;
        case OW_CTRL_GET_TEMPS:
            uartResp->command = OW_CTRL_GET_TEMPS;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;

            TCA9548A_SelectChannel(1, 1);

            consoleTemps.f.t1 = MAX31875_ReadTemperature(MAX31875_TEMP1_DEV_ADDR);
            consoleTemps.f.t2 = MAX31875_ReadTemperature(MAX31875_TEMP2_DEV_ADDR);
            consoleTemps.f.t3 = MAX31875_ReadTemperature(MAX31875_TEMP3_DEV_ADDR);

            uartResp->data_len    = sizeof(consoleTemps.bytes);
            uartResp->data = consoleTemps.bytes;

            break;
        case OW_CTRL_TEC_STATUS:
            uartResp->command = OW_CTRL_TECADC;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;

            tec_temp_good = HAL_GPIO_ReadPin(TEMPGD_GPIO_Port, TEMPGD_Pin)?0:1;
            uartResp->data_len = 1;
            uartResp->data = &tec_temp_good;
            break;
        case OW_CTRL_TECADC:
            uartResp->command = OW_CTRL_TECADC;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            if(uartResp->reserved>4){
                uartResp->data_len = 0;
                uartResp->packet_type = OW_UNKNOWN;
            }else{
                memset(tecadc_last_volts, 0, 16);
                if(cmd->reserved == 4)
                {
                    if(ADS7924_ReadAllRaw(&tec_ads, tecadc_last_raw, 100) != ADS7924_OK)
                    {
                      printf("Failed to read ADC channels\r\n");
                      uartResp->data_len = 0;
                      uartResp->packet_type = OW_UNKNOWN;
                    }else{
                      for(int i=0; i < 4; i++){
                          tecadc_last_volts[i] = ADS7924_CodeToVolts(&tec_ads, tecadc_last_raw[i]);
                      }
                      uartResp->data_len = 16;
                      uartResp->data = (uint8_t *)tecadc_last_volts;
                    }
                    break;
                }

                if(ADS7924_ReadVoltage(&tec_ads, uartResp->reserved, &tecadc_last_volts[uartResp->reserved], 100) != ADS7924_OK)
                {
                  printf("Failed to read ADC channel %d\r\n", uartResp->reserved);
                  uartResp->data_len = 0;
                  uartResp->packet_type = OW_UNKNOWN;
                }else{
                  uartResp->data_len = 4;
                  uartResp->data = (uint8_t *)&tecadc_last_volts[uartResp->reserved];
                }
            }
            break;
        case OW_CTRL_BOARDID:
            uartResp->command = OW_CTRL_BOARDID;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            uartResp->data_len = 1;
            board_id = BoardV_Read();
            uartResp->data = (uint8_t *)(&board_id);
            break;
        case OW_CTRL_PDUMON:
            uartResp->command = OW_CTRL_PDUMON;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            if (ADS7828_ReadAllChannels2(&adc_mon[0], &pdu_frame.f.raws[0], &pdu_frame.f.vals[0]) != HAL_OK) {
                  printf("Failed to read ADC MON 0\r\n");
                  uartResp->data_len = 0;
                  uartResp->data = NULL;
                  uartResp->packet_type = OW_ERROR;
                  break;
            }
            if (ADS7828_ReadAllChannels2(&adc_mon[1], &pdu_frame.f.raws[8], &pdu_frame.f.vals[8]) != HAL_OK) {
                  printf("Failed to read ADC MON 1\r\n");
                  uartResp->data_len = 0;
                  uartResp->data = NULL;
                  uartResp->packet_type = OW_ERROR;
                  break;
            }
            uartResp->data_len = (uint16_t)sizeof(pdu_frame);
            uartResp->data = pdu_frame.bytes;
            break;
        case OW_CTRL_MCP42_SET_WIPER:
            uartResp->command = OW_CTRL_MCP42_SET_WIPER;
            if (cmd->data_len != 3) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint8_t ch = cmd->data[0];
                uint16_t pos = 0;
                memcpy(&pos, &cmd->data[1], sizeof(uint16_t));
                if (mcp42u83_set_wiper(&mcp42u83_device, (mcp42u83_pot_channel)ch, pos) == HAL_OK) {
                    uartResp->data_len = 2;
                    uartResp->data = &cmd->data[1];
                } else {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                }
            }
            break;
        case OW_CTRL_MCP42_SET_BOTH:
            uartResp->command = OW_CTRL_MCP42_SET_BOTH;
            if (cmd->data_len != 2) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint16_t pos = 0;
                memcpy(&pos, &cmd->data[0], sizeof(uint16_t));
                if (mcp42u83_set_both_wipers(&mcp42u83_device, pos) == HAL_OK) {
                    uartResp->data_len = 2;
                    uartResp->data = cmd->data;
                } else {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                }
            }
            break;
        case OW_CTRL_MCP42_SET_WIPERS:
            uartResp->command = OW_CTRL_MCP42_SET_WIPERS;
            if (cmd->data_len != 4) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint16_t pos0 = 0;
                uint16_t pos1 = 0;
                memcpy(&pos0, &cmd->data[0], sizeof(uint16_t));
                memcpy(&pos1, &cmd->data[2], sizeof(uint16_t));
                if (mcp42u83_set_wipers(&mcp42u83_device, pos0, pos1) == HAL_OK) {
                    uartResp->data_len = 4;
                    uartResp->data = cmd->data;
                } else {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                }
            }
            break;
        case OW_CTRL_MCP42_GET_WIPER:
            uartResp->command = OW_CTRL_MCP42_GET_WIPER;
            if (cmd->data_len != 1) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                static uint8_t mcp42_resp_buf[2];
                uint8_t ch = cmd->data[0];
                uint16_t pos = mcp42u83_get_wiper(&mcp42u83_device, (mcp42u83_pot_channel)ch);
                memcpy(mcp42_resp_buf, &pos, sizeof(uint16_t));
                uartResp->data_len = 2;
                uartResp->data = mcp42_resp_buf;
            }
            break;
        case OW_CTRL_MCP42_SHUTDOWN:
            uartResp->command = OW_CTRL_MCP42_SHUTDOWN;
            if (cmd->data_len != 1) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint8_t ch = cmd->data[0];
                HAL_StatusTypeDef st;
                if (ch == 3) {
                    st = mcp42u83_shutdown_both(&mcp42u83_device);
                } else {
                    st = mcp42u83_shutdown(&mcp42u83_device, (mcp42u83_pot_channel)ch);
                }
                if (st != HAL_OK) {
                    uartResp->packet_type = OW_ERROR;
                }
            }
            break;
        case OW_CTRL_MCP42_WAKEUP:
            uartResp->command = OW_CTRL_MCP42_WAKEUP;
            if (cmd->data_len != 3) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint8_t ch = cmd->data[0];
                uint16_t pos = 0;
                memcpy(&pos, &cmd->data[1], sizeof(uint16_t));
                if (mcp42u83_wakeup(&mcp42u83_device, (mcp42u83_pot_channel)ch, pos) != HAL_OK) {
                    uartResp->packet_type = OW_ERROR;
                }
            }
            break;
        case OW_CTRL_MCP42_SET_RES:
            uartResp->command = OW_CTRL_MCP42_SET_RES;
            if (cmd->data_len != 5) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint8_t ch = cmd->data[0];
                float res = 0.0f;
                memcpy(&res, &cmd->data[1], sizeof(float));
                if (mcp42u83_set_resistance(&mcp42u83_device, (mcp42u83_pot_channel)ch, res) != HAL_OK) {
                    uartResp->packet_type = OW_ERROR;
                }
            }
            break;
        case OW_CTRL_MCP42_INC:
            uartResp->command = OW_CTRL_MCP42_INC;
            if (cmd->data_len != 2) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint8_t ch = cmd->data[0];
                uint8_t steps = cmd->data[1];
                if (mcp42u83_increment(&mcp42u83_device, (mcp42u83_pot_channel)ch, steps) != HAL_OK) {
                    uartResp->packet_type = OW_ERROR;
                }
            }
            break;
        case OW_CTRL_MCP42_DEC:
            uartResp->command = OW_CTRL_MCP42_DEC;
            if (cmd->data_len != 2) {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            } else {
                uint8_t ch = cmd->data[0];
                uint8_t steps = cmd->data[1];
                if (mcp42u83_decrement(&mcp42u83_device, (mcp42u83_pot_channel)ch, steps) != HAL_OK) {
                    uartResp->packet_type = OW_ERROR;
                }
            }
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
            break;
        case OW_CMD_NOP:
            break;
        case OW_CMD_VERSION:
            uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
            uartResp->data = FIRMWARE_VERSION_DATA;
            break;
        case OW_CMD_ECHO:
            uartResp->data_len = cmd->data_len;
            uartResp->data = cmd->data;
            break;
        case OW_CMD_HWID:
            id_words[0] = HAL_GetUIDw0();
            id_words[1] = HAL_GetUIDw1();
            id_words[2] = HAL_GetUIDw2();
            uartResp->data_len = 16;
            uartResp->data = (uint8_t *)&id_words;
            break;
        case OW_CMD_TOGGLE_LED:
            break;
        case OW_CMD_RESET:
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
        break;
    }

    return true;
}
