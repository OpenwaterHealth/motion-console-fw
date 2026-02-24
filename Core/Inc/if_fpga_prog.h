/* if_commands.h - command processing API */
#ifndef INC_IF_FPGA_PROG_COMMANDS_H_
#define INC_IF_FPGA_PROG_COMMANDS_H_

#include "common.h"
#include "XO2_dev.h"

void if_cmd_set_xo2_prog_handle(XO2Handle_t *h);
_Bool process_fpga_prog_command(UartPacket *uartResp, UartPacket *cmd);

extern ConsoleTemperatures consoleTemps;

#endif /* INC_IF_FPGA_PROG_COMMANDS_H_ */
