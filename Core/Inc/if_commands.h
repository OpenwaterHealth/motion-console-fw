/* if_commands.h - command processing API */
#ifndef INC_IF_COMMANDS_H_
#define INC_IF_COMMANDS_H_

#include "common.h"

_Bool process_if_command(UartPacket *uartResp, UartPacket *cmd);

extern ConsoleTemperatures consoleTemps;

#endif /* INC_IF_COMMANDS_H_ */
