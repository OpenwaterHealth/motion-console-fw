/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOGGING_H
#define __LOGGING_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <main.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

 void init_dma_logging();
 bool is_using_dma();

 /* Runtime debug-flag control (RAM-only; see DEBUG_FLAG_* in common.h). */
 void     logging_set_debug_flags(uint32_t flags);
 uint32_t logging_get_debug_flags(void);

 /* Drain buffered USB-printf log data to the host. Call from the main loop
  * only — never from an ISR. No-op unless DEBUG_FLAG_USB_PRINTF is set. */
 void logging_pump(void);

 void logging_UART_TxCpltCallback(UART_HandleTypeDef *huart);
 void logging_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
 void logging_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __PACKBURN_LOGGING_H*/
