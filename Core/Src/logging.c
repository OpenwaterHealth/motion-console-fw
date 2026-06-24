#include "main.h"
#include "logging.h"
#include "common.h"
#include "utils.h"
#include "usbd_cdc_if.h"
#include "lwrb.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

static bool bInit_dma = false;
volatile bool bPrintfTransferComplete = false;

static uint8_t usart_start_tx_dma_transfer(void);
static void    log_buffer_write(const uint8_t *src, size_t count);

/* Ring buffer for TX data - Must be in DMA-accessible memory (not DTCM) */
lwrb_t usart_tx_buff;
uint8_t usart_tx_buff_data[1024] __attribute__((section(".dma_buffer")));
volatile size_t usart_tx_dma_current_len;

/* ---------------------------------------------------------------------------
 * USB-printf mirroring (OW_CMD_DEBUG_FLAGS / DEBUG_FLAG_USB_PRINTF)
 *
 * When enabled, _write() copies printf bytes into a dedicated ring buffer in
 * addition to the existing UART path. logging_pump() drains that ring from the
 * main loop and ships it to the host as framed OW_DATA/OW_CMD_ECHO packets over
 * the same USB CDC link that carries command responses.
 *
 * Safety rules that keep this from disturbing the command path:
 *   - _write() only buffers; it never transmits, blocks, or re-enters the USB
 *     send path, so it stays safe to call from any context (incl. ISRs).
 *   - All transmits happen in logging_pump() from the main loop, never an ISR.
 *   - A dedicated log_tx_buf is used, so the shared command txBuffer is never
 *     touched.
 *   - Ring writes run under a short critical section so a printf from an ISR
 *     can't corrupt the buffer state.
 * ------------------------------------------------------------------------- */
static volatile uint32_t debug_flags = 0x00000000u;

#define LOG_MSG_BUFFER_SIZE 2048u
static lwrb_t  log_msg_rb;
static uint8_t log_msg_buffer[LOG_MSG_BUFFER_SIZE];
static bool    log_msg_rb_initialized = false;
static bool    log_drop_marker_pending = false;

/* Max log payload shipped per pump. Bounds both the framed packet size and the
 * time spent with interrupts disabled while copying out of the ring. */
#define LOG_TX_PAYLOAD_MAX 1024u
#define LOG_FRAME_OVERHEAD 12u /* start+id(2)+type+cmd+addr+rsvd+len(2)+crc(2)+end */
static uint8_t log_tx_buf[LOG_TX_PAYLOAD_MAX + LOG_FRAME_OVERHEAD];

/* CDC TX handshake state, owned by uart_comms.c and shared across the single
 * superloop. tx_busy guards against stepping on an in-flight command response;
 * tx_flag is raised by CDC_handle_TxCpltCallback(). */
extern volatile uint8_t tx_flag;
extern volatile uint8_t tx_busy;

/* Keep the log flush from stalling the superloop if the host stops reading. */
#ifndef LOG_TX_TIMEOUT_MS
#define LOG_TX_TIMEOUT_MS 50u
#endif


#ifdef __GNUC__
	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
		 set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#ifdef __GNUC__
int _write(int fd, const void *buf, size_t count){
	UNUSED(fd);
	uint8_t * src = (uint8_t *)buf;
	if(bInit_dma)
	{
	    if (lwrb_get_free(&usart_tx_buff) >= count) {
	        lwrb_write(&usart_tx_buff, buf, count);
	        usart_start_tx_dma_transfer();
	    }
	}
	else
	{
		HAL_StatusTypeDef com_tx_status = HAL_UART_Transmit(&DEBUG_UART, src, count, 10);
		if(com_tx_status != HAL_OK)
		{
			Error_Handler();
		}
	}

	/* Mirror to the USB CDC host link when enabled. Buffer only — the actual
	 * transmit happens in logging_pump() from the main loop, so this stays
	 * non-blocking, ISR-safe, and can't re-enter the USB command path. */
	if ((debug_flags & DEBUG_FLAG_USB_PRINTF) != 0u) {
		log_buffer_write(src, count);
	}

	return count;
}
#else
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
	//HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch, 1,10);
	UART_putChar( (uint8_t) ch);
  return ch;
}
#endif

void init_dma_logging()
{
    /* Initialize ringbuff */
    lwrb_init(&usart_tx_buff, usart_tx_buff_data, sizeof(usart_tx_buff_data));

    /* USB-printf log ring (separate from the UART DMA ring). */
    lwrb_init(&log_msg_rb, log_msg_buffer, sizeof(log_msg_buffer));
    log_msg_rb_initialized = true;
    log_drop_marker_pending = false;

    bInit_dma = true;
	bPrintfTransferComplete = true;
}

bool is_using_dma(){
	return bInit_dma;
}

static uint8_t usart_start_tx_dma_transfer(void) {
    if (usart_tx_dma_current_len == 0 && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_buff)) > 0) {

        /* Limit maximal size to transmit at a time */
        if (usart_tx_dma_current_len > 32) {
            usart_tx_dma_current_len = 32;
        }
    	bPrintfTransferComplete = false;
		if(HAL_UART_Transmit_DMA(&DEBUG_UART, (uint8_t*)lwrb_get_linear_block_read_address(&usart_tx_buff), usart_tx_dma_current_len)!= HAL_OK)
		{
			Error_Handler();
		}
    }
    return 1;
}

void logging_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{

}


void logging_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}


void logging_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	bPrintfTransferComplete = true;
    lwrb_skip(&usart_tx_buff, usart_tx_dma_current_len);/* Data sent, ignore these */
    usart_tx_dma_current_len = 0;
    usart_start_tx_dma_transfer();          /* Try to send more data */
}

/* Append printf bytes to the USB-printf ring. Runs under a short critical
 * section because the caller (printf) may be in any context, including an ISR,
 * so concurrent producers must not race on the ring state. Drops oldest data
 * on overflow and marks the gap with '~'. */
static void log_buffer_write(const uint8_t *src, size_t count)
{
	if (src == NULL || count == 0u) {
		return;
	}

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	if (!log_msg_rb_initialized) {
		lwrb_init(&log_msg_rb, log_msg_buffer, sizeof(log_msg_buffer));
		log_msg_rb_initialized = true;
	}

	/* A single write larger than the whole buffer can only keep its tail. */
	if (count >= LOG_MSG_BUFFER_SIZE) {
		src   += (count - (LOG_MSG_BUFFER_SIZE - 1u));
		count  = LOG_MSG_BUFFER_SIZE - 1u;
	}

	/* Make room by discarding the oldest bytes; flag that a gap occurred. */
	size_t freeb = lwrb_get_free(&log_msg_rb);
	if (freeb < count) {
		lwrb_skip(&log_msg_rb, count - freeb);
		log_drop_marker_pending = true;
	}
	if (log_drop_marker_pending) {
		const uint8_t marker = '~';
		(void)lwrb_write(&log_msg_rb, &marker, 1u); /* best-effort */
		log_drop_marker_pending = false;
	}
	(void)lwrb_write(&log_msg_rb, src, count);

	if (primask == 0u) {
		__enable_irq();
	}
}

void logging_set_debug_flags(uint32_t flags)
{
	uint32_t prev = debug_flags;
	debug_flags = flags;
	if (prev != flags) {
		/* This line itself is mirrored to USB when the flag is being turned on,
		 * giving the host an immediate confirmation. */
		printf("Debug flags: 0x%08lX -> 0x%08lX%s\r\n",
		       (unsigned long)prev, (unsigned long)flags,
		       (flags & DEBUG_FLAG_USB_PRINTF) ? " (USB printf ON)" : " (USB printf OFF)");
	}
	if ((flags & DEBUG_FLAG_USB_PRINTF) == 0u) {
		/* Drop any buffered log data so stale lines don't flush later. */
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		if (log_msg_rb_initialized) {
			lwrb_reset(&log_msg_rb);
		}
		log_drop_marker_pending = false;
		if (primask == 0u) {
			__enable_irq();
		}
	}
}

uint32_t logging_get_debug_flags(void)
{
	return debug_flags;
}

void logging_pump(void)
{
	if ((debug_flags & DEBUG_FLAG_USB_PRINTF) == 0u) {
		return;
	}
	if (!log_msg_rb_initialized) {
		return;
	}
	if (__get_IPSR() != 0u) {
		return; /* never transmit from interrupt context */
	}

	static volatile bool flush_in_progress = false;
	if (flush_in_progress) {
		return; /* re-entry guard (a printf in the send path lands here) */
	}
	if (tx_busy) {
		return; /* a command response is mid-transmit; try again next pump */
	}

	/* Copy one chunk out of the ring atomically — the producer (printf) may run
	 * in an ISR and mutate the ring under us otherwise. The bytes leave the ring
	 * here, so a failed USB transmit drops this chunk rather than backing the
	 * ring up forever when no host is reading. */
	uint16_t payload_len = 0;
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		size_t linlen = lwrb_get_linear_block_read_length(&log_msg_rb);
		if (linlen > LOG_TX_PAYLOAD_MAX) {
			linlen = LOG_TX_PAYLOAD_MAX;
		}
		if (linlen > 0u) {
			memcpy(&log_tx_buf[9], lwrb_get_linear_block_read_address(&log_msg_rb), linlen);
			lwrb_skip(&log_msg_rb, linlen);
			payload_len = (uint16_t)linlen;
		}
		if (primask == 0u) {
			__enable_irq();
		}
	}
	if (payload_len == 0u) {
		return;
	}

	flush_in_progress = true;

	/* Frame as OW_DATA / OW_CMD_ECHO with id 0 — the same envelope sensor-fw
	 * uses for log text, so the host demuxes it from command responses. */
	log_tx_buf[0] = OW_START_BYTE;
	log_tx_buf[1] = 0u;                                /* id hi */
	log_tx_buf[2] = 0u;                                /* id lo */
	log_tx_buf[3] = OW_DATA;
	log_tx_buf[4] = OW_CMD_ECHO;
	log_tx_buf[5] = 0u;                                /* addr */
	log_tx_buf[6] = 0u;                                /* reserved */
	log_tx_buf[7] = (uint8_t)(payload_len >> 8);
	log_tx_buf[8] = (uint8_t)(payload_len & 0xFFu);
	/* payload already sits at log_tx_buf[9 .. 9 + payload_len - 1] */
	uint16_t crc = util_crc16(&log_tx_buf[1], payload_len + 8u);
	uint16_t idx = 9u + payload_len;
	log_tx_buf[idx++] = (uint8_t)(crc >> 8);
	log_tx_buf[idx++] = (uint8_t)(crc & 0xFFu);
	log_tx_buf[idx++] = OW_END_BYTE;

	tx_flag = 0;
	tx_busy = 1;
	if (CDC_Transmit_FS(log_tx_buf, idx) == USBD_OK) {
		uint32_t t0 = HAL_GetTick();
		while (!tx_flag && (HAL_GetTick() - t0) < LOG_TX_TIMEOUT_MS) {
			HAL_Delay(1);
		}
	}
	tx_busy = 0;

	flush_in_progress = false;
}
