/*********************************************************/
/*****   Non-blocking UART log (ring buffer + DMA)     *****/
/*********************************************************/
/*
 * Why this exists: every trace/debug line in the node logic used
 * to go out via a blocking HAL_UART_Transmit() call, which busy-waits for
 * ~3-4 ms per line at 115200 baud. At the full self-test trace rate
 * (~90 lines/s, all 4 line kinds unthrottled) that is enough blocking time
 * per 50 ms window to occasionally delay how promptly Node1_Periodic()
 * notices the TIM2 tick and loads the CAN1 TX mailbox - measured on
 * hardware as the TX period alternating 48/52 ms instead of a flat 50 ms.
 * See CLAUDE.md and [[bea-can-communication-frame]] for the measurement.
 *
 * UartLog_Write() fixes this by only ever copying bytes into a RAM ring
 * buffer (fast, bounded, no peripheral wait) and letting USART3's DMA
 * channel drain that buffer in the background. The CPU is never blocked
 * waiting for a UART byte to finish shifting out, at any print rate - so
 * this module can absorb the full, unthrottled trace without touching the
 * CAN TX timing.
 *
 * If the ring buffer would overflow (producing faster than the UART can
 * drain, e.g. printing continuously well beyond ~11.5 KB/s), a write is
 * dropped wholesale rather than emitting a torn/partial line. Debug
 * scaffolding losing an occasional line under sustained overload is a much
 * better failure mode than corrupting the CAN timing it exists to observe.
 */

#ifndef _UART_LOG_H
#define _UART_LOG_H

#include <stdint.h>

/* Call once after huart3 is initialised (MX_USART3_UART_Init already ran).
 * Sets up the DMA channel used to drain the ring buffer. */
void UartLog_Init(void);

/* Queues 'len' bytes for transmission and returns immediately - never
 * blocks, never waits on the UART. Drops the write entirely (silently) if
 * the ring buffer does not have room for all of it. */
void UartLog_Write(const char *data, uint16_t len);


#endif
