/*********************************************************/
/*****   Non-blocking UART log (ring buffer + DMA)     *****/
/*********************************************************/
/* See uart_log.h for why this exists. */

#include "uart_log.h"
#include "main.h"

#define UART_LOG_RING_SIZE 1024u   /* ~20+ log lines of headroom */

static uint8_t s_ring[UART_LOG_RING_SIZE];
static volatile uint16_t s_head;       /* next free slot, written only by UartLog_Write() */
static volatile uint16_t s_tail;       /* oldest unsent byte, written only by the DMA callback */
static volatile uint8_t  s_dmaBusy;
static uint16_t          s_lastKickLen;

/* USART3_TX is DMA1 Stream3 Channel4 on the STM32F405 (RM0090 Table 42).
 * The stream itself, its clock, its NVIC line and the link to huart3 are
 * all set up by CubeMX (MX_DMA_Init + HAL_UART_MspInit), so none of that
 * is repeated here - this module owns only the ring buffer. */
void UartLog_Init(void)
{
    s_head       = 0u;
    s_tail       = 0u;
    s_dmaBusy    = 0u;
    s_lastKickLen = 0u;
}

/* Starts (or continues) draining the ring buffer via DMA, one contiguous
 * chunk at a time - if the pending data wraps past the end of the buffer,
 * this sends up to the wrap point now and the rest on the next callback.
 * Safe to call from either main-loop or ISR context. */
static void UartLog_Kick(void)
{
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    if (s_dmaBusy)
    {
        __set_PRIMASK(prim);
        return;
    }
    s_dmaBusy = 1u;
    uint16_t tail = s_tail;
    uint16_t head = s_head;
    __set_PRIMASK(prim);

    uint16_t len = (head >= tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(UART_LOG_RING_SIZE - tail);
    if (len == 0u)
    {
        s_dmaBusy = 0u; /* nothing pending after all */
        return;
    }

    s_lastKickLen = len;
    (void)HAL_UART_Transmit_DMA(&huart3, &s_ring[tail], len);
}

void UartLog_Write(const char *data, uint16_t len)
{
    if (len == 0u)
    {
        return;
    }

    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    uint16_t tail = s_tail;
    uint16_t head = s_head;
    __set_PRIMASK(prim);

    uint16_t used = (head >= tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(UART_LOG_RING_SIZE - tail + head);
    uint16_t free = (uint16_t)(UART_LOG_RING_SIZE - 1u - used); /* 1-byte gap: full != empty */

    if (len > free)
    {
        return; /* drop the whole line rather than emit a truncated one */
    }

    for (uint16_t i = 0u; i < len; i++)
    {
        s_ring[(uint16_t)((head + i) % UART_LOG_RING_SIZE)] = (uint8_t)data[i];
    }

    prim = __get_PRIMASK();
    __disable_irq();
    s_head = (uint16_t)((head + len) % UART_LOG_RING_SIZE);
    __set_PRIMASK(prim);

    UartLog_Kick();
}

/* HAL calls this (weak override) when the DMA transfer completes. Runs in
 * ISR context (from DMA1_Stream3_IRQHandler). */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }
    s_tail    = (uint16_t)((s_tail + s_lastKickLen) % UART_LOG_RING_SIZE);
    s_dmaBusy = 0u;
    UartLog_Kick(); /* more queued? keep draining */
}
