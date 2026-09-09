/*********************************************************/
/*****      Node 1 application logic - on CAN1        *****/
/*********************************************************/
/*
 * Node 1 is the graded half of the practice: it receives 0x0A2, and every
 * 50 ms sends 0x012 carrying the values it last received, their sum and a
 * checksum.
 *
 * On this board Node 2 is played by CAN2 (see node2sim.h), so both halves
 * run on one MCU and talk over a real CAN bus wired between the two
 * transceivers. Nothing in this file knows that - as far as Node 1 is
 * concerned there is simply another node out on the bus, which is exactly
 * how it will behave against the lab's Verification Board.
 *
 * ISR/main-loop split, same rule as the Diagnostic practice: interrupt
 * handlers only copy data and set a flag; every slow call (CAN TX, UART,
 * LCD) happens in Node1_Periodic() from the main loop.
 */

#ifndef _NODE1_H
#define _NODE1_H

#include <stdint.h>

/* Diagnostics: the 2-second summary line, the "dt=<n>ms" gap measured on
 * every send, and the raw CAN register dump. "dt=" is exactly what proves
 * the 50 ms +/-1 ms requirement, so this is worth leaving on even for a
 * demo - the output costs almost nothing, since uart_log.c queues it into
 * a ring buffer that DMA drains in the background. */
#define NODE1_DIAG 1

/* Set to 0 to leave the LCD completely alone - no Lcd_Init(), no drawing.
 * Useful while bringing the CAN side up, so a display fault cannot be
 * mistaken for a bus fault. */
#define NODE1_USE_LCD 1

/* 1 = print every frame (labelled which node sent it to which). 0 = print
 * only the 2-second summary plus any failure, which is far easier to read
 * when the question is simply "is the transmission correct". Failures
 * always print regardless of this setting. */
#define NODE1_TRACE_FRAMES 0

/* Each trace line kind is throttled to at most one print per this many ms.
 * Purely a display-rate knob - the real send/receive underneath always runs
 * at the true 20/50 ms rate. 0 disables the throttle (every frame prints,
 * ~90 lines/s), which keeps printed lines in true 1:1 chronological order
 * with real events. Raise it if a slow terminal cannot keep up. */
#define NODE1_TRACE_INTERVAL_MS 0u

#if NODE1_DIAG
/* Formats a bxCAN CAN_ESR register value into readable text. Without a
 * debugger this is the only way to tell "my frames are fine but nobody
 * acknowledges them" (LEC=ACK, TEC climbing) from a genuinely quiet bus.
 * Takes the raw register value so node1.h stays free of HAL types. */
void Node1_FormatCanEsr(uint32_t esr, char *buf, uint16_t len);
#endif

void Node1_Init(void);

/* Called from HAL_CAN_RxFifo0MsgPendingCallback() for CAN1. Drains the
 * mailbox, so the RX header/data buffers stay private here and a failed
 * HAL_CAN_GetRxMessage() can never leave a stale ID to be acted on. */
void Node1_HandleCanRx(void);

/* Call from the 50 ms hardware timer ISR (TIM2). ISR-safe. */
void Node1_OnTimerTick(void);

/* Call every iteration of the main loop. Does the actual CAN transmit,
 * UART logging and LCD refresh queued up by the callbacks above. */
void Node1_Periodic(void);

#endif
