/*********************************************************/
/*****       Node 1 application logic (this board)    *****/
/*********************************************************/
/*
 * Node 1 = this board. Node 2 (Verification board) is external, already
 * built hardware - nothing here implements Node 2's behaviour.
 *
 * ISR/main-loop split, same rule as the Diagnostic practice: interrupt
 * handlers only copy data and set a flag, every blocking call (CAN TX,
 * UART, LCD) happens in Node1_Periodic() from the main loop.
 */

#ifndef _NODE1_H
#define _NODE1_H

#include <stdint.h>

/* Self-test mode, for bringing this board up on its own.
 *
 * 0 = normal operation. CAN1 in normal mode on a real bus, talking to a
 *     real second node - either the lab's Verification Board, or the
 *     second Open405R-C running 05_BEA_CANCommunication_Node2. This is
 *     both the graded configuration and the two-board test configuration.
 *
 * 1 = LOOPBACK, for when no second board is at hand. CAN1 runs in
 *     CAN_MODE_SILENT_LOOPBACK and a fake Node 2 is simulated purely in
 *     software:
 *       - a 0x0A2 is injected every 20 ms straight into the RX path, with
 *         V0/V1 changing and a message counter cycling 0..F in byte 6;
 *       - every 0x012 the CAN peripheral transmits comes back into its own
 *         RX FIFO, is re-checked (sum, checksum, padding) and printed as
 *         "N1 self ... PASS"/"FAIL".
 *     Nothing reaches the physical bus - no transceiver, no second node,
 *     no bus wiring and no pin conflicts.
 *
 * A loud banner prints over UART at startup while this is non-zero, so
 * demoing in a test mode by accident is hard to miss.
 *
 * (There used to be a mode 2 that drove this board's own CAN2 as a
 * stand-in Node 2. It is gone: a real second board does that job now, and
 * far more convincingly - two independent CAN controllers on two
 * independently clocked boards.) */
#define NODE1_SELFTEST 0

/* Diagnostics: the 2-second summary line, the "dt=<n>ms" gap measured on
 * every send, and the raw CAN register dump. Independent of the self-test
 * mode on purpose - "dt=" is exactly what proves the 50 ms +/-1 ms
 * requirement, and that matters most in mode 0, on the real bus, against
 * the real second node.
 *
 * The output costs almost nothing now: uart_log.c queues it into a ring
 * buffer that DMA drains in the background, so no print ever blocks the
 * CPU and none of it can delay the CAN TX it is measuring. Set to 0 only
 * if you want an entirely silent build. */
#define NODE1_DIAG 1

/* Set to 0 to leave the LCD completely alone - no Lcd_Init(), no drawing.
 * That also frees PB6 (LCD_BL), which is CAN2_TX as well - see the pin
 * parking note in MX_GPIO_Init(). Turn it back to 1 once the CAN side is
 * confirmed. */
#define NODE1_USE_LCD 1

/* 1 = print every frame (labelled which node sent it to which). 0 = print
 * only the 2-second summary plus any failure, which is far easier to read
 * when the question is simply "is the transmission correct". Failures
 * always print regardless of this setting. */
#define NODE1_TRACE_FRAMES 1

/* Each of the 3 trace line kinds (Node1->Node2, Node2->Node1, Node2's PASS
 * verdict) is throttled to print at most once per this many ms - purely a
 * display-rate knob, the real send/receive/check underneath always runs at
 * the real 20/50 ms rate regardless of this value. 0 disables the throttle
 * (print every single frame, 100+ lines/s). A FAIL always prints
 * immediately no matter what this is set to.
 *
 * Left at 0 (real rate): printed lines then follow true chronological
 * order 1:1 with real events, which a nonzero value does not guarantee
 * (each line kind samples independently, so two adjacent printed lines can
 * come from different points in time - see [[bea-can-communication-frame]]
 * for a worked example). The tradeoff is the display can outrun a slow
 * terminal at 100+ lines/s; if that becomes a problem, raise this instead
 * of reaching for a heavier terminal app. */
#define NODE1_TRACE_INTERVAL_MS 0u

#if NODE1_DIAG
/* Formats a bxCAN CAN_ESR register value into readable text. Diagnostic
 * scaffolding: without a scope or a debugger this
 * is the only way to tell "my frames are fine but nobody is acknowledging
 * them" (LEC=ACK, TEC climbing) from a bus that is genuinely quiet.
 * Takes the raw register value so node1.h stays free of HAL types. */
void Node1_FormatCanEsr(uint32_t esr, char *buf, uint16_t len);
#endif

void Node1_Init(void);

/* Owns the whole CAN1_RX0_IRQHandler body, so the RX header/data buffers
 * stay private here and a failed HAL_CAN_GetRxMessage() can never leave a
 * stale ID to be acted on. */
void Node1_HandleCanIrq(void);

/* Feeds one received frame into the application. Filters on CAN012_ID_RX
 * internally; ignores anything else. ISR-safe. */
void Node1_OnCanRx(uint32_t stdId, const uint8_t data[8]);

/* Call from the 50 ms hardware timer ISR (TIM2). ISR-safe. */
void Node1_OnTimerTick(void);

/* Call every iteration of the main loop. Does the actual CAN transmit,
 * UART logging and LCD refresh queued up by the two callbacks above. */
void Node1_Periodic(void);

#endif
