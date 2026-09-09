/*********************************************************/
/*****     Node 2 application logic (second board)    *****/
/*********************************************************/
/*
 * This is the SECOND physical board, standing in for the lab's
 * Verification Board so Node 1 can be exercised end to end without it.
 *
 * Same hardware as Node 1 (Open405R-C / STM32F405), same CAN1 pins
 * (PA11/PA12) and the same 500 kbit/s bit timing - the two boards are
 * joined by a real CAN bus, one wire pair plus termination.
 *
 * What it does, per the spec's description of Node 2:
 *   - transmits 0x0A2 every 20 ms: Value0 in byte 0, Value1 in byte 1,
 *     a message counter cycling 0x0..0xF in byte 6, everything else 0;
 *   - receives Node 1's 0x012 and checks it the way the real Verification
 *     Board does - "acknowledge the receiving of 0x012 and valid CRC":
 *     byte2 = Value0+Value1, byte6 = checksum over byte0..5, byte3/4/5/7
 *     zero, and Value0/Value1 echoing a pair this board actually sent.
 *
 * Roles are deliberately mirrored, not shared: Node 1's 0x012 is this
 * board's RX, and this board's 0x0A2 is Node 1's RX. can012.h names the
 * two IDs from Node 1's point of view, so the sense of TX/RX is swapped
 * here - see the CAN012_ID_* uses in node2.c.
 *
 * ISR/main-loop split is the same rule as Node 1: interrupt handlers only
 * copy data and set a flag; every blocking or slow call happens in
 * Node2_Periodic() from the main loop.
 */

#ifndef _NODE2_H
#define _NODE2_H

#include <stdint.h>

/* 1 = print a line for every frame sent/received. 0 = only the 2-second
 * summary, plus any FAIL (failures always print either way). */
#define NODE2_TRACE_FRAMES 1

/* Mirror of Node 1's switch: 0 disables all LCD work, which frees PB6
 * (LCD_BL) and keeps the display out of the way while bringing the CAN
 * side up. Set to 1 once CAN is confirmed. */
#define NODE2_USE_LCD 1

void Node2_Init(void);

/* Owns the whole CAN1_RX0_IRQHandler body so the RX buffers stay private
 * here and a failed HAL_CAN_GetRxMessage() can never leave a stale ID to
 * be acted on. */
void Node2_HandleCanIrq(void);

/* Call from the 20 ms hardware timer ISR (TIM2). ISR-safe. */
void Node2_OnTimerTick(void);

/* Call every iteration of the main loop: sends the periodic 0x0A2 and
 * reports the verdict for any 0x012 received from Node 1. */
void Node2_Periodic(void);

#endif
