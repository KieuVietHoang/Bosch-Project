/*********************************************************/
/*****   Node 2 stand-in, driven by CAN2 on this board *****/
/*********************************************************/
/*
 * The lab's Verification Board plays Node 2 in the real setup. This module
 * puts the same behaviour on this board's second CAN controller, so Node 1
 * can be exercised end to end with one board and no external equipment.
 *
 * It is a genuine hardware test, not a software mock: CAN2 is a separate
 * controller with its own transceiver, and the frames really cross a
 * physical bus between the two. Wire CAN1's CANH/CANL to CAN2's, with the
 * usual 120 ohm termination - the same rig the Diagnostic practice used.
 *
 * What it does, per the spec's description of Node 2:
 *   - transmits 0x0A2 every 20 ms: Value0 in byte 0, Value1 in byte 1, a
 *     message counter cycling 0x0..0xF in byte 6, everything else zero;
 *   - receives Node 1's 0x012 and checks it the way the real Verification
 *     Board does: byte2 = Value0+Value1, byte6 = checksum over byte0..5,
 *     bytes 3/4/5/7 zero, and Value0/Value1 echoing a pair this side
 *     actually sent.
 *
 * The 20 ms cadence comes from HAL_GetTick(), not a second hardware timer.
 * Node 2 is only a test fixture - its timing is not graded, and TIM2 is
 * reserved for Node 1's 50 ms send, which is. Using SysTick here also
 * keeps the two cadences independent, so the 20/50 ms interleaving stays
 * realistic rather than locked in step.
 *
 * Roles are mirrored, not shared: Node 1's 0x012 is this side's RX, and
 * this side's 0x0A2 is Node 1's RX. can012.h names both IDs from Node 1's
 * point of view, so the sense of TX/RX is swapped throughout node2sim.c.
 */

#ifndef _NODE2SIM_H
#define _NODE2SIM_H

#include <stdint.h>

/* 1 = print a line for every frame sent and every verdict. 0 = only the
 * 2-second summary, plus any FAIL (failures always print either way). */
#define NODE2SIM_TRACE_FRAMES 0

/* Diagnostic build: put CAN2 in CAN_MODE_SILENT and stop transmitting.
 *
 * Why this exists. On hardware CAN2 went straight to LEC=bit-dom, TEC=248,
 * BOFF - it transmits, then hears something on its RX line that does not
 * match what it believes it sent. That is the signature of a controller
 * whose RX is connected to a live bus while its TX reaches no transceiver:
 * the PB12/PB13 remap taking effect on the MCU side while the board's
 * CAN2 transceiver stays wired to PB5/PB6.
 *
 * Silent mode settles it. The cell never transmits, so it cannot raise a
 * bit error and cannot go bus-off - it can only listen. If a "N2 GOT"
 * line then appears carrying Node 1's 0x012, the RX half works and only
 * the TX half is unwired, which is a wiring problem with a known fix. If
 * nothing is heard at all, neither half reaches the transceiver.
 *
 * Set back to 0 for normal operation. */
#define NODE2SIM_LISTEN_ONLY 0

/* 1 = ignore the PB12/PB13 remap and drive CAN2 on its default PB5/PB6.
 *
 * The remap was there to keep PB6 free for the LCD backlight. Hardware
 * says it does not work: with CAN2 on PB12/PB13, CAN1 transmitted onto a
 * real bus while CAN2 reported REC=0 - it heard nothing at all, because
 * the board's CAN2 transceiver is wired to PB5/PB6 regardless of what the
 * MCU is told to do. Remapping moves pins, not copper.
 *
 * Sharing PB6 with the backlight costs almost nothing: CAN2_TX idles
 * recessive (high = lit), and each frame pulls it low for a fraction of
 * 260 us out of every 20 ms. Under 1% dimming, invisible.
 *
 * Implemented in HAL_CAN_MspInit()'s USER CODE block in
 * stm32f4xx_hal_msp.c, so the .ioc does not need regenerating. */
#define NODE2SIM_CAN2_DEFAULT_PINS 1

void Node2Sim_Init(void);

/* Called from HAL_CAN_RxFifo0MsgPendingCallback() for CAN2. Drains the
 * mailbox, keeping the RX buffers private here. */
void Node2Sim_HandleCanRx(void);

/* Call every iteration of the main loop, after Node1_Periodic(). Sends the
 * periodic 0x0A2 when due and reports the verdict on any 0x012 received. */
void Node2Sim_Periodic(void);

#endif
