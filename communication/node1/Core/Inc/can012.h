/*********************************************************/
/***** CAN Communication practice - frame 0x012/0x0A2 *****/
/*********************************************************/
/*
 * Byte layout per CANBoardPractice_CANCommunication_Final.docx:
 *
 *   0x0A2 (Node 2 -> Node 1, every 20 ms)
 *     byte0 : Value0
 *     byte1 : Value1
 *     byte6 : message counter 0x0-0xF   (Node 1 must ignore this field)
 *
 *   0x012 (Node 1 -> Node 2, every 50 ms +/-1 ms)
 *     byte0 : Value0            (copied from the latest 0x0A2)
 *     byte1 : Value1            (copied from the latest 0x0A2)
 *     byte2 : Value0 + Value1   (modulo 256)
 *     byte3..5 : 0x00
 *     byte6 : checksum over byte0..5
 *     byte7 : 0x00
 *
 * The checksum is named "CRC 8 SAE J1850" in the spec but the embedded
 * reference C snippet does NOT match the textbook SAE J1850 parameters.
 * Verified against the two vectors printed in the spec:
 *   {2A 0D 00 A3 22 00 6C} -> 0xB4
 *   {A2 5A FE 9F 8C 04 10} -> 0xA3
 * Matching parameters: init 0x00, no output XOR, bytes folded in reverse
 * order (last byte first). Do NOT substitute a textbook J1850 routine.
 */

#ifndef _CAN012_H
#define _CAN012_H

#include <stdint.h>

#define CAN012_ID_TX   0x012u   /* Node 1 -> Node 2 */
#define CAN012_ID_RX   0x0A2u   /* Node 2 -> Node 1 */
#define CAN012_DLC     8u

/* Checksum over the first 'len' bytes of 'data', matching the spec's
 * reference implementation (see file header for the verified parameters). */
uint8_t Can012_Checksum(const uint8_t *data, uint8_t len);

/* Builds the 8-byte 0x012 payload from the latest received Value0/Value1.
 * 'out' must point to an 8-byte buffer. */
void Can012_BuildFrame(uint8_t value0, uint8_t value1, uint8_t out[CAN012_DLC]);

#endif
