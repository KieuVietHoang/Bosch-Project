/*********************************************************/
/*****CAN Transport Protocol (ISO 15765-2) - Single Frame *****/
/*********************************************************/

#ifndef _CAN_TP_H
#define _CAN_TP_H

#include "main.h"

#define CANTP_PAD_BYTE      0x55u   /* NFR-05: unused bytes padded with 0x55 */
#define CANTP_SF_MAX_LEN    7u      /* byte0 = PCI, byte1..7 = payload (max 7) */

/* Encode 'len' bytes of 'data' as a CAN-TP Single Frame and transmit it on
 * 'hcan' with standard ID 'canId'. Always sends DLC=8, padded with
 * CANTP_PAD_BYTE. Returns the HAL_CAN_AddTxMessage status. */
HAL_StatusTypeDef CanTp_SendSF(CAN_HandleTypeDef *hcan, uint32_t canId,
                                const uint8_t *data, uint8_t len);

/* Decode a raw 8-byte CAN payload as a Single Frame.
 * Returns 1 and fills outData/outLen on success (PCI high nibble == 0 and
 * declared length in 1..7). Returns 0 for any other frame type
 * (First/Consecutive/Flow Control) or invalid length - multi-frame is not
 * implemented, caller should ignore those frames for now. */
uint8_t CanTp_DecodeSF(const uint8_t rxData[8], uint8_t *outData, uint8_t *outLen);

#endif
