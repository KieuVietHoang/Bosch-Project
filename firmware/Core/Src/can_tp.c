/*********************************************************/
/*****CAN Transport Protocol (ISO 15765-2) - Single Frame *****/
/*********************************************************/

#include "can_tp.h"
#include <string.h>

HAL_StatusTypeDef CanTp_SendSF(CAN_HandleTypeDef *hcan, uint32_t canId,
                                const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t frame[8];

    if (len > CANTP_SF_MAX_LEN)
    {
        len = CANTP_SF_MAX_LEN;   /* defensive clamp, should not happen */
    }

    memset(frame, CANTP_PAD_BYTE, sizeof(frame));
    frame[0] = len & 0x0Fu;              /* SF PCI: high nibble 0 = Single Frame */
    memcpy(&frame[1], data, len);

    txHeader.StdId = canId;
    txHeader.ExtId = 0;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8u;                 /* NFR-05: ECU frames always DLC=8 */
    txHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &txHeader, frame, &txMailbox);
}

uint8_t CanTp_DecodeSF(const uint8_t rxData[8], uint8_t *outData, uint8_t *outLen)
{
    uint8_t pciType = (rxData[0] >> 4) & 0x0Fu;
    uint8_t len     = rxData[0] & 0x0Fu;

    if (pciType != 0x00u)
    {
        return 0u;   /* not a Single Frame (FF/CF/FC) - not handled yet */
    }
    if ((len == 0u) || (len > CANTP_SF_MAX_LEN))
    {
        return 0u;
    }

    memcpy(outData, &rxData[1], len);
    *outLen = len;
    return 1u;
}
