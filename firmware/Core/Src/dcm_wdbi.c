/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_wdbi.h"
#include "dcm_seca.h"

void Dcm_Wdbi_Handle(const uint8_t *reqData, uint8_t reqLen)
{
    uint16_t did;
    uint32_t newCanId;
    uint8_t  resp[1];

    if (reqLen < 5u)
    {
        Dcm_SendNegative(SID_WRITE_DID, NRC_INVALID_LENGTH);
        return;
    }

    did = ((uint16_t)reqData[1] << 8) | reqData[2];

    if (did != DID_TESTER_CANID)
    {
        Dcm_SendNegative(SID_WRITE_DID, NRC_DID_NOT_SUPPORTED);
        return;
    }

    if (!Dcm_Seca_IsLevel1Unlocked())
    {
        Dcm_SendNegative(SID_WRITE_DID, NRC_SECURITY_DENIED);
        return;
    }

    /* byte1: 0x00-0x7F, byte2: 0x00-0xFF per spec's request-format table */
    newCanId = ((uint32_t)reqData[3] << 8) | reqData[4];
    Dcm_SetPendingCanId(newCanId);   /* applied after the next ignition cycle (BtnU) */

    resp[0] = SID_WRITE_DID + SID_POSITIVE_OFFSET;   /* 0x6E, no DID echoed per spec */
    Dcm_SendPositive(resp, 1u);
}
