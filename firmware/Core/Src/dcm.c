/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm.h"
/*for further services please add service header here*/
#include "dcm_rdbi.h"
#include "dcm_wdbi.h"
#include "dcm_seca.h"

static uint32_t g_DcmReqCanId          = DCM_REQ_CAN_ID_DEFAULT;
static uint32_t g_DcmPendingCanId      = 0u;
static uint8_t  g_DcmPendingCanIdValid = 0u;

void Dcm_Init(void)
{
    g_DcmReqCanId          = DCM_REQ_CAN_ID_DEFAULT;
    g_DcmPendingCanIdValid = 0u;
    Dcm_Seca_Init();
}

void Dcm_Periodic(void)
{
    Dcm_Seca_Periodic();
}

uint32_t Dcm_GetCurrentReqCanId(void)
{
    return g_DcmReqCanId;
}

void Dcm_SetPendingCanId(uint32_t newCanId)
{
    g_DcmPendingCanId      = newCanId & 0x7FFu;   /* clamp to 11-bit standard ID */
    g_DcmPendingCanIdValid = 1u;
}

void Dcm_ApplyPendingCanId(void)
{
    if (g_DcmPendingCanIdValid)
    {
        g_DcmReqCanId          = g_DcmPendingCanId;
        g_DcmPendingCanIdValid = 0u;
    }
}

void Dcm_SendPositive(const uint8_t *data, uint8_t len)
{
    (void)CanTp_SendSF(&hcan2, DCM_RESP_CAN_ID, data, len);
}

void Dcm_SendNegative(uint8_t requestSid, uint8_t nrc)
{
    uint8_t resp[3];
    resp[0] = 0x7Fu;
    resp[1] = requestSid;
    resp[2] = nrc;
    (void)CanTp_SendSF(&hcan2, DCM_RESP_CAN_ID, resp, 3u);
}

void Dcm_HandleRequest(const uint8_t *reqData, uint8_t reqLen)
{
    if (reqLen == 0u)
    {
        return;
    }

    switch (reqData[0])
    {
        case SID_READ_DID:
            Dcm_Rdbi_Handle(reqData, reqLen);
            break;
        case SID_SECURITY:
            Dcm_Seca_Handle(reqData, reqLen);
            break;
        case SID_WRITE_DID:
            Dcm_Wdbi_Handle(reqData, reqLen);
            break;
        default:
            /* Spec: NRC is out of scope for this practice - ECU stays
             * silent on unsupported SIDs instead of answering 0x7F 0x11. */
            break;
    }
}
