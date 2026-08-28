/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_rdbi.h"

/* STM32F405 internal temperature sensor: V25=0.76V, slope=2.5mV/degC,
 * VREF=3.3V, 12-bit ADC. Same formula used throughout the project notes. */
static uint8_t Dcm_Rdbi_ReadTemperature(void)
{
    uint32_t raw_mv;
    int32_t  temp_c;

    raw_mv = ((uint32_t)g_TemperatureSensorRawValue_u16[0] * 3300u) / 4095u;
    temp_c = (((int32_t)raw_mv - 760) * 10) / 25 + 25;

    if (temp_c < 0)   { temp_c = 0; }
    if (temp_c > 255) { temp_c = 255; }
    return (uint8_t)temp_c;
}

void Dcm_Rdbi_Handle(const uint8_t *reqData, uint8_t reqLen)
{
    uint16_t did;
    uint8_t  resp[8];

    if (reqLen != 3u)
    {
        Dcm_SendNegative(SID_READ_DID, NRC_INVALID_LENGTH);
        return;
    }

    did = ((uint16_t)reqData[1] << 8) | reqData[2];   /* Motorola / big endian */

    switch (did)
    {
        case DID_TESTER_CANID:
        {
            uint32_t canId = Dcm_GetCurrentReqCanId();
            resp[0] = SID_READ_DID + SID_POSITIVE_OFFSET;   /* 0x62 */
            resp[1] = reqData[1];
            resp[2] = reqData[2];
            resp[3] = (uint8_t)((canId >> 8) & 0xFFu);
            resp[4] = (uint8_t)(canId & 0xFFu);
            Dcm_SendPositive(resp, 5u);
            break;
        }
        case DID_TEMPERATURE:
        {
            resp[0] = SID_READ_DID + SID_POSITIVE_OFFSET;
            resp[1] = reqData[1];
            resp[2] = reqData[2];
            resp[3] = Dcm_Rdbi_ReadTemperature();
            Dcm_SendPositive(resp, 4u);
            break;
        }
        default:
            Dcm_SendNegative(SID_READ_DID, NRC_DID_NOT_SUPPORTED);
            break;
    }
}
