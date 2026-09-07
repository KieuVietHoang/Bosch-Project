/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_seca.h"

typedef enum
{
    DCM_SECA_LOCKED = 0,
    DCM_SECA_SEED_SENT,
    DCM_SECA_UNLOCKED
} Dcm_Seca_State_t;

static Dcm_Seca_State_t g_SecaState;
static uint8_t  g_SecaSeed[4];
static uint32_t g_SecaUnlockDeadlineMs;
static uint32_t g_SecaPenaltyDeadlineMs;
static uint8_t  g_SecaPenaltyActive;
static uint32_t g_SecaRngState;

static void Dcm_Seca_Lock(void)
{
    g_SecaState = DCM_SECA_LOCKED;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   /* LED0 off */
}

/* Simple xorshift32 PRNG - good enough to satisfy "random except
 * 0x00000000 / 0xFFFFFFFF", not meant to be cryptographically secure. */
static uint32_t Dcm_Seca_NextRandom(void)
{
    g_SecaRngState ^= g_SecaRngState << 13;
    g_SecaRngState ^= g_SecaRngState >> 17;
    g_SecaRngState ^= g_SecaRngState << 5;
    return g_SecaRngState;
}

static void Dcm_Seca_GenerateSeed(void)
{
    uint32_t seedValue;

    do
    {
        seedValue = Dcm_Seca_NextRandom();
    } while ((seedValue == 0x00000000u) || (seedValue == 0xFFFFFFFFu));

    g_SecaSeed[0] = (uint8_t)((seedValue >> 24) & 0xFFu);
    g_SecaSeed[1] = (uint8_t)((seedValue >> 16) & 0xFFu);
    g_SecaSeed[2] = (uint8_t)((seedValue >> 8)  & 0xFFu);
    g_SecaSeed[3] = (uint8_t)(seedValue & 0xFFu);
}

/* KEY-0 = SEED-0 XOR SEED-1 ; KEY-1 = SEED-1 + SEED-2
 * KEY-2 = SEED-2 XOR SEED-3 ; KEY-3 = SEED-3 + SEED-0   (spec, 4-seed variant) */
static void Dcm_Seca_ComputeKey(const uint8_t seed[4], uint8_t key[4])
{
    key[0] = seed[0] ^ seed[1];
    key[1] = (uint8_t)(seed[1] + seed[2]);
    key[2] = seed[2] ^ seed[3];
    key[3] = (uint8_t)(seed[3] + seed[0]);
}

void Dcm_Seca_Init(void)
{
    /* Seed the PRNG from the temperature ADC + current tick so it differs
     * across resets without needing a hardware RNG peripheral. */
    g_SecaRngState = (uint32_t)g_TemperatureSensorRawValue_u16[0] ^ TimeStamp ^ 0xA5A5A5A5u;
    if (g_SecaRngState == 0u)
    {
        g_SecaRngState = 0xDEADBEEFu;
    }

    g_SecaPenaltyActive = 0u;
    Dcm_Seca_Lock();
}

void Dcm_Seca_Periodic(void)
{
    /* Signed-difference comparison: safe even if TimeStamp (uint32) wraps
     * around after ~49 days of uptime. */
    if (g_SecaState == DCM_SECA_UNLOCKED)
    {
        if ((int32_t)(TimeStamp - g_SecaUnlockDeadlineMs) >= 0)
        {
            Dcm_Seca_Lock();
        }
    }

    if (g_SecaPenaltyActive)
    {
        if ((int32_t)(TimeStamp - g_SecaPenaltyDeadlineMs) >= 0)
        {
            g_SecaPenaltyActive = 0u;
        }
    }
}

uint8_t Dcm_Seca_IsLevel1Unlocked(void)
{
    return (g_SecaState == DCM_SECA_UNLOCKED) ? 1u : 0u;
}

void Dcm_Seca_Handle(const uint8_t *reqData, uint8_t reqLen)
{
    uint8_t subFunction;

    if (reqLen < 2u)
    {
        Dcm_SendNegative(SID_SECURITY, NRC_INVALID_LENGTH);
        return;
    }

    subFunction = reqData[1];

    if (subFunction == DCM_SECA_LEVEL1_SEED_SUBFUNC)
    {
        uint8_t resp[8];

        if (g_SecaPenaltyActive)
        {
            /* Spec: invalid key -> 10 s delay before the ECU can receive
             * and process the next seed request. Stay silent meanwhile. */
            return;
        }

        Dcm_Seca_GenerateSeed();
        g_SecaState = DCM_SECA_SEED_SENT;

        resp[0] = SID_SECURITY + SID_POSITIVE_OFFSET;   /* 0x67 */
        resp[1] = DCM_SECA_LEVEL1_SEED_SUBFUNC;
        resp[2] = g_SecaSeed[0];
        resp[3] = g_SecaSeed[1];
        resp[4] = g_SecaSeed[2];
        resp[5] = g_SecaSeed[3];
        Dcm_SendPositive(resp, 6u);
    }
    else if (subFunction == DCM_SECA_LEVEL1_KEY_SUBFUNC)
    {
        uint8_t expectedKey[4];
        uint8_t resp[2];

        if (reqLen != 6u)
        {
            Dcm_SendNegative(SID_SECURITY, NRC_INVALID_LENGTH);
            return;
        }
        if (g_SecaState != DCM_SECA_SEED_SENT)
        {
            /* No outstanding seed to answer against. */
            Dcm_SendNegative(SID_SECURITY, NRC_INVALID_KEY);
            return;
        }

        Dcm_Seca_ComputeKey(g_SecaSeed, expectedKey);

        if ((reqData[2] == expectedKey[0]) && (reqData[3] == expectedKey[1]) &&
            (reqData[4] == expectedKey[2]) && (reqData[5] == expectedKey[3]))
        {
            g_SecaState            = DCM_SECA_UNLOCKED;
            g_SecaUnlockDeadlineMs = TimeStamp + DCM_SECA_UNLOCK_DURATION_MS;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   /* LED0 on */

            resp[0] = SID_SECURITY + SID_POSITIVE_OFFSET;
            resp[1] = DCM_SECA_LEVEL1_KEY_SUBFUNC;
            Dcm_SendPositive(resp, 2u);
        }
        else
        {
            Dcm_Seca_Lock();
            g_SecaPenaltyActive     = 1u;
            g_SecaPenaltyDeadlineMs = TimeStamp + DCM_SECA_PENALTY_DURATION_MS;
            Dcm_SendNegative(SID_SECURITY, NRC_INVALID_KEY);
        }
    }
    else
    {
        Dcm_SendNegative(SID_SECURITY, NRC_GENERAL_REJECT);
    }
}
