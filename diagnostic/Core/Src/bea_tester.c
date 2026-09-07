/*********************************************************/
/***** BEA Tester - cau noi UART (PC) <-> CAN1 (bus) ******/
/*********************************************************/

#include "bea_tester.h"
#include "can_tp.h"
#include "dcm.h"
#include <string.h>
#include <stdio.h>

volatile uint32_t g_Can1RxCount = 0u;
volatile uint32_t g_Can2RxCount = 0u;

static const uint8_t s_sof[3] = { BEA_SOF_0, BEA_SOF_1, BEA_SOF_2 };
static const uint8_t s_eof[3] = { BEA_EOF_0, BEA_EOF_1, BEA_EOF_2 };

/* ---- Chieu PC -> CAN: do khung BEA tu dong byte UART ------------------ */
static uint8_t  s_sofMatch;                    /* da khop bao nhieu byte SOF */
static uint8_t  s_inPayload;                   /* dang o giua khung? */
static uint8_t  s_rxBuf[BEA_MAX_PAYLOAD + 3u]; /* payload + cho chua EOF */
static uint16_t s_rxLen;

static volatile uint8_t s_reqReady;            /* co: da co 1 request hoan chinh */
static uint8_t  s_reqPayload[BEA_MAX_PAYLOAD];
static uint8_t  s_reqLen;

/* ---- Chieu CAN -> PC: dap ung tu ECU cho gui len UART ----------------- */
static volatile uint8_t s_respReady;
static uint8_t  s_respPayload[BEA_MAX_PAYLOAD];
static uint8_t  s_respLen;

void BeaTester_Init(void)
{
    s_sofMatch  = 0u;
    s_inPayload = 0u;
    s_rxLen     = 0u;
    s_reqReady  = 0u;
    s_respReady = 0u;
}

void BeaTester_OnUartByte(uint8_t b)
{
    if (!s_inPayload)
    {
        /* Dang tim 0F FF F0 */
        if (b == s_sof[s_sofMatch])
        {
            s_sofMatch++;
            if (s_sofMatch >= 3u)
            {
                s_inPayload = 1u;
                s_sofMatch  = 0u;
                s_rxLen     = 0u;
            }
        }
        else
        {
            /* Khong khop: byte nay co the la byte dau cua SOF moi */
            s_sofMatch = (b == s_sof[0]) ? 1u : 0u;
        }
        return;
    }

    /* Dang trong khung: gom byte, dong thoi soi duoi xem da gap EOF chua */
    if (s_rxLen >= sizeof(s_rxBuf))
    {
        /* Khung qua dai -> bo, quay ve cho SOF moi (chan tran bo dem) */
        s_inPayload = 0u;
        s_rxLen     = 0u;
        s_sofMatch  = 0u;
        return;
    }
    s_rxBuf[s_rxLen++] = b;

    if ((s_rxLen >= 3u) &&
        (s_rxBuf[s_rxLen - 3u] == s_eof[0]) &&
        (s_rxBuf[s_rxLen - 2u] == s_eof[1]) &&
        (s_rxBuf[s_rxLen - 1u] == s_eof[2]))
    {
        uint16_t payLen = s_rxLen - 3u;   /* bo 3 byte EOF o cuoi */

        /* Bo qua neu request truoc chua kip xu ly - tranh ghi de */
        if ((payLen > 0u) && (payLen <= BEA_MAX_PAYLOAD) && (s_reqReady == 0u))
        {
            memcpy(s_reqPayload, s_rxBuf, payLen);
            s_reqLen   = (uint8_t)payLen;
            s_reqReady = 1u;
        }
        s_inPayload = 0u;
        s_rxLen     = 0u;
    }
}

void BeaTester_OnCanResponse(const uint8_t *data, uint8_t len)
{
    if (s_respReady != 0u)
    {
        return;   /* dap ung truoc chua gui xong */
    }
    if (len > BEA_MAX_PAYLOAD)
    {
        len = BEA_MAX_PAYLOAD;
    }
    memcpy(s_respPayload, data, len);
    s_respLen   = len;
    s_respReady = 1u;
}

static void BeaTester_SendFramedToPc(const uint8_t *payload, uint8_t len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s_sof, 3u, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t *)payload, len, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t *)s_eof, 3u, HAL_MAX_DELAY);
}

#if BEA_DEBUG_TRACE
/* In dinh ky trang thai hai bo dieu khien CAN.
 * ESR (Error Status Register) la chia khoa chan doan:
 *   bit0 EWGF=1 : canh bao loi         bit1 EPVF=1 : error passive
 *   bit2 BOFF=1 : BUS-OFF (chet han)   bit6..4 LEC : ma loi cuoi cung
 * LEC=3 (ACK error) => phat di nhung KHONG AI tra loi ACK
 *                      -> dau day sai / CAN2 khong nghe duoc / thieu 120 ohm */
static uint32_t s_dbgLastMs = 0u;

static void BeaTester_DebugStatus(void)
{
    char dbg[96];

    if ((uint32_t)(TimeStamp - s_dbgLastMs) < 2000u)
    {
        return;
    }
    s_dbgLastMs = TimeStamp;

    sprintf(dbg, "[DBG] C1esr=%08lX C2esr=%08lX rx1=%lu rx2=%lu\r\n",
            (unsigned long)hcan1.Instance->ESR,
            (unsigned long)hcan2.Instance->ESR,
            (unsigned long)g_Can1RxCount,
            (unsigned long)g_Can2RxCount);
    USART3_SendString((uint8_t *)dbg);
}
#endif

void BeaTester_Periodic(void)
{
    if (s_reqReady != 0u)
    {
        uint8_t len = s_reqLen;
        HAL_StatusTypeDef st;

        /* Chua ho tro phat da khung: gioi han 7 byte cua Single Frame.
         * Du cho ca 4 dich vu cua de bai (dai nhat la $2E: 5 byte). */
        if (len > CANTP_SF_MAX_LEN)
        {
            len = CANTP_SF_MAX_LEN;
        }

        /* Phat tren dung ID ma ECU dang lang nghe. Dung chung nguon voi DCM
         * de sau khi $2E doi CAN ID (va qua chu ky Ignition) thi Tester van
         * bam theo duoc - dac thu cua mo hinh CAN2CAN tren cung mot board. */
        st = CanTp_SendSF(&hcan1, Dcm_GetCurrentReqCanId(), s_reqPayload, len);

#if BEA_DEBUG_TRACE
        {
            char dbg[80];
            sprintf(dbg, "[DBG] REQ %uB -> id=%03lX TX=%s\r\n",
                    (unsigned)len,
                    (unsigned long)Dcm_GetCurrentReqCanId(),
                    (st == HAL_OK) ? "OK" : "FAIL");
            USART3_SendString((uint8_t *)dbg);
        }
#else
        (void)st;
#endif
        s_reqReady = 0u;
    }

    if (s_respReady != 0u)
    {
        BeaTester_SendFramedToPc(s_respPayload, s_respLen);
        s_respReady = 0u;
    }

#if BEA_DEBUG_TRACE
    BeaTester_DebugStatus();
#endif
}
