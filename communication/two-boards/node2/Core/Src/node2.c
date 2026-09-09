/*********************************************************/
/*****     Node 2 application logic (second board)    *****/
/*********************************************************/
/* See node2.h. */

#include "node2.h"
#include "can012.h"
#include "main.h"
#include "uart_log.h"
#include <stdio.h>
#include <string.h>

#if NODE2_USE_LCD
#include "lcd.h"
#endif

/* 4 * 20 ms = 80 ms of history, comfortably longer than Node 1's 50 ms
 * reply period - so a 0x012 can be matched against a pair this board
 * really sent, even though Node 1 always answers with a value that is by
 * then one or two frames old. */
#define NODE2_ECHO_HISTORY 4u

static uint8_t s_value0, s_value1;
static uint8_t s_counter;                 /* message counter, byte 6, 0x0..0xF */

static uint8_t s_sentV0[NODE2_ECHO_HISTORY];
static uint8_t s_sentV1[NODE2_ECHO_HISTORY];
static uint8_t s_sentIdx;

static uint8_t s_rxFrame[8];              /* last 0x012, exactly as received */
static volatile uint8_t s_rxPending;
static volatile uint8_t s_sendDue;

static uint32_t s_statTick;
static uint16_t s_txCount, s_pass, s_fail;

/* s_rxFrame is written from the CAN1 RX ISR, so main-loop reads take a
 * snapshot with interrupts briefly masked. */
#define NODE2_ENTER_CRITICAL()  uint32_t prim_ = __get_PRIMASK(); __disable_irq()
#define NODE2_EXIT_CRITICAL()   __set_PRIMASK(prim_)

static void Node2_LogText(const char *text)
{
    UartLog_Write(text, (uint16_t)strlen(text));
}

/* 'tag' names sender and receiver explicitly (e.g. "N2->N1"), the same
 * convention Node 1 uses, so a capture from either board reads the same. */
static void Node2_LogLine(const char *tag, uint16_t canId, const uint8_t frame[8],
                          const char *suffix)
{
    char line[80];
    int  n = snprintf(line, sizeof(line),
                      "%-7s 0x%03X: %02X %02X %02X %02X %02X %02X %02X %02X%s\r\n",
                      tag, canId,
                      frame[0], frame[1], frame[2], frame[3],
                      frame[4], frame[5], frame[6], frame[7],
                      (suffix != NULL) ? suffix : "");
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }
}

void Node2_Init(void)
{
    s_value0    = 0u;
    s_value1    = 0u;
    s_counter   = 0u;
    s_sentIdx   = 0u;
    s_rxPending = 0u;
    s_sendDue   = 0u;
    s_txCount   = 0u;
    s_pass      = 0u;
    s_fail      = 0u;
    memset(s_sentV0, 0x00, sizeof(s_sentV0));
    memset(s_sentV1, 0x00, sizeof(s_sentV1));
    memset(s_rxFrame, 0x00, sizeof(s_rxFrame));

#if NODE2_USE_LCD
    Lcd_Init();
#endif

    Node2_LogText("\r\n=== BEA Node2 FW#3 : CAN1 500k, TX 0x0A2 / 20 ms ===\r\n");

    /* Same startup check Node 1 prints: MSR bit0 (INAK) must be 0, or the
     * cell never left initialisation mode and nothing will ever transmit.
     * HAL state 2 = LISTENING (started), 1 = READY, 5 = ERROR. */
    {
        uint32_t mcr = hcan1.Instance->MCR;
        uint32_t msr = hcan1.Instance->MSR;
        const char *verdict =
            ((msr & 0x1u) == 0u) ? "started OK" :
            ((mcr & 0x1u) != 0u) ? "<< STUCK: INRQ still set, Start bailed out" :
                                   "<< STUCK: INRQ cleared but cell wont leave init";

        char line[128];
        int  n = snprintf(line, sizeof(line),
                          "init: CAN1 filt=%u start=%u notif=%u state=%u err=%08lX MCR=%08lX MSR=%08lX %s\r\n",
                          (unsigned)(g_Can1SetupStatus & 0x3u),
                          (unsigned)((g_Can1SetupStatus >> 2) & 0x3u),
                          (unsigned)((g_Can1SetupStatus >> 4) & 0x3u),
                          (unsigned)hcan1.State,
                          (unsigned long)hcan1.ErrorCode,
                          (unsigned long)mcr, (unsigned long)msr, verdict);
        if (n > 0)
        {
            UartLog_Write(line, (uint16_t)n);
        }
    }

    s_statTick = HAL_GetTick() + 2000u;
}

void Node2_HandleCanIrq(void)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];

    HAL_CAN_IRQHandler(&hcan1);
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return;
    }
    /* Node 1's TX is this board's RX. */
    if (header.StdId == CAN012_ID_TX)
    {
        memcpy(s_rxFrame, data, sizeof(s_rxFrame));
        s_rxPending = 1u;
    }
}

void Node2_OnTimerTick(void)
{
    s_sendDue = 1u;
}

/* Same recovery Node 1 has: CAN1 can fail to start when the bus is not
 * idle at power-up, and the HAL handle then stays in ERROR even after the
 * hardware itself recovers. Retries twice a second while broken, costs one
 * comparison once running. Matters here because the two boards are powered
 * up independently - whichever comes up first sees a quiet bus. */
static void Node2_RecoverCan(void)
{
    static uint32_t s_recoverTick;

    if (hcan1.State == HAL_CAN_STATE_LISTENING)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_recoverTick) < 0)
    {
        return;
    }
    s_recoverTick = now + 500u;

    if (HAL_CAN_Init(&hcan1) == HAL_OK)
    {
        MX_CAN1_Setup();
    }
}

static void Node2_Send(void)
{
    /* Values are arbitrary - the spec only fixes the message counter in
     * byte 6 - but they must keep changing, so that Node 1 echoing them
     * back proves it really tracked the latest frame. */
    s_value0 = (uint8_t)(0x2Au + s_counter);
    s_value1 = (uint8_t)(0x0Du + 2u * s_counter);

    uint8_t frame[CAN012_DLC] = {0};
    frame[0] = s_value0;
    frame[1] = s_value1;
    frame[6] = s_counter;
    s_counter = (uint8_t)((s_counter + 1u) & 0x0Fu);

    s_sentV0[s_sentIdx] = s_value0;
    s_sentV1[s_sentIdx] = s_value1;
    s_sentIdx = (uint8_t)((s_sentIdx + 1u) % NODE2_ECHO_HISTORY);

    CAN_TxHeaderTypeDef header = {0};
    uint32_t            mailbox;
    header.StdId = CAN012_ID_RX;    /* 0x0A2: this board -> Node 1 */
    header.IDE   = CAN_ID_STD;
    header.RTR   = CAN_RTR_DATA;
    header.DLC   = CAN012_DLC;
    (void)HAL_CAN_AddTxMessage(&hcan1, &header, frame, &mailbox);

    s_txCount++;
#if NODE2_TRACE_FRAMES
    Node2_LogLine("N2->N1", CAN012_ID_RX, frame, NULL);
#endif
}

/* Was (v0,v1) one of the pairs this board sent recently? */
static uint8_t Node2_WasSent(uint8_t v0, uint8_t v1)
{
    for (uint8_t i = 0u; i < NODE2_ECHO_HISTORY; i++)
    {
        if ((s_sentV0[i] == v0) && (s_sentV1[i] == v1))
        {
            return 1u;
        }
    }
    return 0u;
}

static void Node2_CheckReply(void)
{
    uint8_t f[8];

    NODE2_ENTER_CRITICAL();
    memcpy(f, s_rxFrame, sizeof(f));
    s_rxPending = 0u;
    NODE2_EXIT_CRITICAL();

    uint8_t okEcho = Node2_WasSent(f[0], f[1]);
    uint8_t okSum  = (f[2] == (uint8_t)(f[0] + f[1]));
    uint8_t okCrc  = (f[6] == Can012_Checksum(f, 6u));
    uint8_t okPad  = ((f[3] | f[4] | f[5] | f[7]) == 0u);

    if (okEcho && okSum && okCrc && okPad)
    {
        s_pass++;
#if NODE2_USE_LCD
        Lcd_ShowValues(f[0], f[1]);
#endif
#if NODE2_TRACE_FRAMES
        Node2_LogLine("N2 check", CAN012_ID_TX, f, "  PASS");
#endif
    }
    else
    {
        s_fail++;
        /* Failures always print, whatever NODE2_TRACE_FRAMES is set to. */
        const char *verdict;
        if      (!okCrc)  { verdict = "  FAIL crc";  }
        else if (!okSum)  { verdict = "  FAIL sum";  }
        else if (!okEcho) { verdict = "  FAIL echo"; }
        else              { verdict = "  FAIL pad";  }
        Node2_LogLine("N2 check", CAN012_ID_TX, f, verdict);
    }
}

/* One compact line every 2 s: how many 0x0A2 went out, how many 0x012 came
 * back and how many were fully valid, plus CAN1's error state.
 * "got=0" means nothing is arriving from Node 1 at all - check the bus
 * wiring and the 120 ohm termination before suspecting either firmware. */
static void Node2_Stats(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_statTick) < 0)
    {
        return;
    }
    s_statTick = now + 2000u;

    /* CAN_ESR: bit0 EWGF, bit1 EPVF, bit2 BOFF, bits6:4 LEC,
     * bits23:16 TEC, bits31:24 REC. */
    static const char *lecName[8] =
    {
        "ok", "stuff", "form", "ACK", "bit1", "bit0", "crc", "sw"
    };
    uint32_t esr = hcan1.Instance->ESR;

    char line[144];
    int  n = snprintf(line, sizeof(line),
                      "[2s] TX=%u  got=%u pass=%u fail=%u  CAN1 LEC=%s TEC=%u REC=%u%s%s%s\r\n",
                      (unsigned)s_txCount,
                      (unsigned)(s_pass + s_fail), (unsigned)s_pass, (unsigned)s_fail,
                      lecName[(esr >> 4) & 0x7u],
                      (unsigned)((esr >> 16) & 0xFFu),
                      (unsigned)((esr >> 24) & 0xFFu),
                      (esr & 0x1u) ? " WARN" : "",
                      (esr & 0x2u) ? " PASSIVE" : "",
                      (esr & 0x4u) ? " BUSOFF" : "");
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }

    s_txCount = 0u;
    s_pass    = 0u;
    s_fail    = 0u;
}

void Node2_Periodic(void)
{
    Node2_RecoverCan();

    /* Send first, before any logging or LCD work, for the same reason Node 1
     * does: the mailbox load is what sets the frame's position on the bus,
     * so nothing slow may sit in front of it. */
    if (s_sendDue)
    {
        s_sendDue = 0u;
        Node2_Send();
    }

    if (s_rxPending)
    {
        Node2_CheckReply();
    }

    Node2_Stats();

#if NODE2_USE_LCD
    Lcd_Periodic();
#endif
}
