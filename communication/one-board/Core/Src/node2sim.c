/*********************************************************/
/*****   Node 2 stand-in, driven by CAN2 on this board *****/
/*********************************************************/
/* See node2sim.h. */

#include "node2sim.h"
#include "can012.h"
#include "node1.h"   /* for Node1_FormatCanEsr under NODE1_DIAG */
#include "main.h"
#include "uart_log.h"
#include <stdio.h>
#include <string.h>

#define NODE2SIM_PERIOD_MS 20u

/* 4 * 20 ms = 80 ms of history, comfortably longer than Node 1's 50 ms
 * reply period - so a 0x012 can be matched against a pair this side really
 * sent, even though Node 1 always answers with a value that is by then one
 * or two frames old. */
#define NODE2SIM_ECHO_HISTORY 4u

static uint8_t s_counter;                 /* message counter, byte 6, 0x0..0xF */
static uint32_t s_nextSendTick;

static uint8_t s_sentV0[NODE2SIM_ECHO_HISTORY];
static uint8_t s_sentV1[NODE2SIM_ECHO_HISTORY];
static uint8_t s_sentIdx;

static uint8_t s_rxFrame[8];              /* last 0x012, exactly as received */
static volatile uint8_t s_rxPending;

static uint32_t s_statTick;
static uint16_t s_txCount, s_pass, s_fail;

/* s_rxFrame is written from the CAN2 RX ISR, so main-loop reads take a
 * snapshot with interrupts briefly masked. */
#define NODE2SIM_ENTER_CRITICAL()  uint32_t prim_ = __get_PRIMASK(); __disable_irq()
#define NODE2SIM_EXIT_CRITICAL()   __set_PRIMASK(prim_)

/* Same line format Node 1 uses, and the same "who did what" tag rule:
 * "N2 sent" is what this side put on the bus, "N2 GOT " is what it really
 * received back. Never reuse one tag for both a send and a receive - a log
 * that cannot tell those apart hides a completely dead bus. */
static void Node2Sim_LogLine(const char *tag, uint16_t canId, const uint8_t frame[8],
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

void Node2Sim_Init(void)
{
    s_counter   = 0u;
    s_sentIdx   = 0u;
    s_rxPending = 0u;
    s_txCount   = 0u;
    s_pass      = 0u;
    s_fail      = 0u;
    memset(s_sentV0, 0x00, sizeof(s_sentV0));
    memset(s_sentV1, 0x00, sizeof(s_sentV1));
    memset(s_rxFrame, 0x00, sizeof(s_rxFrame));

    s_nextSendTick = HAL_GetTick() + NODE2SIM_PERIOD_MS;
    s_statTick     = HAL_GetTick() + 2000u;

    /* Same INAK check Node 1 prints, for the second controller. If CAN2 is
     * stuck in initialisation mode nothing it sends ever reaches the bus,
     * and Node 1 would simply look like it is receiving nothing. */
    {
        uint32_t mcr = hcan2.Instance->MCR;
        uint32_t msr = hcan2.Instance->MSR;
        const char *verdict =
            ((msr & 0x1u) == 0u) ? "started OK" :
            ((mcr & 0x1u) != 0u) ? "<< STUCK: INRQ still set, Start bailed out" :
                                   "<< STUCK: INRQ cleared but cell wont leave init";

        char line[128];
        int  n = snprintf(line, sizeof(line),
                          "init: CAN2 state=%u err=%08lX MCR=%08lX MSR=%08lX %s\r\n",
                          (unsigned)hcan2.State, (unsigned long)hcan2.ErrorCode,
                          (unsigned long)mcr, (unsigned long)msr, verdict);
        if (n > 0)
        {
            UartLog_Write(line, (uint16_t)n);
        }
    }
}

void Node2Sim_HandleCanRx(void)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];

    /* Dispatched from HAL_CAN_IRQHandler(), so this only drains the
     * mailbox - the read is also what clears the pending condition. */
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return;
    }
    /* Node 1's TX is this side's RX. */
    if (header.StdId == CAN012_ID_TX)
    {
        memcpy(s_rxFrame, data, sizeof(s_rxFrame));
        s_rxPending = 1u;
    }
}

/* Mirror of Node 1's recovery: CAN2 can fail to start when the bus is not
 * idle at power-up, and the HAL handle then stays in ERROR even after the
 * hardware itself recovers. */
static void Node2Sim_RecoverCan(void)
{
    static uint32_t s_recoverTick;

    /* Same BUS-OFF trap as Node 1 - see the note there. This is the cell
     * that actually hit it on hardware: LEC=bit-dom, TEC=248, BOFF. */
    uint8_t busOff = ((hcan2.Instance->ESR & 0x4u) != 0u);
    if ((hcan2.State == HAL_CAN_STATE_LISTENING) && !busOff)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_recoverTick) < 0)
    {
        return;
    }
    s_recoverTick = now + 500u;

    if (HAL_CAN_Init(&hcan2) == HAL_OK)
    {
        Can_Setup();
    }
}

/* Everything from here to the matching #endif belongs to the transmit half,
 * which listen-only mode deliberately does not build. */
#if NODE2SIM_LISTEN_ONLY == 0

static void Node2Sim_Send(void)
{
    /* The values are arbitrary - the spec only fixes the message counter in
     * byte 6 - but they must keep changing, so that Node 1 echoing them
     * back proves it really tracked the latest frame rather than repeating
     * a constant. */
    uint8_t v0 = (uint8_t)(0x2Au + s_counter);
    uint8_t v1 = (uint8_t)(0x0Du + 2u * s_counter);

    uint8_t frame[CAN012_DLC] = {0};
    frame[0] = v0;
    frame[1] = v1;
    frame[6] = s_counter;
    s_counter = (uint8_t)((s_counter + 1u) & 0x0Fu);

    s_sentV0[s_sentIdx] = v0;
    s_sentV1[s_sentIdx] = v1;
    s_sentIdx = (uint8_t)((s_sentIdx + 1u) % NODE2SIM_ECHO_HISTORY);

    CAN_TxHeaderTypeDef header = {0};
    uint32_t            mailbox;
    header.StdId = CAN012_ID_RX;    /* 0x0A2: this side -> Node 1 */
    header.IDE   = CAN_ID_STD;
    header.RTR   = CAN_RTR_DATA;
    header.DLC   = CAN012_DLC;
    (void)HAL_CAN_AddTxMessage(&hcan2, &header, frame, &mailbox);

    s_txCount++;
#if NODE2SIM_TRACE_FRAMES
    Node2Sim_LogLine("N2 sent", CAN012_ID_RX, frame, NULL);
#endif
}

/* Was (v0,v1) one of the pairs this side sent recently? */
static uint8_t Node2Sim_WasSent(uint8_t v0, uint8_t v1)
{
    for (uint8_t i = 0u; i < NODE2SIM_ECHO_HISTORY; i++)
    {
        if ((s_sentV0[i] == v0) && (s_sentV1[i] == v1))
        {
            return 1u;
        }
    }
    return 0u;
}

#endif /* NODE2SIM_LISTEN_ONLY == 0 - end of the transmit half */

static void Node2Sim_CheckReply(void)
{
    uint8_t f[8];

    NODE2SIM_ENTER_CRITICAL();
    memcpy(f, s_rxFrame, sizeof(f));
    s_rxPending = 0u;
    NODE2SIM_EXIT_CRITICAL();

    /* In listen-only mode this side never transmits, so there is no history
     * for Node 1 to have echoed - the check would fail every time and say
     * nothing useful. Everything else about the frame is still verified. */
#if NODE2SIM_LISTEN_ONLY
    uint8_t okEcho = 1u;
#else
    uint8_t okEcho = Node2Sim_WasSent(f[0], f[1]);
#endif
    uint8_t okSum  = (f[2] == (uint8_t)(f[0] + f[1]));
    uint8_t okCrc  = (f[6] == Can012_Checksum(f, 6u));
    uint8_t okPad  = ((f[3] | f[4] | f[5] | f[7]) == 0u);

    if (okEcho && okSum && okCrc && okPad)
    {
        s_pass++;
        /* The first few always print, whatever the trace switches say -
         * "did this controller ever hear anything?" must not be a question
         * the log can be configured to hide. */
        static uint8_t s_firstRx;
        if (s_firstRx < 5u)
        {
            s_firstRx++;
            Node2Sim_LogLine("N2 GOT ", CAN012_ID_TX, f, "  PASS  <<< RX WORKS");
        }
#if NODE2SIM_TRACE_FRAMES
        else
        {
            Node2Sim_LogLine("N2 GOT ", CAN012_ID_TX, f, "  PASS");
        }
#endif
    }
    else
    {
        s_fail++;
        /* Failures always print, whatever NODE2SIM_TRACE_FRAMES is set to -
         * naming which check failed is the whole diagnostic value. */
        const char *verdict;
        if      (!okCrc)  { verdict = "  FAIL crc";  }
        else if (!okSum)  { verdict = "  FAIL sum";  }
        else if (!okEcho) { verdict = "  FAIL echo"; }
        else              { verdict = "  FAIL pad";  }
        Node2Sim_LogLine("N2 GOT ", CAN012_ID_TX, f, verdict);
    }
}

static void Node2Sim_Stats(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_statTick) < 0)
    {
        return;
    }
    s_statTick = now + 2000u;

    /* Reuses Node 1's ESR decoder rather than duplicating the bit layout.
     * It only exists when NODE1_DIAG is on, so the register text goes away
     * with it and the counters still print. */
#if NODE1_DIAG
    char esr[48];
    Node1_FormatCanEsr(hcan2.Instance->ESR, esr, sizeof(esr));
#else
    static const char esr[] = "";
#endif

    char line[128];
    int  n = snprintf(line, sizeof(line),
                      "[2s] N2 TX=%u pass=%u fail=%u  CAN2 %s\r\n",
                      (unsigned)s_txCount, (unsigned)s_pass,
                      (unsigned)s_fail, esr);
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }

    s_txCount = 0u;
    s_pass    = 0u;
    s_fail    = 0u;
}

void Node2Sim_Periodic(void)
{
    Node2Sim_RecoverCan();

#if NODE2SIM_LISTEN_ONLY == 0
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_nextSendTick) >= 0)
    {
        /* Advance by exactly one period rather than from "now", so a late
         * call does not push the whole cadence out permanently. */
        s_nextSendTick += NODE2SIM_PERIOD_MS;
        Node2Sim_Send();
    }
#endif

    if (s_rxPending)
    {
        Node2Sim_CheckReply();
    }

    Node2Sim_Stats();
}
