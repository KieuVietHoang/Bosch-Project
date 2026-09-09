/*********************************************************/
/*****      Node 1 application logic - on CAN1        *****/
/*********************************************************/
/* See node1.h. */

#include "node1.h"
#include "can012.h"
#include "main.h"
#include "uart_log.h"
#include <stdio.h>
#include <string.h>

#if NODE1_USE_LCD
#include "lcd.h"
#endif

static uint8_t s_value0, s_value1;
static uint8_t s_rxFrame[8];              /* last 0x0A2, exactly as received */
static volatile uint8_t s_rxPending;
static volatile uint8_t s_sendDue;

#if NODE1_DIAG
static uint32_t s_lastTxTick;             /* for the gap measured on each TX */
static uint32_t s_statTick;               /* next 2-second summary */
static uint16_t s_txCount, s_rxCount;
static uint32_t s_dtMin, s_dtMax;
/* With no debugger these counters are the only way to see whether frames
 * actually leave the mailbox and whether the RX interrupt ever runs. */
static volatile uint16_t s_isrCount;      /* CAN1 RX0 interrupts taken */
static volatile uint16_t s_getErr;        /* HAL_CAN_GetRxMessage failures */
static volatile uint32_t s_lastRxId;      /* ID of the last frame received */
static uint16_t s_txErr;                  /* HAL_CAN_AddTxMessage failures */
#endif

/* s_value0/1 and s_rxFrame are written from the CAN1 RX ISR, so main-loop
 * reads take a snapshot with interrupts briefly masked. Without this a
 * frame arriving mid-read could pair Value0 from one 0x0A2 with Value1
 * from the next, and the 0x012 sent back would match neither. */
#define NODE1_ENTER_CRITICAL()  uint32_t prim_ = __get_PRIMASK(); __disable_irq()
#define NODE1_EXIT_CRITICAL()   __set_PRIMASK(prim_)

static void Node1_LogText(const char *text)
{
    UartLog_Write(text, (uint16_t)strlen(text));
}

/* 'tag' says which node did what, and that distinction is the whole point:
 * "N1 sent" is a frame this node put on the bus, "N1 GOT " is one it
 * actually received. An earlier version labelled the receive line the same
 * way node2sim.c labelled its transmit line - which made a bus carrying
 * nothing at all read like a healthy conversation, and cost a debugging
 * round. 'suffix' is appended after the 8 data bytes, or NULL for none. */
static void Node1_LogLine(const char *tag, uint16_t canId, const uint8_t frame[8],
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

#if NODE1_TRACE_FRAMES
/* Gates a trace line to print at most once per NODE1_TRACE_INTERVAL_MS,
 * independent of the real send/receive rate (which is untouched - only the
 * printout is slowed). Each call site keeps its own 'tick', so the kinds
 * of line are throttled independently. */
static uint8_t Node1_TraceGate(uint32_t *tick)
{
#if NODE1_TRACE_INTERVAL_MS == 0u
    /* #if, not runtime: keeps "always print" free of an unsigned "< 0"
     * comparison, which GCC (rightly) flags as always-false. */
    (void)tick;
    return 1u;
#else
    uint32_t now = HAL_GetTick();
    if ((now - *tick) < NODE1_TRACE_INTERVAL_MS)
    {
        return 0u;
    }
    *tick = now;
    return 1u;
#endif
}
#endif /* NODE1_TRACE_FRAMES */

void Node1_Init(void)
{
    s_value0    = 0u;
    s_value1    = 0u;
    s_rxPending = 0u;
    s_sendDue   = 0u;
    memset(s_rxFrame, 0x00, sizeof(s_rxFrame));

#if NODE1_DIAG
    s_lastTxTick = HAL_GetTick();
    s_statTick   = HAL_GetTick() + 2000u;
    s_txCount    = 0u;
    s_rxCount    = 0u;
    s_dtMin      = 0xFFFFFFFFu;
    s_dtMax      = 0u;
    s_isrCount   = 0u;
    s_getErr     = 0u;
    s_lastRxId   = 0u;
    s_txErr      = 0u;
#endif

#if NODE1_USE_LCD
    /* Runs before the periodic timer starts: the one-off full-screen clear
     * inside blocks for ~117 ms, which would swallow whole 50 ms ticks if
     * TIM2 were already going. main() is ordered accordingly. */
    Lcd_Init();
#endif

    Node1_LogText("\r\n=== BEA Communication (one board): CAN1 = Node1, CAN2 = Node2 ===\r\n");

    /* MSR bit0 (INAK) must be 0, or the cell never left initialisation
     * mode and nothing will ever transmit - a failure that leaves every
     * other part of the firmware looking perfectly healthy. HAL state
     * 2 = LISTENING (started), 1 = READY (Start never succeeded), 5 = ERROR. */
    {
        uint32_t mcr = hcan1.Instance->MCR;
        uint32_t msr = hcan1.Instance->MSR;
        const char *verdict =
            ((msr & 0x1u) == 0u) ? "started OK" :
            ((mcr & 0x1u) != 0u) ? "<< STUCK: INRQ still set, Start bailed out" :
                                   "<< STUCK: INRQ cleared but cell wont leave init";

        char line[128];
        int  n = snprintf(line, sizeof(line),
                          "init: CAN1 state=%u err=%08lX MCR=%08lX MSR=%08lX %s\r\n",
                          (unsigned)hcan1.State, (unsigned long)hcan1.ErrorCode,
                          (unsigned long)mcr, (unsigned long)msr, verdict);
        if (n > 0)
        {
            UartLog_Write(line, (uint16_t)n);
        }
    }
}

void Node1_HandleCanRx(void)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];

#if NODE1_DIAG
    s_isrCount++;
#endif
    /* HAL_CAN_IRQHandler() has already run - it is what dispatched here -
     * so this only drains the mailbox. Reading it is also what clears the
     * pending condition; skipping the read would re-enter immediately. */
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
#if NODE1_DIAG
        s_getErr++;
#endif
        return;
    }
#if NODE1_DIAG
    s_lastRxId = header.StdId;
#endif

    /* Node 2's 0x0A2 is this node's input. Byte 0/1 are the values to echo
     * back in the next 0x012. */
    if (header.StdId == CAN012_ID_RX)
    {
        s_value0 = data[0];
        s_value1 = data[1];
        memcpy(s_rxFrame, data, sizeof(s_rxFrame));
        s_rxPending = 1u;
    }
}

void Node1_OnTimerTick(void)
{
    s_sendDue = 1u;
}

#if NODE1_DIAG
void Node1_FormatCanEsr(uint32_t esr, char *buf, uint16_t len)
{
    static const char *lec[8] =
    {
        "ok", "stuff", "form", "ack", "bit-rec", "bit-dom", "crc", "sw"
    };
    (void)snprintf(buf, len, "LEC=%s TEC=%lu REC=%lu%s%s",
                   lec[(esr >> 4) & 0x7u],
                   (unsigned long)((esr >> 16) & 0xFFu),
                   (unsigned long)((esr >> 24) & 0xFFu),
                   ((esr & 0x2u) != 0u) ? " EPV" : "",
                   ((esr & 0x4u) != 0u) ? " BOFF" : "");
}

/* One summary line every 2 seconds. dt is the measured send gap - that is
 * the 50 ms +/-1 ms criterion, readable straight off the terminal with no
 * scope. Sustained drift means a real problem; occasional 49/51 is just
 * 1 ms tick quantisation. */
static void Node1_Stats(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_statTick) < 0)
    {
        return;
    }
    s_statTick = now + 2000u;

    char esr[48];
    Node1_FormatCanEsr(hcan1.Instance->ESR, esr, sizeof(esr));

    char line[160];
    int  n = snprintf(line, sizeof(line),
                      "[2s] TX=%u RX=%u  dt=%lu..%lu ms  CAN1 %s\r\n"
                      "     dbg isr=%u getErr=%u lastId=%03lX txErr=%u  "
                      "MSR=%08lX TSR=%08lX BTR=%08lX\r\n",
                      (unsigned)s_txCount, (unsigned)s_rxCount,
                      (unsigned long)((s_dtMin == 0xFFFFFFFFu) ? 0u : s_dtMin),
                      (unsigned long)s_dtMax, esr,
                      (unsigned)s_isrCount, (unsigned)s_getErr,
                      (unsigned long)s_lastRxId, (unsigned)s_txErr,
                      (unsigned long)hcan1.Instance->MSR,
                      (unsigned long)hcan1.Instance->TSR,
                      (unsigned long)hcan1.Instance->BTR);
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }

    s_txCount  = 0u;
    s_rxCount  = 0u;
    s_dtMin    = 0xFFFFFFFFu;
    s_dtMax    = 0u;
    s_isrCount = 0u;
}
#endif /* NODE1_DIAG */

/* CAN1 can fail to start when the bus is not idle at power-up, and the HAL
 * handle then stays in ERROR even after the hardware itself recovers.
 * Retries twice a second while broken; costs one comparison once running. */
static void Node1_RecoverCan(void)
{
    static uint32_t s_recoverTick;

    /* Two distinct failures, and checking State alone catches only one. A
     * cell that never started shows up in State. A cell driven to BUS-OFF
     * by 256 transmit errors stays HAL_CAN_STATE_LISTENING while being
     * completely off the bus - and with AutoBusOff disabled it never comes
     * back on its own, so without this it stays dead for good. ESR bit 2
     * is BOFF. */
    uint8_t busOff = ((hcan1.Instance->ESR & 0x4u) != 0u);
    if ((hcan1.State == HAL_CAN_STATE_LISTENING) && !busOff)
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
        Can_Setup();
    }
}

void Node1_Periodic(void)
{
    Node1_RecoverCan();

    /* Timing-critical: load the TX mailbox first, before any UART/LCD work
     * below, so the +/-1 ms send-on-time budget is never eaten by a debug
     * print or a screen redraw issued from this same call. */
    if (s_sendDue)
    {
        s_sendDue = 0u;

        NODE1_ENTER_CRITICAL();
        uint8_t v0 = s_value0;
        uint8_t v1 = s_value1;
        NODE1_EXIT_CRITICAL();

        uint8_t txFrame[CAN012_DLC];
        Can012_BuildFrame(v0, v1, txFrame);

        CAN_TxHeaderTypeDef txHeader = {0};
        uint32_t txMailbox;
        txHeader.StdId = CAN012_ID_TX;
        txHeader.IDE   = CAN_ID_STD;
        txHeader.RTR   = CAN_RTR_DATA;
        txHeader.DLC   = CAN012_DLC;
#if NODE1_DIAG
        if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txFrame, &txMailbox) != HAL_OK)
        {
            s_txErr++;
        }

        uint32_t now = HAL_GetTick();
        uint32_t dt  = now - s_lastTxTick;
        s_lastTxTick = now;
        if (dt < s_dtMin) { s_dtMin = dt; }
        if (dt > s_dtMax) { s_dtMax = dt; }
        s_txCount++;
#if NODE1_TRACE_FRAMES
        {
            static uint32_t s_traceTickTx;
            if (Node1_TraceGate(&s_traceTickTx))
            {
                char gap[16];
                (void)snprintf(gap, sizeof(gap), "  dt=%lums", (unsigned long)dt);
                Node1_LogLine("N1 sent", CAN012_ID_TX, txFrame, gap);
            }
        }
#endif
#else  /* !NODE1_DIAG */
        (void)HAL_CAN_AddTxMessage(&hcan1, &txHeader, txFrame, &txMailbox);
#if NODE1_TRACE_FRAMES
        {
            static uint32_t s_traceTickTx;
            if (Node1_TraceGate(&s_traceTickTx))
            {
                Node1_LogLine("N1 sent", CAN012_ID_TX, txFrame, NULL);
            }
        }
#endif
#endif /* NODE1_DIAG */

#if NODE1_USE_LCD
        /* Deliberately after the mailbox load, never before: the screen can
         * afford to hear about the frame a few microseconds late, the bus
         * cannot. */
        Lcd_ShowFrame(0u, txFrame);   /* CAn1 row */
#endif
    }

    if (s_rxPending)
    {
        s_rxPending = 0u;

        uint8_t rx[8];
        NODE1_ENTER_CRITICAL();
        memcpy(rx, s_rxFrame, sizeof(rx));
        NODE1_EXIT_CRITICAL();

#if NODE1_DIAG
        s_rxCount++;
#endif
#if NODE1_USE_LCD
        Lcd_ShowFrame(1u, rx);        /* CAn2 row: cheap, no SPI here */
#endif
        /* The first few receives print no matter how the trace switches are
         * set. "Did this node ever hear anything at all?" is the question a
         * dead bus makes you ask, and the answer must not be something the
         * log can be configured to hide - or throttled into the gap between
         * two screenfuls. After that the normal trace rules apply. */
        {
            static uint8_t s_firstRx;
            if (s_firstRx < 5u)
            {
                s_firstRx++;
                Node1_LogLine("N1 GOT ", CAN012_ID_RX, rx, "  <<< RX WORKS");
            }
#if NODE1_TRACE_FRAMES
            else
            {
                static uint32_t s_traceTickRx;
                if (Node1_TraceGate(&s_traceTickRx))
                {
                    Node1_LogLine("N1 GOT ", CAN012_ID_RX, rx, NULL);
                }
            }
#endif
        }
    }

#if NODE1_DIAG
    Node1_Stats();
#endif

#if NODE1_USE_LCD
    /* Draws at most one rectangle's worth of SPI traffic per call, so a
     * redraw in progress can never eat more than that from the next
     * send-on-time budget. Costs ~nothing when the log is not dirty. */
    Lcd_Periodic();
#endif
}
