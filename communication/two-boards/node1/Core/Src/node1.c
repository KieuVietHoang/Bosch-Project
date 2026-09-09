/*********************************************************/
/*****       Node 1 application logic (this board)    *****/
/*********************************************************/

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
/* Diagnostics: with no debugger these counters are the only way to see
 * whether frames actually leave the mailbox and whether the RX interrupt
 * ever runs. */
static volatile uint16_t s_isrCount;      /* CAN1 RX0 interrupts taken */
static volatile uint16_t s_getErr;        /* HAL_CAN_GetRxMessage failures */
static volatile uint32_t s_lastRxId;      /* ID of the last frame received */
static uint16_t s_txErr;                  /* HAL_CAN_AddTxMessage failures */
#endif

#if NODE1_SELFTEST == 1
static uint8_t s_lbFrame[8];              /* our own 0x012, looped back by CAN1 */
static volatile uint8_t s_lbPending;
static uint32_t s_nextFeedTick;
static uint8_t  s_feedCounter;
static uint16_t s_lbPass, s_lbFail;
#endif

/* s_value0/1, s_rxFrame and s_lbFrame are written from the CAN1 RX ISR, so
 * main-loop reads take a snapshot with interrupts briefly masked. Without
 * this a frame arriving mid-read could pair Value0 from one 0x0A2 with
 * Value1 from the next, and the 0x012 we send would match neither. */
#define NODE1_ENTER_CRITICAL()  uint32_t prim_ = __get_PRIMASK(); __disable_irq()
#define NODE1_EXIT_CRITICAL()   __set_PRIMASK(prim_)

static void Node1_LogText(const char *text)
{
    UartLog_Write(text, (uint16_t)strlen(text));
}

/* Only needed for the per-frame trace and for loopback failure reports;
 * in the other configurations nothing calls it. */
#if NODE1_TRACE_FRAMES || (NODE1_SELFTEST == 1)
/* 'tag' names the sender and receiver explicitly (e.g. "N1->N2") so the
 * line is self-explanatory without a legend. 'suffix' is appended after
 * the 8 data bytes, or NULL for none. */
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
 * human-readable printout is slowed down). Each call site keeps its own
 * 'tick' so the kinds of line are throttled independently. Only the trace
 * lines use this; the loopback FAIL report is never throttled. */
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
#endif /* NODE1_TRACE_FRAMES - Node1_TraceGate */
#endif /* Node1_LogLine */

#if NODE1_DIAG
/* Borrows CAN1_RX (PA11) as a plain GPIO for a moment to find out what is
 * actually driving it. The CAN cell only leaves initialisation mode after
 * 11 consecutive recessive (high) bits, so a line held low blocks it
 * forever - and nothing else in the firmware misbehaves, which makes it
 * very hard to spot.
 *
 * Reading it twice, once with the internal pull-up and once with the
 * pull-down, separates the three cases: an outside driver wins both times,
 * a floating pin follows whichever resistor is enabled.
 *
 * Prime suspect on this board: PA11/PA12 are also USB_OTG_FS D-/D+, and
 * the Core405R wires them straight to the mini-USB socket. A cable to a PC
 * puts the host's 15k pull-downs on them, which beats the MCU's ~40k
 * internal pull-up. */
static void Node1_ProbeCanRxPin(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = GPIO_PIN_11;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);
    HAL_Delay(2);
    uint8_t withPu = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) ? 1u : 0u;

    gi.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &gi);
    HAL_Delay(2);
    uint8_t withPd = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) ? 1u : 0u;

    const char *verdict;
    if ((withPu == 0u) && (withPd == 0u))
    {
        verdict = "HELD LOW (dominant) << this is what blocks CAN";
    }
    else if ((withPu == 1u) && (withPd == 1u))
    {
        verdict = "driven high (recessive) - fine";
    }
    else
    {
        verdict = "floating - nothing driving it";
    }

    char line[112];
    int  n = snprintf(line, sizeof(line), "probe: PA11/CAN1_RX pu=%u pd=%u  %s\r\n",
                      withPu, withPd, verdict);
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }

    /* Hand the pin back to the CAN peripheral, pull-up enabled. */
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &gi);
}
#endif

void Node1_Init(void)
{
    s_value0    = 0u;
    s_value1    = 0u;
    s_rxPending = 0u;
    s_sendDue   = 0u;
    memset(s_rxFrame, 0x00, sizeof(s_rxFrame));

#if NODE1_USE_LCD
    Lcd_Init();
#endif

    /* FW#10 - bump this on every build handed over, so the terminal makes it
     * obvious whether the board is really running the newest hex. */
    Node1_LogText("\r\n=== BEA Node1 FW#10 : CAN1 500k, TX 0x012 / 50 ms ===\r\n");
#if NODE1_SELFTEST == 1
    Node1_LogText("!!! SELF-TEST 1: CAN1 IS IN LOOPBACK - NOTHING REACHES THE REAL BUS !!!\r\n"
                  "!!! set NODE1_SELFTEST to 0 in node1.h before the demo            !!!\r\n");
    s_lbPending    = 0u;
    s_feedCounter  = 0u;
    s_lbPass       = 0u;
    s_lbFail       = 0u;
    s_nextFeedTick = HAL_GetTick();
    memset(s_lbFrame, 0x00, sizeof(s_lbFrame));
#endif
#if NODE1_DIAG
    /* Did the peripheral actually start? HAL state 2 = LISTENING (started),
     * 1 = READY (init'd but Start never succeeded), 5 = ERROR. MSR bit0 INAK
     * must be 0 - if it is 1 the cell is still in initialisation mode and
     * every transmission will fail. */
    {
        /* MCR bit0 = INRQ (software's request to be in init mode),
         * MSR bit0 = INAK (hardware's acknowledge). The pair tells the two
         * failure modes apart:
         *   INRQ=1            -> HAL_CAN_Start() never even cleared the
         *                        request, i.e. it bailed out early
         *   INRQ=0 && INAK=1  -> we asked to leave init and the cell refuses,
         *                        i.e. it cannot see an idle bus */
        uint32_t mcr = hcan1.Instance->MCR;
        uint32_t msr = hcan1.Instance->MSR;
        const char *verdict =
            ((msr & 0x1u) == 0u)  ? "started OK" :
            ((mcr & 0x1u) != 0u)  ? "<< STUCK: INRQ still set, Start bailed out" :
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

        if ((msr & 0x1u) != 0u)
        {
            /* Stuck. Find out what is holding CAN1_RX down, then give the
             * peripheral one more chance now the pin has a pull-up on it. */
            Node1_ProbeCanRxPin();

            if (HAL_CAN_Init(&hcan1) == HAL_OK)
            {
                MX_CAN1_Setup();
            }
            n = snprintf(line, sizeof(line),
                         "retry: CAN1 start=%u state=%u MSR=%08lX %s\r\n",
                         (unsigned)((g_Can1SetupStatus >> 2) & 0x3u),
                         (unsigned)hcan1.State,
                         (unsigned long)hcan1.Instance->MSR,
                         ((hcan1.Instance->MSR & 0x1u) == 0u) ? "recovered" : "still stuck");
            if (n > 0)
            {
                UartLog_Write(line, (uint16_t)n);
            }
        }
    }

    s_lastTxTick = HAL_GetTick();
    s_statTick   = HAL_GetTick() + 2000u;
    s_txCount    = 0u;
    s_rxCount    = 0u;
    s_dtMin      = 0xFFFFFFFFu;
    s_dtMax      = 0u;
#endif
}

void Node1_HandleCanIrq(void)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];

    HAL_CAN_IRQHandler(&hcan1);
#if NODE1_DIAG
    s_isrCount++;
#endif

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
    Node1_OnCanRx(header.StdId, data);
}

void Node1_OnCanRx(uint32_t stdId, const uint8_t data[8])
{
    if (stdId == CAN012_ID_RX)
    {
        memcpy(s_rxFrame, data, sizeof(s_rxFrame));
        s_value0    = data[0];
        s_value1    = data[1];
        s_rxPending = 1u;
    }
#if NODE1_SELFTEST == 1
    else if (stdId == CAN012_ID_TX)
    {
        /* Only reachable in loopback: this is the frame we just sent. */
        memcpy(s_lbFrame, data, sizeof(s_lbFrame));
        s_lbPending = 1u;
    }
#endif
    else
    {
        /* not ours */
    }
}

void Node1_OnTimerTick(void)
{
    s_sendDue = 1u;
}

#if NODE1_SELFTEST == 1
/* Stands in for the Verification Board: one 0x0A2 every 20 ms, V0/V1
 * changing and a message counter cycling 0..F in byte 6, exactly as the
 * spec describes Node 2. Injected straight into the RX path so everything
 * downstream (LCD log, UART trace, the next 0x012) runs unmodified. */
static void Node1_SelfTestFeed(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_nextFeedTick) < 0)
    {
        return;
    }
    s_nextFeedTick = now + 20u;

    uint8_t fake[8] = {0};
    fake[0] = (uint8_t)(0x2Au + s_feedCounter);
    fake[1] = (uint8_t)(0x0Du + 2u * s_feedCounter);
    fake[6] = s_feedCounter;
    s_feedCounter = (uint8_t)((s_feedCounter + 1u) & 0x0Fu);

    Node1_OnCanRx(CAN012_ID_RX, fake);
}

/* Re-checks the 0x012 that CAN1 actually put on the wire, byte for byte,
 * against everything the assessment scores about it. */
static void Node1_SelfTestCheckLoopback(void)
{
    uint8_t f[8];

    NODE1_ENTER_CRITICAL();
    memcpy(f, s_lbFrame, sizeof(f));
    s_lbPending = 0u;
    NODE1_EXIT_CRITICAL();

    uint8_t okSum = (f[2] == (uint8_t)(f[0] + f[1]));
    uint8_t okCrc = (f[6] == Can012_Checksum(f, 6u));
    uint8_t okPad = ((f[3] | f[4] | f[5] | f[7]) == 0u);

    if (okSum && okCrc && okPad)
    {
        s_lbPass++;
#if NODE1_TRACE_FRAMES
        static uint32_t s_traceTickLb;
        if (Node1_TraceGate(&s_traceTickLb))
        {
            Node1_LogLine("N1 self", CAN012_ID_TX, f, "  PASS (looped back)");
        }
#endif
    }
    else
    {
        s_lbFail++;
        /* Failures always print, whatever the trace setting. */
        Node1_LogLine("LB", CAN012_ID_TX, f,
                      (!okCrc) ? "  FAIL crc" : ((!okSum) ? "  FAIL sum" : "  FAIL pad"));
    }
}
#endif

#if NODE1_DIAG
void Node1_FormatCanEsr(uint32_t esr, char *buf, uint16_t len)
{
    /* CAN_ESR: bit0 EWGF, bit1 EPVF, bit2 BOFF, bits6:4 LEC,
     * bits23:16 TEC, bits31:24 REC. */
    static const char *lecName[8] =
    {
        "ok", "stuff", "form", "ACK", "bit1", "bit0", "crc", "sw"
    };
    (void)snprintf(buf, len, "LEC=%s TEC=%u REC=%u%s%s%s",
                   lecName[(esr >> 4) & 0x7u],
                   (unsigned)((esr >> 16) & 0xFFu),
                   (unsigned)((esr >> 24) & 0xFFu),
                   (esr & 0x1u) ? " WARN" : "",
                   (esr & 0x2u) ? " PASSIVE" : "",
                   (esr & 0x4u) ? " BUSOFF" : "");
}

/* One compact line every 2 s - the quickest way to answer "is the
 * transmission correct": frame counts, the min/max send gap against the
 * 50 ms +/-1 ms requirement, and CAN1's error state. */
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

    char line[192];
    int  n = snprintf(line, sizeof(line),
#if NODE1_SELFTEST == 1
                      "[2s] TX=%u RX=%u  dt=%lu..%lu ms  LB pass=%u fail=%u  CAN1 %s\r\n",
#else
                      "[2s] TX=%u RX=%u  dt=%lu..%lu ms  CAN1 %s\r\n",
#endif
                      (unsigned)s_txCount, (unsigned)s_rxCount,
                      (unsigned long)((s_dtMin == 0xFFFFFFFFu) ? 0u : s_dtMin),
                      (unsigned long)s_dtMax,
#if NODE1_SELFTEST == 1
                      (unsigned)s_lbPass, (unsigned)s_lbFail,
#endif
                      esr);
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }

    /* Raw hardware view. TSR bit2/bit10/bit18 = TXOK for mailbox 0/1/2,
     * bits 26..28 = mailbox empty; RF0R low 2 bits = frames waiting in
     * FIFO0; MSR bit0 = INAK (still in init), bit1 = SLAK (asleep). */
    n = snprintf(line, sizeof(line),
                 "     dbg isr=%u getErr=%u lastId=%03lX txErr=%u  MCR=%08lX MSR=%08lX TSR=%08lX RF0R=%08lX BTR=%08lX\r\n",
                 (unsigned)s_isrCount, (unsigned)s_getErr,
                 (unsigned long)s_lastRxId, (unsigned)s_txErr,
                 (unsigned long)hcan1.Instance->MCR,
                 (unsigned long)hcan1.Instance->MSR,
                 (unsigned long)hcan1.Instance->TSR,
                 (unsigned long)hcan1.Instance->RF0R,
                 (unsigned long)hcan1.Instance->BTR);
    if (n > 0)
    {
        UartLog_Write(line, (uint16_t)n);
    }

    s_txCount  = 0u;
    s_rxCount  = 0u;
    s_isrCount = 0u;
    s_getErr   = 0u;
    s_txErr    = 0u;
    s_dtMin    = 0xFFFFFFFFu;
    s_dtMax    = 0u;
#if NODE1_SELFTEST == 1
    s_lbPass = 0u;
    s_lbFail = 0u;
#endif
}
#endif

/* CAN1 can fail to start when the bus is not idle at power-up - the cell
 * needs 11 recessive bits before it will leave initialisation mode, and
 * HAL_CAN_Start() gives up after 10 ms. The hardware recovers on its own the
 * moment the bus goes quiet, but the HAL handle stays in ERROR and every
 * transmission keeps failing, so nothing works again until a reset.
 *
 * This retries twice a second while the peripheral is broken and costs a
 * single comparison once it is running. It also covers the real demo: if the
 * Verification Board is powered up after this one, CAN1 recovers by itself
 * instead of needing a manual reset. */
static void Node1_RecoverCan(void)
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

void Node1_Periodic(void)
{
    Node1_RecoverCan();

    /* Timing-critical: load the TX mailbox first, before any blocking
     * UART/LCD work below, so the +-1ms send-on-time budget is never
     * eaten by a slow debug print or screen redraw from this same call. */
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
#else
        (void)HAL_CAN_AddTxMessage(&hcan1, &txHeader, txFrame, &txMailbox);
#endif

#if NODE1_DIAG
        uint32_t now = HAL_GetTick();
        uint32_t dt  = now - s_lastTxTick;
        s_lastTxTick = now;
        if (dt < s_dtMin) { s_dtMin = dt; }
        if (dt > s_dtMax) { s_dtMax = dt; }
        s_txCount++;
#if NODE1_TRACE_FRAMES
        static uint32_t s_traceTickTx;
        if (Node1_TraceGate(&s_traceTickTx))
        {
            char gap[16];
            (void)snprintf(gap, sizeof(gap), "  dt=%lums", (unsigned long)dt);
            Node1_LogLine("N1->N2", CAN012_ID_TX, txFrame, gap);
        }
#endif
#elif NODE1_TRACE_FRAMES
        {
            static uint32_t s_traceTickTx;
            if (Node1_TraceGate(&s_traceTickTx))
            {
                Node1_LogLine("N1->N2", CAN012_ID_TX, txFrame, NULL);
            }
        }
#endif
    }

#if NODE1_SELFTEST == 1
    Node1_SelfTestFeed();
    if (s_lbPending)
    {
        Node1_SelfTestCheckLoopback();
    }
#endif

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
        Lcd_ShowValues(rx[0], rx[1]); /* cheap: just queues a log row, no SPI here */
#endif
#if NODE1_TRACE_FRAMES
        static uint32_t s_traceTickRx;
        if (Node1_TraceGate(&s_traceTickRx))
        {
            Node1_LogLine("N2->N1", CAN012_ID_RX, rx, NULL);
        }
#else
        (void)rx;
#endif
    }

#if NODE1_DIAG
    Node1_Stats();
#endif

#if NODE1_USE_LCD
    /* Draws at most one log row's worth of SPI traffic per call (~0.4 ms),
     * so a redraw in progress can never eat more than that from the next
     * send-on-time budget. Costs ~nothing when the log isn't dirty. */
    Lcd_Periodic();
#endif
}
