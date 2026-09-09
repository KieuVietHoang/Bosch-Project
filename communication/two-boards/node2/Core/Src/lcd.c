/*********************************************************/
/*****            LCD display - ST7789 over SPI1       *****/
/*********************************************************/
/* See lcd.h for the pin mapping and how confident each pin is.
 *
 * Drawing is DMA-driven and never blocks. Lcd_ShowValues() only touches a
 * RAM log; Lcd_Periodic() advances a small state machine that starts at
 * most one DMA burst per call and returns immediately, so no call costs
 * more than a few microseconds of CPU. That matters because this runs in
 * the same main loop as the 50 ms CAN send: the old blocking version spent
 * ~0.4 ms per call inside HAL_SPI_Transmit(), which eats into the +/-1 ms
 * send-on-time budget - the same trap uart_log.c was written to escape.
 *
 * Only the one-off full-screen clear in Lcd_Init() still blocks; it runs
 * before the periodic timer is started, so it cannot delay anything.
 */

#include "lcd.h"
#include "main.h"
#if LCD_TEST_PINSCAN
#include <stdio.h>
#endif

#define LCD_WIDTH   240u
#define LCD_HEIGHT  320u

#define LCD_CS_PORT   GPIOB
#define LCD_CS_PIN    GPIO_PIN_7
#define LCD_DC_PORT   GPIOB
#define LCD_DC_PIN    GPIO_PIN_8
#define LCD_BL_PORT   GPIOB
#define LCD_BL_PIN    GPIO_PIN_6

/* PB9 is the TOUCH controller's chip select (TP_CS), not the LCD reset -
 * confirmed against the board schematic. The XPT2046 shares this SPI bus,
 * so it is parked HIGH (deselected) and never touched again; leaving it
 * low or floating would let the touch chip fight the panel on the bus.
 * The panel's own RST line is wired to the board RESET signal, so it is
 * not software-controllable at all - the SWRESET command in
 * Lcd_PanelInit() is what resets the controller. */
#define LCD_TPCS_PORT GPIOB
#define LCD_TPCS_PIN  GPIO_PIN_9

#define LCD_CS_LOW()   HAL_GPIO_WritePin(LCD_CS_PORT,  LCD_CS_PIN,  GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(LCD_CS_PORT,  LCD_CS_PIN,  GPIO_PIN_SET)
#define LCD_DC_CMD()   HAL_GPIO_WritePin(LCD_DC_PORT,  LCD_DC_PIN,  GPIO_PIN_RESET)
#define LCD_DC_DATA()  HAL_GPIO_WritePin(LCD_DC_PORT,  LCD_DC_PIN,  GPIO_PIN_SET)

#define COLOR_BLACK   0x0000u
#define COLOR_WHITE   0xFFFFu
#define COLOR_CYAN    0x07FFu
#define COLOR_YELLOW  0xFFE0u
#define COLOR_GRAY    0x2104u

#define LOG_ROWS      8u
#define LOG_ROW_H     (LCD_HEIGHT / LOG_ROWS)   /* 320/8 = 40, divides evenly */

/* Layout of one log row: 4 seven-segment digits (V0 in cyan, V1 in yellow)
 * plus a divider line underneath. */
#define DIG_W         26u
#define DIG_H         28u
#define DIG_THICK     5u
#define DIG_GAP       4u
#define DIG_GROUP_GAP 16u
#define DIG_X0        20u
#define SEGS_PER_DIGIT 7u
#define DIGITS_PER_ROW 4u
#define RECTS_PER_ROW  (DIGITS_PER_ROW * SEGS_PER_DIGIT + 1u)  /* +1 divider */

/* One DMA burst paints this many pixels. Bigger = fewer interrupts, more
 * RAM; 256 px is ~390 us of SPI time in the background per burst. */
#define LCD_CHUNK_PX  256u

static SPI_HandleTypeDef hspi1;
static DMA_HandleTypeDef s_hdmaSpi1Tx;

/* Source for the fills: the current colour repeated. Memory-increment is
 * left on and the buffer pre-filled, rather than pointing DMA at a single
 * word with increment off - that would need 16-bit SPI frames, and the
 * command path wants 8-bit. */
static uint8_t s_chunk[LCD_CHUNK_PX * 2u];
static volatile uint8_t s_dmaBusy;

/* Standard segment-to-hex-digit table, bit0=a(top) .. bit6=g(middle). */
static const uint8_t SEG7[16] =
{
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
    0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71
};

typedef struct
{
    uint8_t v0;
    uint8_t v1;
} LogEntry;

/* Newest entry is always s_log[LOG_ROWS-1]; index 0 is the oldest still on
 * screen. Pure RAM state - Lcd_ShowValues() only touches this, never SPI. */
static LogEntry s_log[LOG_ROWS];

/* Redraw state machine. s_rowsPending rows starting at s_rowCursor still
 * need repainting; within a row, s_rectIdx is the rectangle being painted
 * and s_pxRemain how much of it is left. */
static uint8_t  s_rowsPending;
static uint8_t  s_rowCursor;
static uint8_t  s_rectIdx;
static uint32_t s_pxRemain;
static uint8_t  s_rectOpen;      /* CS is low, a rect is mid-transfer */

/* HAL calls this from HAL_SPI_Init(). Only SPI1 is used on this board. */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF5_SPI1;

    gi.Pin = GPIO_PIN_7;           /* PA7 = SPI1_MOSI */
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin = GPIO_PIN_3;           /* PB3 = SPI1_SCK (remapped off PA5) */
    HAL_GPIO_Init(GPIOB, &gi);

    /* MISO not wired - the panel is write-only. */

    /* SPI1_TX is DMA2 Stream3 Channel3 (RM0090 Table 43). DMA1 Stream3 is
     * the UART log's, a different controller - no conflict. */
    __HAL_RCC_DMA2_CLK_ENABLE();
    s_hdmaSpi1Tx.Instance                 = DMA2_Stream3;
    s_hdmaSpi1Tx.Init.Channel             = DMA_CHANNEL_3;
    s_hdmaSpi1Tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdmaSpi1Tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdmaSpi1Tx.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdmaSpi1Tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdmaSpi1Tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_hdmaSpi1Tx.Init.Mode                = DMA_NORMAL;
    s_hdmaSpi1Tx.Init.Priority            = DMA_PRIORITY_LOW;
    s_hdmaSpi1Tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdmaSpi1Tx) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(hspi, hdmatx, s_hdmaSpi1Tx);

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

void Lcd_HandleDmaIrq(void)
{
    HAL_DMA_IRQHandler(&s_hdmaSpi1Tx);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        s_dmaBusy = 0u;
    }
}

static void Lcd_GpioInit(void)
{
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Pin   = LCD_CS_PIN | LCD_DC_PIN | LCD_BL_PIN | LCD_TPCS_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gi);

    LCD_CS_HIGH();
    HAL_GPIO_WritePin(LCD_TPCS_PORT, LCD_TPCS_PIN, GPIO_PIN_SET);  /* touch chip deselected, for good */
    HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);      /* backlight on */
}

static void Lcd_SpiInit(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    /* SPI1 is on APB2 = 84 MHz. /8 = 10.5 MHz - conservative for a first
     * bring-up; ST7789 is rated well beyond this, raise it later if needed. */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

/* Commands and their few argument bytes stay blocking: each is 1-4 bytes,
 * a couple of microseconds, far below anything that could disturb timing. */
static void Lcd_WriteCmd(uint8_t cmd)
{
    LCD_DC_CMD();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    LCD_CS_HIGH();
}

static void Lcd_WriteData(const uint8_t *data, uint16_t len)
{
    LCD_DC_DATA();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
    LCD_CS_HIGH();
}

static void Lcd_WriteData1(uint8_t d)
{
    Lcd_WriteData(&d, 1u);
}

/* Leaves the panel pointed at the given window with RAMWR issued, ready
 * for pixel bytes. */
static void Lcd_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(x0 >> 8); buf[1] = (uint8_t)x0;
    buf[2] = (uint8_t)(x1 >> 8); buf[3] = (uint8_t)x1;
    Lcd_WriteCmd(0x2Au);
    Lcd_WriteData(buf, 4u);

    buf[0] = (uint8_t)(y0 >> 8); buf[1] = (uint8_t)y0;
    buf[2] = (uint8_t)(y1 >> 8); buf[3] = (uint8_t)y1;
    Lcd_WriteCmd(0x2Bu);
    Lcd_WriteData(buf, 4u);

    Lcd_WriteCmd(0x2Cu); /* RAMWR */
}

static void Lcd_FillChunkBuffer(uint16_t color)
{
    for (uint16_t i = 0u; i < LCD_CHUNK_PX; i++)
    {
        s_chunk[2u * i]      = (uint8_t)(color >> 8);
        s_chunk[2u * i + 1u] = (uint8_t)color;
    }
}

/* Blocking fill - used only by Lcd_Init(), before the periodic timer runs. */
static void Lcd_FillRectBlocking(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0u) || (h == 0u))
    {
        return;
    }
    Lcd_FillChunkBuffer(color);
    Lcd_SetAddrWindow(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));

    uint32_t total = (uint32_t)w * (uint32_t)h;
    LCD_DC_DATA();
    LCD_CS_LOW();
    while (total > 0u)
    {
        uint32_t n = (total > LCD_CHUNK_PX) ? LCD_CHUNK_PX : total;
        HAL_SPI_Transmit(&hspi1, s_chunk, (uint16_t)(n * 2u), HAL_MAX_DELAY);
        total -= n;
    }
    LCD_CS_HIGH();
}

#if LCD_TEST_PATTERN
/* Bring-up aid, not part of normal operation. Paints the whole screen a
 * few solid colours, then leaves four static bars, before the log takes
 * over. It splits the problem in two:
 *   - colours appear -> panel, SPI, DMA, the init command sequence and the
 *     backlight are all fine, and any remaining fault is in the drawing
 *     logic above this layer;
 *   - screen stays dark -> the fault is below that: backlight, wiring,
 *     seating, or the panel is the HX8347D variant rather than ST7789.
 * Runs before the periodic timer is started, so the blocking fills and
 * delays here cannot disturb anything. */
static void Lcd_TestPattern(void)
{
    static const uint16_t seq[4] = { 0xF800u, 0x07E0u, 0x001Fu, 0xFFFFu }; /* R G B W */

    for (uint8_t i = 0u; i < 4u; i++)
    {
        Lcd_FillRectBlocking(0u, 0u, LCD_WIDTH, LCD_HEIGHT, seq[i]);
        HAL_Delay(600);
    }

    /* Four bars left on screen: a single photo then shows colour order and
     * orientation at once. */
    Lcd_FillRectBlocking(0u,   0u, LCD_WIDTH, 80u, 0xF800u);  /* red    */
    Lcd_FillRectBlocking(0u,  80u, LCD_WIDTH, 80u, 0x07E0u);  /* green  */
    Lcd_FillRectBlocking(0u, 160u, LCD_WIDTH, 80u, 0x001Fu);  /* blue   */
    Lcd_FillRectBlocking(0u, 240u, LCD_WIDTH, 80u, 0xFFFFu);  /* white  */
    HAL_Delay(3000);
}
#endif

static void Lcd_PanelInit(void)
{
    Lcd_WriteCmd(0x01u); /* SWRESET */
    HAL_Delay(150);
    Lcd_WriteCmd(0x11u); /* SLPOUT */
    HAL_Delay(120);

    Lcd_WriteCmd(0x3Au); /* COLMOD */
    Lcd_WriteData1(0x55u); /* 16 bpp RGB565 */

    Lcd_WriteCmd(0x36u); /* MADCTL - default orientation; rotate here if the
                           * image comes up sideways/mirrored on real hardware */
    Lcd_WriteData1(0x00u);

    Lcd_WriteCmd(0x21u); /* INVON - most ST7789 IPS panels need this for correct colours */
    Lcd_WriteCmd(0x13u); /* NORON */
    HAL_Delay(10);
    Lcd_WriteCmd(0x29u); /* DISPON */
    HAL_Delay(100);
}

/* Geometry of one rectangle of a log row, computed rather than stored:
 * index 0..27 are the 7 segments of each of the 4 digits, index 28 is the
 * divider. Returns 0 for indices that draw nothing (the last row has no
 * divider). Keeping this stateless is what lets the redraw be resumed one
 * rectangle at a time without a queue. */
static uint8_t Lcd_RowRect(uint8_t row, uint8_t idx,
                           uint16_t *x, uint16_t *y, uint16_t *w, uint16_t *h,
                           uint16_t *color)
{
    if (idx == (RECTS_PER_ROW - 1u))
    {
        if (row >= (LOG_ROWS - 1u))
        {
            return 0u;      /* no divider under the bottom row */
        }
        *x = 0u;
        *y = (uint16_t)((row + 1u) * LOG_ROW_H - 1u);
        *w = LCD_WIDTH;
        *h = 1u;
        *color = COLOR_GRAY;
        return 1u;
    }

    uint8_t digit = (uint8_t)(idx / SEGS_PER_DIGIT);
    uint8_t seg   = (uint8_t)(idx % SEGS_PER_DIGIT);

    /* digits 0,1 show V0 (cyan); digits 2,3 show V1 (yellow) */
    uint8_t  val = (digit < 2u) ? s_log[row].v0 : s_log[row].v1;
    uint8_t  nib = ((digit & 1u) == 0u) ? (uint8_t)((val >> 4) & 0x0Fu)
                                        : (uint8_t)(val & 0x0Fu);
    uint16_t fg  = (digit < 2u) ? COLOR_CYAN : COLOR_YELLOW;
    uint8_t  mask = SEG7[nib];

    uint16_t dx = DIG_X0 + (uint16_t)digit * (DIG_W + DIG_GAP);
    if (digit >= 2u)
    {
        dx += DIG_GROUP_GAP;   /* extra space between the V0 and V1 pairs */
    }
    uint16_t dy  = (uint16_t)(row * LOG_ROW_H + (LOG_ROW_H - DIG_H) / 2u);
    uint16_t t   = DIG_THICK;
    uint16_t mid = DIG_H / 2u;

    /* Segment order and bit mapping match SEG7[]: bit0=a .. bit6=g. Every
     * segment is painted either foreground or background, so an old digit
     * never needs clearing first. */
    static const uint8_t segBit[SEGS_PER_DIGIT] = { 0x01u, 0x40u, 0x08u, 0x20u, 0x02u, 0x10u, 0x04u };
    switch (seg)
    {
        case 0: *x = dx + t;         *y = dy;                  *w = DIG_W - 2u * t; *h = t; break;               /* a */
        case 1: *x = dx + t;         *y = dy + mid - t / 2u;   *w = DIG_W - 2u * t; *h = t; break;               /* g */
        case 2: *x = dx + t;         *y = dy + DIG_H - t;      *w = DIG_W - 2u * t; *h = t; break;               /* d */
        case 3: *x = dx;             *y = dy + t;              *w = t; *h = mid - t; break;                      /* f */
        case 4: *x = dx + DIG_W - t; *y = dy + t;              *w = t; *h = mid - t; break;                      /* b */
        case 5: *x = dx;             *y = dy + mid + t / 2u;   *w = t; *h = mid - t - t / 2u; break;             /* e */
        default:*x = dx + DIG_W - t; *y = dy + mid + t / 2u;   *w = t; *h = mid - t - t / 2u; break;             /* c */
    }
    *color = ((mask & segBit[seg]) != 0u) ? fg : COLOR_BLACK;
    return 1u;
}

/* Starts one DMA burst for the rectangle currently open. */
static void Lcd_StartChunk(void)
{
    uint32_t n = (s_pxRemain > LCD_CHUNK_PX) ? LCD_CHUNK_PX : s_pxRemain;
    s_pxRemain -= n;
    s_dmaBusy = 1u;
    (void)HAL_SPI_Transmit_DMA(&hspi1, s_chunk, (uint16_t)(n * 2u));
}

#if LCD_TEST_BACKLIGHT
/* Backlight-only bring-up. Touches nothing but PB6 (LCD_BL) and blinks it
 * at 1 Hz forever - no SPI, no panel commands, no CAN. It answers one
 * question and nothing else: does this pin control the backlight?
 *
 *   backlight pulses  -> PB6 is right and the module has power; whatever
 *                        is wrong lies in the panel/SPI side
 *   nothing at all    -> the fault is below the driver entirely: module
 *                        seating, its 5V supply, or PB6 is not the
 *                        backlight on this particular board
 *
 * Deliberately never returns: during this test the board is a lamp, and
 * that keeps the result unambiguous. */
static void Lcd_TestBacklight(void)
{
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gi.Pin   = LCD_BL_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_BL_PORT, &gi);

    for (;;)
    {
        HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
}
#endif

#if LCD_TEST_PINSCAN
#include "uart_log.h"

/* Walks every pin of GPIOB then GPIOA, blinking each in turn and naming it
 * over UART first. Watch the screen and the terminal together: whichever
 * pin is being announced when the backlight starts pulsing is the one that
 * drives it. Blinks rather than just driving high, so an active-low
 * backlight reveals itself too.
 *
 * Each pin is returned to plain input afterwards, so only one is ever
 * driven at a time. Nothing here touches GPIOC, where the UART lives, so
 * the log that identifies the pin cannot break itself.
 *
 * If a whole sweep passes with the screen never reacting, the answer is
 * not "wrong pin" - it is that the module has no power or is not
 * connected, which is then a hardware matter, not a firmware one.
 * Never returns. */
static void Lcd_TestPinScan(void)
{
    /* GPIOC included because the 11-pin LCD header is a different connector
     * from the 40-pin LCD PORT the schematic sheet covers, so its routing
     * cannot be assumed to match. PC10/PC11 are skipped: that is the UART
     * carrying the very log used to read this scan's results. */
    GPIO_TypeDef * const ports[3] = { GPIOB, GPIOA, GPIOC };
    const char           names[3] = { 'B', 'A', 'C' };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    static const char banner[] = "\r\n=== LCD PIN SCAN: watch the screen, note the pin ===\r\n";
    UartLog_Write(banner, (uint16_t)(sizeof(banner) - 1u));

    for (;;)
    {
        for (uint8_t p = 0u; p < 3u; p++)
        {
            for (uint8_t bit = 0u; bit < 16u; bit++)
            {
                if ((ports[p] == GPIOC) && ((bit == 10u) || (bit == 11u)))
                {
                    continue;      /* USART3 - do not disturb the log */
                }

                char msg[24];
                int  n = snprintf(msg, sizeof(msg), "  now P%c%u\r\n",
                                  names[p], (unsigned)bit);
                if (n > 0)
                {
                    UartLog_Write(msg, (uint16_t)n);
                }

                GPIO_InitTypeDef gi = {0};
                gi.Pin   = (uint32_t)1u << bit;
                gi.Mode  = GPIO_MODE_OUTPUT_PP;
                gi.Pull  = GPIO_NOPULL;
                gi.Speed = GPIO_SPEED_FREQ_LOW;
                HAL_GPIO_Init(ports[p], &gi);

                for (uint8_t k = 0u; k < 4u; k++)
                {
                    HAL_GPIO_WritePin(ports[p], (uint16_t)gi.Pin, GPIO_PIN_SET);
                    HAL_Delay(250);
                    HAL_GPIO_WritePin(ports[p], (uint16_t)gi.Pin, GPIO_PIN_RESET);
                    HAL_Delay(250);
                }

                gi.Mode = GPIO_MODE_INPUT;   /* release it before the next one */
                HAL_GPIO_Init(ports[p], &gi);
            }
        }
    }
}
#endif

void Lcd_Init(void)
{
#if LCD_TEST_PINSCAN
    Lcd_TestPinScan();     /* never returns */
#endif
#if LCD_TEST_BACKLIGHT
    Lcd_TestBacklight();   /* never returns */
#endif
    Lcd_GpioInit();
    Lcd_SpiInit();
    /* No hardware reset pulse: the panel's RST is tied to the board RESET
     * line, so it has already been reset along with the MCU. SWRESET below
     * covers the controller state. */
    Lcd_PanelInit();

#if LCD_TEST_PATTERN
    Lcd_TestPattern();
#endif

    Lcd_FillRectBlocking(0u, 0u, LCD_WIDTH, LCD_HEIGHT, COLOR_BLACK);

    s_rowsPending = 0u;
    s_rowCursor   = 0u;
    s_rectIdx     = 0u;
    s_pxRemain    = 0u;
    s_rectOpen    = 0u;
    s_dmaBusy     = 0u;
}

void Lcd_ShowValues(uint8_t value0, uint8_t value1)
{
    for (uint8_t i = 0u; i < (LOG_ROWS - 1u); i++)
    {
        s_log[i] = s_log[i + 1u];
    }
    s_log[LOG_ROWS - 1u].v0 = value0;
    s_log[LOG_ROWS - 1u].v1 = value1;

    /* Every row's content shifted, so all of them need repainting. Restart
     * from the top; anything half-drawn is simply overwritten. */
    s_rowCursor   = 0u;
    s_rectIdx     = 0u;
    s_rowsPending = LOG_ROWS;
}

void Lcd_Periodic(void)
{
    if (s_dmaBusy)
    {
        return;                     /* burst still in flight - nothing to do */
    }

    /* Continue the rectangle already open. */
    if (s_pxRemain > 0u)
    {
        Lcd_StartChunk();
        return;
    }

    /* Rectangle finished: wait out the last byte still in the shift
     * register (a few hundred ns at 10.5 MHz), then release CS. */
    if (s_rectOpen)
    {
        while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET)
        {
        }
        LCD_CS_HIGH();
        s_rectOpen = 0u;
        s_rectIdx++;
    }

    if (s_rowsPending == 0u)
    {
        return;                     /* display is up to date */
    }

    if (s_rectIdx >= RECTS_PER_ROW)
    {
        s_rectIdx = 0u;
        s_rowCursor++;
        s_rowsPending--;
        if (s_rowsPending == 0u)
        {
            return;
        }
    }

    /* Open the next rectangle. */
    uint16_t x, y, w, h, color;
    if (!Lcd_RowRect(s_rowCursor, s_rectIdx, &x, &y, &w, &h, &color) ||
        (w == 0u) || (h == 0u))
    {
        s_rectIdx++;                /* nothing to draw here, try the next one */
        return;
    }

    Lcd_FillChunkBuffer(color);
    Lcd_SetAddrWindow(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));

    s_pxRemain = (uint32_t)w * (uint32_t)h;
    LCD_DC_DATA();
    LCD_CS_LOW();
    s_rectOpen = 1u;
    Lcd_StartChunk();
}
