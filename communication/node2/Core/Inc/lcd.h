/*********************************************************/
/*****            LCD display - thin interface        *****/
/*********************************************************/
/*
 * Physical module (identified from a board photo + the Waveshare wiki page
 * for the Open405R-C's companion display, "2.8inch Resistive Touch LCD"):
 *   - Panel controller : ST7789, 240x320, SPI, RGB565
 *   - Touch controller  : XPT2046 (resistive) - NOT wired up here, the
 *     Communication practice only needs to *show* V0/V1, not read touch.
 *
 * Interface  : SPI1, MOSI=PA7, SCK=PB3 (both AF5; this is the documented
 * "remap SCK off PA5" trick - MISO is not wired, the panel is write-only).
 *
 * Pin mapping, all read off the board schematic's LCD PORT sheet:
 *   LCD_BL  = PB6   (schematic name LCD_PWM / BLPWM)
 *   LCD_CS  = PB7
 *   LCD_DC  = PB8   (schematic name LCD-RS, "register select")
 *   TP_CS   = PB9   -- the TOUCH chip select, NOT the LCD reset
 *   TP_IRQ  = PB4   -- touch interrupt, unused
 *   LCD_RST         -- tied to the board RESET net; no GPIO controls it
 *
 * The PB9 line is worth spelling out: an earlier version of this driver
 * assumed it was LCD_RST and pulsed it low at startup. It is the XPT2046's
 * chip select, and the touch controller shares this SPI bus - so PB9 is
 * now driven HIGH once in Lcd_GpioInit() and left there, keeping the touch
 * chip deselected. A low or floating TP_CS would let it drive the bus
 * against the panel. There is likewise no reset pulse any more: the panel
 * resets with the board, and Lcd_PanelInit() issues SWRESET anyway.
 */

#ifndef _LCD_H
#define _LCD_H

#include <stdint.h>

/* 1 = paint a red/green/blue/white bring-up pattern at startup before the
 * log begins. Only for first power-on of a display; set back to 0 after. */
#define LCD_TEST_PATTERN 0

/* 1 = do nothing but blink the backlight pin at 1 Hz, forever. The
 * narrowest possible test of the display: no SPI, no panel, no CAN. */
#define LCD_TEST_BACKLIGHT 0

/* 1 = blink every GPIOB then GPIOA pin in turn, naming each over UART, to
 * find which one actually drives the backlight. Overrides the others. */
#define LCD_TEST_PINSCAN 0

/* Call once from main() after peripherals are up. Resets and initialises
 * the ST7789 panel, clears the screen. */
void Lcd_Init(void);

/* Call whenever a new 0x0A2 is received. Pushes (value0, value1) as the
 * newest row of an on-screen scrolling log (oldest row falls off the top).
 * Cheap - only updates a RAM array, does no SPI transfer itself; the
 * actual drawing happens incrementally from Lcd_Periodic(). */
void Lcd_ShowValues(uint8_t value0, uint8_t value1);

/* Call every main-loop iteration (unconditionally). Advances the redraw
 * state machine by at most one DMA burst and returns immediately - a few
 * microseconds of CPU, never a wait on the SPI bus, so it cannot eat into
 * the CAN send's +/-1 ms budget. Does nothing at all once the display has
 * caught up with the RAM log. */
void Lcd_Periodic(void);

/* Call from DMA2_Stream3_IRQHandler (the stream SPI1_TX uses). */
void Lcd_HandleDmaIrq(void);

#endif
