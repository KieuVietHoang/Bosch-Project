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

/* How often the screen is actually repainted, in milliseconds.
 *
 * Not a cosmetic preference. Frames arrive roughly every 20 ms (0x0A2 at
 * 20 ms plus 0x012 at 50 ms), while repainting all eight rows costs about
 * 25 ms of background SPI time. Repainting on every arrival would restart
 * the pass before it could finish, and the rows it never reached would be
 * the newest ones - the display would end up showing mostly stale data.
 *
 * At 250 ms the repaint has ten times the room it needs, and four updates
 * a second is already faster than anyone can read. Every frame is still
 * recorded in the log regardless; this only governs how often what is
 * recorded gets drawn. */
#define LCD_REFRESH_MS 250u

/* Call for every frame that crosses the bus, in either direction - both
 * the 0x012 this board sends and the 0x0A2 it receives. Each call becomes
 * one row of the on-screen log:
 *
 *     CAn2: 2A 0D 00 03      <- oldest still on screen
 *     CAn1: 2A 0D 37 B4
 *     CAn2: 2B 0F 00 04
 *     ...                    <- newest at the bottom
 *
 * 'fromNode2' picks the label and its colour: 1 for a 0x0A2 from Node 2
 * (yellow), 0 for a 0x012 this board sent (cyan). The four data columns
 * are frame bytes 0, 1, 2 and 6 - Value0, Value1, then the sum and
 * checksum for 0x012, or 00 and the message counter for 0x0A2.
 *
 * Cheap: updates a RAM array only, no SPI here. The drawing happens
 * incrementally from Lcd_Periodic(), throttled to LCD_REFRESH_MS. */
void Lcd_ShowFrame(uint8_t fromNode2, const uint8_t frame[8]);

/* Call every main-loop iteration (unconditionally). Advances the redraw
 * state machine by at most one DMA burst and returns immediately - a few
 * microseconds of CPU, never a wait on the SPI bus, so it cannot eat into
 * the CAN send's +/-1 ms budget. Does nothing at all once the display has
 * caught up with the RAM log. */
void Lcd_Periodic(void);


#endif
