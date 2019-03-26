// Simple example to embed pictures in your sketch
// and draw on the ILI9341 display with writeRect()
//
// By Frank Bösing
//
// https://forum.pjrc.com/threads/32601-SPI-Library-Issue-w-ILI9341-TFT-amp-PN532-NFC-Module-on-Teensy-3-2?p=94534&viewfull=1#post94534

#include "SPI.h"
#include <ILI9341_t3n.h>
#include <SPIN.h>

// Converted to code with:
// http://www.rinkydinkelectronics.com/t_imageconverter565.php
//
#include "picture.c" //the picture
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
// On Teensy 4 beta with Paul's breakout out: 
// Using pins (MOSI, MISO, SCK which are labeled on Audio board breakout location 
// which are not in the Normal processor positions
// Also DC=10(CS), CS=9(MEMCS) and RST 23(MCLK)
#define TFT_RST 23
#define TFT_DC 10
#define TFT_CS 9
#else

// Normal Connections
#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#endif

#define TFT_MOSI    11
#define TFT_SCLK    13
#define TFT_MISO    12

// Alternate Connections with Teensy Audio Shield
//#define TFT_DC      20
//#define TFT_CS      21
//#define TFT_RST    255  // 255 = unused, connect to 3.3V
//#define TFT_MOSI     7
//#define TFT_SCLK    14
//#define TFT_MISO    12

ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.writeRect(32, 33, 256, 174, (uint16_t*)picture);
}

void loop(void) {
}
