#include <ili9341_t3n_font_Arial.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <ILI9341_t3n.h>

#include <SPIN.h>
#include "SPI.h"
#define KURTS_FLEXI
#ifdef KURTS_FLEXI
#define TFT_DC 22
#define TFT_CS 15
#define TFT_RST -1
#define TFT_SCK 14
#define TFT_MISO 12
#define TFT_MOSI 7
#define DEBUG_PIN 13
#else
#define TFT_DC  9
#define TFT_CS 10
#define TFT_RST 7
#define TFT_SCK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#endif
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO, &SPIN);
Adafruit_GFX_Button button;
uint8_t use_fb = 0;
uint8_t use_clip_rect = 0;
uint8_t use_set_origin = 0;

#define ORIGIN_TEST_X 50
#define ORIGIN_TEST_Y 50

void setup() {
  while (!Serial && (millis() < 4000)) ;
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);
#ifdef DEBUG_PIN
  pinMode(DEBUG_PIN, OUTPUT);
#endif

  button.initButton(&tft, 200, 125, 100, 40, ILI9341_GREEN, ILI9341_YELLOW, ILI9341_RED, "UP", 1);

  drawTestScreen();
}

void SetupOrClearClipRectAndOffsets() {
  if (use_clip_rect) {
    tft.setClipRect();  // make sure we clear the whole screen
    tft.setOrigin();    // make sure none are set yet

    tft.fillScreen(ILI9341_LIGHTGREY);

    // Now lets set origin.
    if (use_set_origin)
      tft.setOrigin(ORIGIN_TEST_X, ORIGIN_TEST_Y);
    int x = tft.width() / 4;
    int y = tft.height() / 4;
    int w = tft.width() / 2;
    int h = tft.height() / 2;
    tft.drawRect(x, y, w, h, ILI9341_ORANGE);
    tft.updateScreen();
    tft.setClipRect(x + 1, y + 1, w - 2, h - 2);
    delay(250);

  } else {
    tft.setClipRect();
    if (use_set_origin)
      tft.setOrigin(ORIGIN_TEST_X, ORIGIN_TEST_Y);
    else
      tft.setOrigin();
  }
}


uint16_t palette[16];  // Should probably be 256, but I don't use many colors...
uint16_t pixel_data[2500];
const uint8_t pict1bpp[] = {0xff, 0xff, 0xc0, 0x03, 0xa0, 0x05, 0x90, 0x9, 0x88, 0x11, 0x84, 0x21, 0x82, 0x41, 0x81, 0x81,
                            0x81, 0x81, 0x82, 0x41, 0x84, 0x21, 0x88, 0x11, 0x90, 0x09, 0xa0, 0x05, 0xc0, 0x03, 0xff, 0xff
                           };
const uint8_t pict2bpp[] = {
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 
};
const uint8_t pict4bpp[] = {  
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,  
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,  
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,  
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,  
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


                           
void drawTestScreen() {
  Serial.printf("Use FB: %d ", use_fb); Serial.flush();
  tft.useFrameBuffer(use_fb);
  SetupOrClearClipRectAndOffsets();
  uint32_t start_time = millis();
  tft.fillScreen(use_fb ? ILI9341_RED : ILI9341_BLACK);
  //tft.setFont(Inconsolata_60);
  tft.setFont(Arial_24_Bold);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(0, 0);
  tft.println("Test");
  tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
  tft.println("text");
  tft.setCursor(85, 65);
  tft.print("XYZ");
  tft.setFontAdafruit();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("01234");
  tft.setTextColor(ILI9341_WHITE, ILI9341_GREEN);
  tft.println("56789!@#$%");

  tft.drawRect(0, 150, 100, 50, ILI9341_WHITE);
  tft.drawLine(0, 150, 100, 50, ILI9341_GREEN);
  tft.fillRectVGradient(125, 150, 50, 50, ILI9341_GREEN, ILI9341_YELLOW);
  tft.fillRectHGradient(200, 150, 50, 50, ILI9341_YELLOW, ILI9341_GREEN);
  // Try a read rect and write rect
#ifdef DEBUG_PIN
  digitalWrite(DEBUG_PIN, HIGH);
#endif

  tft.readRect(0, 0, 50, 50, pixel_data);

#ifdef DEBUG_PIN
  digitalWrite(DEBUG_PIN, LOW);
#endif
  tft.writeRect(250, 0, 50, 50, pixel_data);

  // Lets try to pack this rectangle of data into 8 byte
  tft.readRect(85, 65, 50, 50, pixel_data);
  uint16_t *ppd16 = pixel_data;
  uint8_t *ppd8 = (uint8_t*)pixel_data;
  uint8_t palette_cnt = 0;
  int palette_index;
  for (int i = 0; i < 2500; i++) {
    for (palette_index = 0; palette_index < palette_cnt; palette_index++) {
      if (*ppd16 == palette[palette_index])
        break;
    }
    if (palette_index >= palette_cnt) {
      palette[palette_cnt++] = *ppd16;  // save away the color
    }
    *ppd8++ = palette_index;
    ppd16++;
  }
  tft.writeRect8BPP(200, 50, 50, 50, (uint8_t*)pixel_data, palette);
  palette[0] = ILI9341_CYAN; 
  palette[1] = ILI9341_OLIVE; 
  tft.writeRect1BPP(75, 100, 16, 16, pict1bpp, palette);
  tft.writeRect1BPP(320-90, 75, 16, 16, pict1bpp, palette);
  
  palette[2] = ILI9341_MAROON; 
  palette[3] = ILI9341_PINK; 
  tft.writeRect2BPP(75, 125, 32, 16, pict2bpp, palette);

  tft.writeRectNBPP(15, 125, 32, 16, 2, pict2bpp, palette);
  tft.writeRectNBPP(75, 150, 16, 16, 4, pict4bpp, palette);

  // Try drawing button
  tft.setFontAdafruit();
  button.drawButton();

  tft.updateScreen();

  Serial.println(millis() - start_time, DEC);

  use_fb = !use_fb;

}

void drawTextScreen(bool fOpaque) {
  SetupOrClearClipRectAndOffsets();
  uint32_t start_time = millis();
  tft.useFrameBuffer(use_fb);
  tft.fillScreen(use_fb ? ILI9341_RED : ILI9341_BLACK);
  tft.setFont(Arial_40_Bold);
  if (fOpaque)
    tft.setTextColor(ILI9341_WHITE, use_fb ? ILI9341_BLACK : ILI9341_RED);
  else
    tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(0, 5);
  tft.println("AbCdEfGhIj");
  tft.setFont(Arial_28_Bold);
  tft.println("0123456789!@#$");
#if 1
  tft.setFont(Arial_20_Bold);
  tft.println("abcdefghijklmnopq");
  tft.setFont(Arial_14_Bold);
  tft.println("ABCDEFGHIJKLMNOPQRST");
  tft.setFont(Arial_10_Bold);
  tft.println("0123456789zyxwvutu");
#endif
  tft.updateScreen();

  Serial.printf("Use FB: %d OP: %d, DT: %d OR: %d\n", use_fb, fOpaque, use_set_origin, millis() - start_time);
}

void loop(void) {
  // See if any text entered
  int ich;
  if ((ich = Serial.read()) != -1) {
    while (Serial.read() != -1) ;
    if (ich == 'c') {
      use_clip_rect = !use_clip_rect;
      if (use_clip_rect) Serial.println("Clip Rectangle Turned on");
      else Serial.println("Clip Rectangle turned off");
      return;
    }
    if (ich == 's') {
      use_set_origin = !use_set_origin;
      if (use_set_origin) Serial.printf("Set origin to %d, %d\n", ORIGIN_TEST_X, ORIGIN_TEST_Y);
      else Serial.println("Clear origin");
      return;
    }
    if (ich == 'o')
      drawTextScreen(1);
    else if (ich == 't')
      drawTextScreen(0);
    else
      drawTestScreen();
  }

}
