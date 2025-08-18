/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


#include <SPI.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_ComicSansMS.h>

// For the Adafruit shield, these are the default.
#define ILI9341_RST 8
#define ILI9341_DC 9
#define ILI9341_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3n tft = ILI9341_t3n(ILI9341_CS, ILI9341_DC, ILI9341_RST);

// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);


/*********************************************************************************
 * MISO needs to be connected and working for this example
 * to work properly, so provide a way to check that and tell the user
 * the result. Implemeted as a template class, so it should be easy
 * to transfer to any display which provides writeRect() and readRect()
 */
template <class TFTdrv>
class checkMISO
{
    TFTdrv& tft;
    bool miso_ok{false};
  public:
    checkMISO(TFTdrv& _tft) 
      : tft(_tft)
      {}

    bool isOK(uint16_t x=0, uint16_t y=0)
    {
      uint16_t ref[]{ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE, ILI9341_WHITE};
      uint16_t chk[4]{0,0,0,0};
      tft.writeRect(x,y,2,2,ref);
      tft.readRect(x,y,2,2,chk);

      return 0 == memcmp(ref,chk,sizeof ref);
    }      
};


// One way of reporting to the user - use the Serial port
void printMISOcheck(void)
{
  checkMISO<ILI9341_t3n> checker(tft);
  Serial.printf("Screen %s functional MISO\n",checker.isOK()?"has":"does NOT have");
}
//********************************************************************************


void setup() {

  Serial.begin(9600);
 
  tft.begin();
  tft.setRotation(3);
  tft.enableScroll();

  tft.fillScreen(ILI9341_NAVY);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20,120-8);
  tft.print("Waiting for");
  tft.setCursor(20,120+8);
  tft.print("serial connection...");
  while (!Serial) // wait for Serial connection
    ; 
  printMISOcheck();
}


void scrollCS1(const ILI9341_t3_font_t& font, bool reverse=false)
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.setScrollTextArea(0,0,120,240);
  tft.setScrollBackgroundColor(ILI9341_GREEN);

  tft.setCursor(180, 100);
  tft.setFont(font);
  tft.print("Fixed text");


  tft.setTextColor(ILI9341_BLACK); 

  if (reverse)
  {
    unsigned char line_space = font.line_space;
    int ypos = 240-line_space+4;
    for(int i=20;i>=0;i--){
      tft.setCursor(0, ypos);
      tft.print("  this is line ");
      tft.print(i);
      delay(100);
      if (ypos >= line_space)
        ypos -= line_space;
      else        
        tft.scrollTextArea(-line_space);     
    }
  }
  else
  {
    tft.setCursor(0, 2);
    for(int i=0;i<=20;i++){
      tft.print("  this is line ");
      tft.println(i);
      delay(100);
    }
  }
}


void scrollCS2(const ILI9341_t3_font_t& font, bool reverse=false)
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setScrollTextArea(40,50,120,120);
  tft.setScrollBackgroundColor(ILI9341_GREEN);
  tft.setFont(font);

  tft.setTextSize(1);
  
  if (reverse)
  {
    unsigned char line_space = font.line_space;
    int ypos = 50+120-line_space-2;
    for(int i=20;i>=0;i--){
      tft.setCursor(40, ypos);
      tft.print("  this is line ");
      tft.print(i);
      delay(500);
      if (ypos-50 >= line_space)
        ypos -= line_space;
      else        
        tft.scrollTextArea(-line_space);     
    }
  }
  else
  {
    tft.setCursor(40, 52);  
    for(int i=0;i<=20;i++){
      tft.print("  this is line ");
      tft.println(i);
      delay(500);
    }
  }
}  


void loop(void) 
{
  scrollCS1(ComicSansMS_12);
  scrollCS1(ComicSansMS_12, true);
  scrollCS2(ComicSansMS_10);
  scrollCS2(ComicSansMS_10, true);
}
