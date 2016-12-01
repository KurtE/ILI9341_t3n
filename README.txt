Warning: This is a modified version of the official PJRC ILI9341_t3 library (https://github.com/PaulStoffregen/ILI9341_t3).

Maybe some day, some of the additional functionality in this library will be merged into the official library or into another
derivative which supports DMA access to the screen that can be found at: https://github.com/FrankBoesing/ILI9341_t3DMA


In particular - This uses my SPIN library (https://github.com/KurtE/SPIN), which allows me to use different SPI busses, by
passing in which SPIN object to use.  It also allows multiple instances of this class, with a possible mix of SPI busses.

The constructor for the class now takes an optional pointer to a SPIN object, which defaults to SPIN which is for the SPI buss.  To use SPI1, pass in &SPIN1, and likewise for SPI2 buss pass in &SPIN2.  Note: there is also code in place that if you forget to update which SPIN object to use and the begin method finds that the passed in values for MISO/MOSI/SCK are not valid for the currently selected buss, it will see if those pins are valid on SPI1 or SPI2 and if so, will recover and set to use the appropriate SPIN object. 

In addition, this code allows the ILI9341 code to work with only one hardware CS pin available, 
which in this case must be used for the DC pin.  This is very useful to support SPI1 on the new T3.5 and T3.6 boards which only
have one CS pin unless you use some form of adapter to use the SPI pins that are on the SDCARD. 

On the T3.5 and T3.6 which have a lot more memory than previous Teensy processors, I also borrowed from the DMA version of the library and added code to be able to use a logical Frame Buffer.  To enable this I added a couple of API's 

    uint8_t useFrameBuffer(boolean b) - if b non-zero it will allocate memory and start using
    void	freeFrameBuffer(void) - Will free up the memory that was used.
    void	updateScreen(void); - Will update the screen with all of your updates...

In addition, this library now has some of the API's and functionality that has been requested in a pull request.  In particular it now supports, the ability to set a clipping rectangle as well as setting an origin that is used with the drawing primitives.   These new API's include:

	void setOrigin(int16_t x = 0, int16_t y = 0); 
	void getOrigin(int16_t* x, int16_t* y);
	void setClipRect(int16_t x1, int16_t y1, int16_t w, int16_t h); 
	void setClipRect();

Discussion regarding this optimized version:

http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-%28320x240-TFT-color-display%29-library

This version of the library supports the Adafruit displays that use the ILI9341 displays, but in
addition are setup to support the displays that are sold by PJRC, which include:
	http://pjrc.com/store/display_ili9341.html
	http://pjrc.com/store/display_ili9341_touch.html

Note: this library like the the ILI9341_t3 library which it is derived from no longer  require any of the Adafruit libraries, such as their Adafruit_ILI9341 and Adafruit_GFX libraries APIS are based on.

But as this code is based of of their work, their original information is included below:

------------------------------------------

This is a library for the Adafruit ILI9341 display products

This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
  ----> http://www.adafruit.com/products/1651
 
Check out the links above for our tutorials and wiring diagrams.
These displays use SPI to communicate, 4 or 5 pins are required
to interface (RST is optional).

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
MIT license, all text above must be included in any redistribution

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder Adafruit_ILI9341. Check that the Adafruit_ILI9341 folder contains Adafruit_ILI9341.cpp and Adafruit_ILI9341.

Place the Adafruit_ILI9341 library folder your arduinosketchfolder/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE

Also requires the Adafruit_GFX library for Arduino.
