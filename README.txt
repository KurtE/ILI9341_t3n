Warning: This is a hacked up version of the official PJRC ILI9341_t3 library and may be temporary

In particular - This uses my SPIN library, which allows me to use different SPI busses, by
passing in which SPIN object to use.  It also allows multiple instances of this class, with
a possible mix of SPI busses.

In addition, this code allows the ILI9341 code to work with only one hardware CS pin available, 
which in this case must be used for the DC pin.  This is being used in the Teensy 3.5 Hardware
Testing. 

Discussion regarding this optimized version:

http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-%28320x240-TFT-color-display%29-library

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
