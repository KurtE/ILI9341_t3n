Overview and Warning: 
=====
This is a modified version of the official PJRC ILI9341_t3 library (https://github.com/PaulStoffregen/ILI9341_t3).
And it is always a Work In Progress.

This library borrows some concepts and functionality like the usage of DMA from another variant library: https://github.com/FrankBoesing/ILI9341_t3DMA

This library was originally created to be able to test out SPI on the newer Teensy boards (3.5 and 3.6), which have multiple SPI busses. Later it was
also adapted to allow this on the Teensy LC as well. 


SPIN
----
Currently this library uses my SPIN library (https://github.com/KurtE/SPIN), which allows me to use different SPI busses.

The constructor for the class now takes an optional pointer to a SPIN object, which defaults to SPIN which is for the SPI buss.  To use SPI1, pass in &SPIN1, and likewise for SPI2 buss pass in &SPIN2.  Note: there is also code in place that if you forget to update which SPIN object to use and the begin method finds that the passed in values for MISO/MOSI/SCK are not valid for the currently selected buss, it will see if those pins are valid on SPI1 or SPI2 and if so, will recover and set to use the appropriate SPIN object. 

In addition, this code allows the ILI9341 code to work with only one hardware CS pin available, 
which in this case must be used for the DC pin.  This is very useful to support SPI1 on the new T3.5 and T3.6 boards which only
have one CS pin unless you use some form of adapter to use the SPI pins that are on the SDCARD. 

Frame Buffer
------------
On the T3.6 which have a lot more memory than previous Teensy processors, I also borrowed from the DMA version of the library and added code to be able to use a logical Frame Buffer.  To enable this I added a couple of API's 

    uint8_t useFrameBuffer(boolean b) - if b non-zero it will allocate memory and start using
    void	freeFrameBuffer(void) - Will free up the memory that was used.
    void	updateScreen(void); - Will update the screen with all of your updates...
	void	setFrameBuffer(uint16_t *frame_buffer); - Now have the ability allocate the frame buffer and pass it in, to avoid use of malloc

Asynchronous Update support (Frame buffer)
------------------------

The code now has support to use DMA for Asynchronous updates of the screen.  You can choose to do the updates once or in continuous mode.  Note: I mainly use the 
oneshot as I prefer more control on when the screen updates which helps to minimize things like flashing and tearing. 
Some of the New methods for this include: 


	bool	updateScreenAsync(bool update_cont = false); - Starts an update either one shot or continuous
	void	waitUpdateAsyncComplete(void);  - Wait for any active update to complete
	void	endUpdateAsync();			 - Turn of the continuous mode.
	boolean	asyncUpdateActive(void)      - Lets you know if an async operation is still active


Additional APIs
---------------
In addition, this library now has some of the API's and functionality that has been requested in a pull request.  In particular it now supports, the ability to set a clipping rectangle as well as setting an origin that is used with the drawing primitives.   These new API's include:

	void setOrigin(int16_t x = 0, int16_t y = 0); 
	void getOrigin(int16_t* x, int16_t* y);
	void setClipRect(int16_t x1, int16_t y1, int16_t w, int16_t h); 
	void setClipRect();

Discussion regarding this optimized version:
==========================

http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-%28320x240-TFT-color-display%29-library

This version of the library supports the Adafruit displays that use the ILI9341 displays, but in
addition are setup to support the displays that are sold by PJRC, which include:
	http://pjrc.com/store/display_ili9341.html
	http://pjrc.com/store/display_ili9341_touch.html

Note: this library like the ILI9341_t3 library which it is derived from no longer  require any of the Adafruit libraries, such as their Adafruit_ILI9341 and Adafruit_GFX libraries APIS are based on.

Adafruit library info
=======================

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

Future Updates
==============

I am hoping to phase out SPIN and be able to directly use SPI.  This has been helped by the recent updates to SPI, which all of the SPI objects are of one class. Currently I 
still need a few additional things like pointers to the underlying SPI registers.  But hopefully will get there. 

Again WIP
=====
