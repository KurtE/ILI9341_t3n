// https://github.com/PaulStoffregen/ILI9341_t3n
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library

/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
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
// <SoftEgg>

//Additional graphics routines by Tim Trzepacz, SoftEgg LLC added December 2015
//(And then accidentally deleted and rewritten March 2016. Oops!)
//Gradient support
//----------------
//		fillRectVGradient	- fills area with vertical gradient
//		fillRectHGradient	- fills area with horizontal gradient
//		fillScreenVGradient - fills screen with vertical gradient
// 	fillScreenHGradient - fills screen with horizontal gradient

//Additional Color Support
//------------------------
//		color565toRGB		- converts 565 format 16 bit color to RGB
//		color565toRGB14		- converts 16 bit 565 format color to 14 bit RGB (2 bits clear for math and sign)
//		RGB14tocolor565		- converts 14 bit RGB back to 16 bit 565 format color

//Low Memory Bitmap Support
//-------------------------
// 		writeRect8BPP - 	write 8 bit per pixel paletted bitmap
// 		writeRect4BPP - 	write 4 bit per pixel paletted bitmap
// 		writeRect2BPP - 	write 2 bit per pixel paletted bitmap
// 		writeRect1BPP - 	write 1 bit per pixel paletted bitmap

//TODO: transparent bitmap writing routines for sprites

//String Pixel Length support
//---------------------------
//		strPixelLen			- gets pixel length of given ASCII string

// <\SoftEgg>

#include "ILI9341_t3n.h"
#include <SPIN.h>
#include <SPI.h>  

//#define DEBUG_ASYNC_UPDATE  // Enable to print out dma info
//#define DEBUG_ASYNC_LEDS	// Enable to use digitalWrites to Debug
#ifdef DEBUG_ASYNC_LEDS
#define DEBUG_PIN_1 0
#define DEBUG_PIN_2 1
#define DEBUG_PIN_3 4
#endif

#ifdef ENABLE_ILI9341_FRAMEBUFFER
#define CBALLOC (ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH*2)
#define	COUNT_WORDS_WRITE  ((ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH)/SCREEN_DMA_NUM_SETTINGS) // Note I know the divide will give whole number

#if defined(__MK66FX1M0__) 
	// T3.6 use Scatter/gather with chain to do transfer
DMASetting 	ILI9341_t3n::_dmasettings[4];
DMAChannel 	ILI9341_t3n::_dmatx;
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
DMASetting 	ILI9341_t3n::_dmasettings[4];
DMAChannel 	ILI9341_t3n::_dmatx;
#else
// T3.5 - had issues scatter/gather so do just use channels/interrupts
// and update and continue
DMAChannel  ILI9341_t3n::_dmatx;
DMAChannel  ILI9341_t3n::_dmarx;
uint16_t 	ILI9341_t3n::_dma_count_remaining;
uint16_t	ILI9341_t3n::_dma_write_size_words;
volatile short _dma_dummy_rx;
#endif	

ILI9341_t3n *ILI9341_t3n::_dmaActiveDisplay = 0;
volatile uint8_t  	ILI9341_t3n::_dma_state = 0;  // Use pointer to this as a way to get back to object...
volatile uint32_t	ILI9341_t3n::_dma_frame_count = 0;	// Can return a frame count...

void ILI9341_t3n::dmaInterrupt(void) {
	if (_dmaActiveDisplay)  {
		_dmaActiveDisplay->process_dma_interrupt();
	}
}

#ifdef DEBUG_ASYNC_UPDATE
extern void dumpDMA_TCD(DMABaseClass *dmabc);
#endif

void ILI9341_t3n::process_dma_interrupt(void) {
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, HIGH);
#endif
#if defined(__MK66FX1M0__) 
	// T3.6
	_dma_frame_count++;
	_dmatx.clearInterrupt();

	// See if we are in continuous mode or not..
	if ((_dma_state & ILI9341_DMA_CONT) == 0) {
		// We are in single refresh mode or the user has called cancel so
		// Lets try to release the CS pin
		_pspin->waitFifoNotFull();
		writecommand_last(ILI9341_NOP);
		endSPITransaction();
		_dma_state &= ~ILI9341_DMA_ACTIVE;
		_dmaActiveDisplay = 0;	// We don't have a display active any more... 

	}
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
	// T4

	_dma_frame_count++;

	if ((_dma_state & ILI9341_DMA_CONT) == 0) {
		// We are in single refresh mode or the user has called cancel so
		// Lets try to release the CS pin
		// Lets wait until FIFO is not empty
		// Serial4.printf("Before FSR wait: %x %x\n", _pimxrt_spi->FSR, _pimxrt_spi->SR);
		while (_pimxrt_spi->FSR & 0x1f)  ;	// wait until this one is complete

		// Serial4.printf("Before SR busy wait: %x\n", _pimxrt_spi->SR);
		while (_pimxrt_spi->SR & LPSPI_SR_MBF)  ;	// wait until this one is complete

		_dmatx.clearComplete();
		// Serial4.println("Restore FCR");
		_pimxrt_spi->FCR = LPSPI_FCR_TXWATER(15); // _spi_fcr_save;	// restore the FSR status... 
 		_pimxrt_spi->DER = 0;		// DMA no longer doing TX (or RX)

		_pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF;   // actually clear both...
		_pimxrt_spi->SR = 0x3f00;	// clear out all of the other status...


		maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7));	// output Command with 8 bits
		// Serial4.printf("Output NOP (SR %x CR %x FSR %x FCR %x %x TCR:%x)\n", _pimxrt_spi->SR, _pimxrt_spi->CR, _pimxrt_spi->FSR, 
		//	_pimxrt_spi->FCR, _spi_fcr_save, _pimxrt_spi->TCR);
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif
		_pspin->pending_rx_count = 0;	// Make sure count is zero
		writecommand_last(ILI9341_NOP);
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, LOW);
#endif

		// Serial4.println("Do End transaction");
		endSPITransaction();
		_dma_state &= ~ILI9341_DMA_ACTIVE;
		_dmaActiveDisplay = 0;	// We don't have a display active any more... 

 		// Serial4.println("After End transaction");

	}
	_dmatx.clearInterrupt();

#else
	// T3.5...
	_dmarx.clearInterrupt();
	_dmatx.clearComplete();
	_dmarx.clearComplete();

	if (!_dma_count_remaining && !(_dma_state & ILI9341_DMA_CONT)) {
		// The DMA transfers are done.
		_dma_frame_count++;
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

		_pkinetisk_spi->RSER = 0;
		//_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);  // clear out the queue
		_pkinetisk_spi->SR = 0xFF0F0000;
		_pkinetisk_spi->CTAR0  &= ~(SPI_CTAR_FMSZ(8)); 	// Hack restore back to 8 bits

		writecommand_last(ILI9341_NOP);
		endSPITransaction();
		_dma_state &= ~ILI9341_DMA_ACTIVE;
		_dmaActiveDisplay = 0;	// We don't have a display active any more... 
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, LOW);
#endif

	} else {
		uint16_t w;
		if (_dma_count_remaining) { // Still part of one frome. 
			_dma_count_remaining -= _dma_write_size_words;
			w = *((uint16_t*)_dmatx.TCD->SADDR);
			_dmatx.TCD->SADDR = (volatile uint8_t*)(_dmatx.TCD->SADDR) + 2;
		} else {  // start a new frame
			_dma_frame_count++;
			_dmatx.sourceBuffer(&_pfbtft[1], (_dma_write_size_words-1)*2);
			_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
			w = _pfbtft[0];
			_dma_count_remaining = CBALLOC/2 - _dma_write_size_words;	// how much more to transfer? 
		}
#ifdef DEBUG_ASYNC_UPDATE
//		dumpDMA_TCD(&_dmatx);
//		dumpDMA_TCD(&_dmarx);
#endif
		_pkinetisk_spi->PUSHR = (w | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT);
		_dmarx.enable();
		_dmatx.enable();
	}

#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, LOW);
#endif
}

#endif
// Teensy 3.1 can only generate 30 MHz SPI when running at 120 MHz (overclock)

#define WIDTH  ILI9341_TFTWIDTH
#define HEIGHT ILI9341_TFTHEIGHT

// Constructor when using hardware ILI9241_KINETISK__pspi->  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ILI9341_t3n::ILI9341_t3n(uint8_t cs, uint8_t dc, uint8_t rst, 
	uint8_t mosi, uint8_t sclk, uint8_t miso, SPINClass *pspin )
{
	_cs   = cs;
	_dc   = dc;
	_rst  = rst;
	_mosi = mosi;
	_sclk = sclk;
	_miso = miso;
	_width    = WIDTH;
	_height   = HEIGHT;
	_pspin	  = pspin; 
#ifdef KINETISK	
	_pkinetisk_spi = &_pspin->port();
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
 	_pimxrt_spi = &_pspin->port();
#else
	_pkinetisl_spi = &_pspin->port();
#endif	

	rotation  = 0;
	cursor_y  = cursor_x    = 0;
	textsize  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	setClipRect();
	setOrigin();

	// Added to see how much impact actually using non hardware CS pin might be
    _cspinmask = 0;
    _csport = NULL;

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
    _pfbtft = NULL;	
    _use_fbtft = 0;						// Are we in frame buffer mode?
	_we_allocated_buffer = NULL;
    #endif

}

//=======================================================================
// Add optinal support for using frame buffer to speed up complex outputs
//=======================================================================
void ILI9341_t3n::setFrameBuffer(uint16_t *frame_buffer) 
{
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	_pfbtft = frame_buffer;
	if (_pfbtft != NULL) {
		memset(_pfbtft, 0, ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH*2);
	}

	#endif	
}

uint8_t ILI9341_t3n::useFrameBuffer(boolean b)		// use the frame buffer?  First call will allocate
{
	#ifdef ENABLE_ILI9341_FRAMEBUFFER

	if (b) {
		// First see if we need to allocate buffer
		if (_pfbtft == NULL) {
			// Hack to start frame buffer on 32 byte boundary
			_we_allocated_buffer = (uint16_t *)malloc(CBALLOC+32);
			if (_we_allocated_buffer == NULL)
				return 0;	// failed 
			_pfbtft = (uint16_t*) (((uintptr_t)_we_allocated_buffer + 32) & ~ ((uintptr_t) (31)));
			memset(_pfbtft, 0, CBALLOC);	
		}
		_use_fbtft = 1;
	} else 
		_use_fbtft = 0;

	return _use_fbtft;	
	#else
	return 0;
	#endif
}

void ILI9341_t3n::freeFrameBuffer(void)						// explicit call to release the buffer
{
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_we_allocated_buffer) {
		free(_we_allocated_buffer);
		_pfbtft = NULL;
		_use_fbtft = 0;	// make sure the use is turned off
		_we_allocated_buffer = NULL;
	}
	#endif
}
void ILI9341_t3n::updateScreen(void)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		beginSPITransaction();
		if (_standard) {
			// Doing full window. 
			setAddr(0, 0, _width-1, _height-1);
			writecommand_cont(ILI9341_RAMWR);

			// BUGBUG doing as one shot.  Not sure if should or not or do like
			// main code and break up into transactions...
			uint16_t *pfbtft_end = &_pfbtft[(ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT)-1];	// setup 
			uint16_t *pftbft = _pfbtft;

			// Quick write out the data;
			while (pftbft < pfbtft_end) {
				writedata16_cont(*pftbft++);
			}
			writedata16_last(*pftbft);
		} else {
			// setup just to output the clip rectangle area. 
			setAddr(_displayclipx1, _displayclipy1, _displayclipx2-1, _displayclipy2-1);
			writecommand_cont(ILI9341_RAMWR);

			// BUGBUG doing as one shot.  Not sure if should or not or do like
			// main code and break up into transactions...
			uint16_t * pfbPixel_row = &_pfbtft[ _displayclipy1*_width + _displayclipx1];
			for (uint16_t y = _displayclipy1; y < _displayclipy2; y++) {
				uint16_t * pfbPixel = pfbPixel_row;
				for (uint16_t x = _displayclipx1; x < (_displayclipx2-1); x++) {
					writedata16_cont(*pfbPixel++);
				}
				if (y < (_displayclipy2-1))
					writedata16_cont(*pfbPixel);
				else	
					writedata16_last(*pfbPixel);
				pfbPixel_row += _width;	// setup for the next row. 
			}
		}
		endSPITransaction();
	}
	#endif
}			 

#ifdef DEBUG_ASYNC_UPDATE
void dumpDMA_TCD(DMABaseClass *dmabc)
{
	Serial4.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

	Serial4.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
		dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR, 
		dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif

#ifdef ENABLE_ILI9341_FRAMEBUFFER
//==============================================
#ifdef ENABLE_ILI9341_FRAMEBUFFER
void	ILI9341_t3n::initDMASettings(void) 
{
	// Serial4.printf("initDMASettings called %d\n", _dma_state);
	if (_dma_state) {  // should test for init, but...
		return;	// we already init this. 
	}

	//Serial.println("InitDMASettings");
	uint8_t dmaTXevent = _pspin->dmaTXEvent();
#if defined(__MK66FX1M0__) 
	// T3.6

	// BUGBUG:: check for -1 as wont work on SPI2 on T3.5
//	uint16_t *fbtft_start_dma_addr = _pfbtft;

	
	//Serial.printf("CWW: %d %d %d\n", CBALLOC, SCREEN_DMA_NUM_SETTINGS, count_words_write);
	// Now lets setup DMA access to this memory... 
	_dmasettings[0].sourceBuffer(&_pfbtft[1], (COUNT_WORDS_WRITE-1)*2);
	_dmasettings[0].destination(_pkinetisk_spi->PUSHR);

	// Hack to reset the destination to only output 2 bytes.
	_dmasettings[0].TCD->ATTR_DST = 1;
	_dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

	_dmasettings[1].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE], COUNT_WORDS_WRITE*2);
	_dmasettings[1].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[1].TCD->ATTR_DST = 1;
	_dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);

	_dmasettings[2].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE*2], COUNT_WORDS_WRITE*2);
	_dmasettings[2].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[2].TCD->ATTR_DST = 1;
	_dmasettings[2].replaceSettingsOnCompletion(_dmasettings[3]);

	// Sort of hack - but wrap around to output the first word again. 
	_dmasettings[3].sourceBuffer(_pfbtft, 2);
	_dmasettings[3].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[3].TCD->ATTR_DST = 1;
	_dmasettings[3].replaceSettingsOnCompletion(_dmasettings[0]);

	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	_dmatx.begin(true);
	_dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dmatx = _dmasettings[0];
	_dmatx.attachInterrupt(dmaInterrupt);
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
	// Now lets setup DMA access to this memory... 
	// Try to do like T3.6 except not kludge for first word...
	// Serial4.println("DMA initDMASettings - before settings");
	// Serial4.printf("  CWW: %d %d %d\n", CBALLOC, SCREEN_DMA_NUM_SETTINGS, COUNT_WORDS_WRITE);
	_dmasettings[0].sourceBuffer(&_pfbtft[0], COUNT_WORDS_WRITE*2);
	_dmasettings[0].destination(_pimxrt_spi->TDR);
	_dmasettings[0].TCD->ATTR_DST = 1;
	_dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

	_dmasettings[1].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE], COUNT_WORDS_WRITE*2);
	_dmasettings[1].destination(_pimxrt_spi->TDR);
	_dmasettings[1].TCD->ATTR_DST = 1;
	_dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);

	_dmasettings[2].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE*2], COUNT_WORDS_WRITE*2);
	_dmasettings[2].destination(_pimxrt_spi->TDR);
	_dmasettings[2].TCD->ATTR_DST = 1;
	_dmasettings[2].replaceSettingsOnCompletion(_dmasettings[3]);

	_dmasettings[3].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE*3], COUNT_WORDS_WRITE*2);
	_dmasettings[3].destination(_pimxrt_spi->TDR);
	_dmasettings[3].TCD->ATTR_DST = 1;
	_dmasettings[3].replaceSettingsOnCompletion(_dmasettings[0]);
	_dmasettings[3].interruptAtCompletion();

	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	// Serial4.println("DMA initDMASettings - before dmatx");
	_dmatx.begin(true);
	_dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dmatx = _dmasettings[0];
	_dmatx.attachInterrupt(dmaInterrupt);
#else
	// T3.5
	// Lets setup the write size.  For SPI we can use up to 32767 so same size as we use on T3.6...
	// But SPI1 and SPI2 max of 511.  We will use 480 in that case as even divider...

	_dmarx.disable();
	_dmarx.source(_pkinetisk_spi->POPR);
	_dmarx.TCD->ATTR_SRC = 1;
	_dmarx.destination(_dma_dummy_rx);
	_dmarx.disableOnCompletion();
	_dmarx.triggerAtHardwareEvent(_pspin->dmaRXEvent());
	_dmarx.attachInterrupt(dmaInterrupt);
	_dmarx.interruptAtCompletion();

	// We may be using settings chain here so lets set it up. 
	// Now lets setup TX chain.  Note if trigger TX is not set
	// we need to have the RX do it for us.
	_dmatx.disable();
	_dmatx.destination(_pkinetisk_spi->PUSHR);
	_dmatx.TCD->ATTR_DST = 1;
	_dmatx.disableOnCompletion();
	// Current SPIN, has both RX/TX same for SPI1/2 so just know f
	if (_pspin == &SPIN) {
		_dmatx.triggerAtHardwareEvent(dmaTXevent);
		_dma_write_size_words = COUNT_WORDS_WRITE;
	} else {
		_dma_write_size_words = 480;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	}
	Serial.printf("Init DMA Settings: TX:%d size:%d\n", dmaTXevent, _dma_write_size_words);

#endif
	_dma_state = ILI9341_DMA_INIT;  // Should be first thing set!
	// Serial4.println("DMA initDMASettings - end");

}
#endif

void ILI9341_t3n::dumpDMASettings() {
#ifdef DEBUG_ASYNC_UPDATE
#if defined(__MK66FX1M0__) 
	// T3.6
	Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmasettings[0]);
	dumpDMA_TCD(&_dmasettings[1]);
	dumpDMA_TCD(&_dmasettings[2]);
	dumpDMA_TCD(&_dmasettings[3]);
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
	// Serial4.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmasettings[0]);
	dumpDMA_TCD(&_dmasettings[1]);
	dumpDMA_TCD(&_dmasettings[2]);
	dumpDMA_TCD(&_dmasettings[3]);
#else
	Serial.printf("DMA dump TX:%d RX:%d\n", _dmatx.channel, _dmarx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmarx);
#endif	
#endif

}

bool ILI9341_t3n::updateScreenAsync(bool update_cont)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	// BUGBUG:: only handles full screen so bail on the rest of it...
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (!_use_fbtft) return false;


	#if defined(__MK64FX512__)  // If T3.5 only allow on SPI...
	// The T3.5 DMA to SPI has issues with preserving stuff like we want 16 bit mode
	// and we want CS to stay on... So hack it.  We will turn off using CS for the CS
	//	pin.
	if (!_csport) {
		pcs_data = 0;
		pcs_command = pcs_data | _pspin->setCS(_dc);
		pinMode(_cs, OUTPUT);
		_csport    = portOutputRegister(digitalPinToPort(_cs));
		_cspinmask = digitalPinToBitMask(_cs);
		*_csport |= _cspinmask;
	}
	#endif

#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, HIGH);
#endif
	// Init DMA settings. 
	initDMASettings();

	// Don't start one if already active.
	if (_dma_state & ILI9341_DMA_ACTIVE) {
	#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_1, LOW);
	#endif
		return false;
	}

#if defined(__MK66FX1M0__) 
	//==========================================
	// T3.6
	//==========================================
	if (update_cont) {
		// Try to link in #3 into the chain
		_dmasettings[2].replaceSettingsOnCompletion(_dmasettings[3]);
		_dmasettings[2].TCD->CSR &= ~(DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ);  // Don't interrupt on this one... 
		_dmasettings[3].interruptAtCompletion();
		_dmasettings[3].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one  
		_dma_state |= ILI9341_DMA_CONT;
	} else {
		// In this case we will only run through once...
		_dmasettings[2].replaceSettingsOnCompletion(_dmasettings[0]);
		_dmasettings[2].interruptAtCompletion();
		_dmasettings[2].disableOnCompletion();
		_dma_state &= ~ILI9341_DMA_CONT;
	}


#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif
	beginSPITransaction();

	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand_cont(ILI9341_RAMWR);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	writedata16_cont(*_pfbtft);
	// now lets start up the DMA
//	volatile uint16_t  biter = _dmatx.TCD->BITER;
	//DMA_CDNE_CDNE(_dmatx.channel);
//	_dmatx = _dmasettings[0];
//	_dmatx.TCD->BITER = biter;
	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay = this;
	_dma_state |= ILI9341_DMA_ACTIVE;
	_pkinetisk_spi->RSER |= SPI_RSER_TFFF_DIRS |	 SPI_RSER_TFFF_RE;	 // Set DMA Interrupt Request Select and Enable register
	_pkinetisk_spi->MCR &= ~SPI_MCR_HALT;  //Start transfers.
	_dmatx.enable();
	//==========================================
	// T4
	//==========================================
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
	// TODO

	if (update_cont) {
		// Try to link in #3 into the chain
		_dmasettings[3].TCD->CSR &= ~( DMA_TCD_CSR_DREQ);  // Don't disable on completion.
	} else {
		// In this case we will only run through once...
		_dmasettings[3].disableOnCompletion();
		_dma_state &= ~ILI9341_DMA_CONT;
	}
#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif

	beginSPITransaction();
	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand_last(ILI9341_RAMWR);

	// Update TCR to 16 bit mode. and output the first entry.
	_spi_fcr_save = _pimxrt_spi->FCR;	// remember the FCR
	_pimxrt_spi->FCR = 0;	// clear water marks... 	
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_RXMSK /*| LPSPI_TCR_CONT*/);
//	_pimxrt_spi->CFGR1 |= LPSPI_CFGR1_NOSTALL;
//	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
 	_pimxrt_spi->DER = LPSPI_DER_TDDE;
	_pimxrt_spi->SR = 0x3f00;	// clear out all of the other status...

  	_dmatx.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_TX );

 	_dmatx = _dmasettings[0];

  	_dmatx.begin(false);
  	_dmatx.enable();

	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay = this;
	if (update_cont) {
		_dma_state |= ILI9341_DMA_CONT;
	} else {
		_dma_state &= ~ILI9341_DMA_CONT;

	}

	_dma_state |= ILI9341_DMA_ACTIVE;
#else
	//==========================================
	// T3.5
	//==========================================

	// lets setup the initial pointers. 
	_dmatx.sourceBuffer(&_pfbtft[1], (_dma_write_size_words-1)*2);
	_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
	_dmarx.transferCount(_dma_write_size_words);
	_dma_count_remaining = CBALLOC/2 - _dma_write_size_words;	// how much more to transfer? 
	Serial.printf("SPI1/2 - TC:%d TR:%d\n", _dma_write_size_words, _dma_count_remaining);

#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif

	beginSPITransaction();
	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand_cont(ILI9341_RAMWR);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	// On T3.5 DMA only appears to work with CTAR 0 so hack it up...
	_pkinetisk_spi->CTAR0 |= SPI_CTAR_FMSZ(8); 	// Hack convert from 8 bit to 16 bit...

	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);

	_pkinetisk_spi->SR = 0xFF0F0000;

	// Lets try to output the first byte to make sure that we are in 16 bit mode...
	_pkinetisk_spi->PUSHR = *_pfbtft | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;	

	if (_pspin == &SPIN) {
		// SPI - has both TX and RX so use it
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;

	    _dmarx.enable();
	    _dmatx.enable();
	} else {
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS ;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	    _dmatx.enable();
	    _dmarx.enable();
	}

	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay = this;
	if (update_cont) {
		_dma_state |= ILI9341_DMA_CONT;
	} else {
		_dma_state &= ~ILI9341_DMA_CONT;

	}

	_dma_state |= ILI9341_DMA_ACTIVE;
#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, LOW);
#endif
	return true;
    #else
    return false;     // no frame buffer so will never start... 
	#endif

}			 

void ILI9341_t3n::endUpdateAsync() {
	// make sure it is on
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_dma_state & ILI9341_DMA_CONT) {
		_dma_state &= ~ILI9341_DMA_CONT; // Turn of the continueous mode
#if defined(__MK66FX1M0__) 
		_dmasettings[3].disableOnCompletion();
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
		_dmasettings[3].disableOnCompletion();
#endif
	}
	#endif
}
	
void ILI9341_t3n::waitUpdateAsyncComplete(void) 
{
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

	while ((_dma_state & ILI9341_DMA_ACTIVE)) {
		// asm volatile("wfi");
	};
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, LOW);
#endif
	#endif	
}
#endif
//=======================================================================


void ILI9341_t3n::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	beginSPITransaction();
	setAddr(x0, y0, x1, y1);
	writecommand_last(ILI9341_RAMWR); // write to RAM
	endSPITransaction();
}

void ILI9341_t3n::pushColor(uint16_t color)
{
	beginSPITransaction();
	writedata16_last(color);
	endSPITransaction();
}

void ILI9341_t3n::drawPixel(int16_t x, int16_t y, uint16_t color) {
	x += _originx;
	y += _originy;
	if((x < _displayclipx1) ||(x >= _displayclipx2) || (y < _displayclipy1) || (y >= _displayclipy2)) return;

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		_pfbtft[y*_width + x] = color;

	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x, y);
		writecommand_cont(ILI9341_RAMWR);
		writedata16_last(color);
		endSPITransaction();
	}
}

void ILI9341_t3n::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	x+=_originx;
	y+=_originy;
	// Rectangular clipping
	if((x < _displayclipx1) || (x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if(y < _displayclipy1) { h = h - (_displayclipy1 - y); y = _displayclipy1;}
	if((y+h-1) >= _displayclipy2) h = _displayclipy2-y;
	if(h<1) return;

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel = &_pfbtft[ y*_width + x];
		while (h--) {
			*pfbPixel = color;
			pfbPixel += _width;
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x, y+h-1);
		writecommand_cont(ILI9341_RAMWR);
		while (h-- > 1) {
			writedata16_cont(color);
		}
		writedata16_last(color);
		endSPITransaction();
	}
}

void ILI9341_t3n::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	x+=_originx;
	y+=_originy;

	// Rectangular clipping
	if((y < _displayclipy1) || (x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if(x<_displayclipx1) { w = w - (_displayclipx1 - x); x = _displayclipx1; }
	if((x+w-1) >= _displayclipx2)  w = _displayclipx2-x;
	if (w<1) return;

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel = &_pfbtft[ y*_width + x];
			while (w--) {
				*pfbPixel++ = color;
			}
		} else {
			// X is even and so is w, try 32 bit writes..
			uint32_t color32 = (color << 16) | color;
			uint32_t * pfbPixel = (uint32_t*)((uint16_t*)&_pfbtft[ y*_width + x]);
			while (w) {
				*pfbPixel++ = color32;
				w -= 2;
			}
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x+w-1, y);
		writecommand_cont(ILI9341_RAMWR);
		while (w-- > 1) {
			writedata16_cont(color);
		}
		writedata16_last(color);
		endSPITransaction();
	}
}

void ILI9341_t3n::fillScreen(uint16_t color)
{
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft && _standard) {
		// Speed up lifted from Franks DMA code... _standard is if no offsets and rects..
		uint32_t color32 = (color << 16) | color;

		uint32_t *pfbPixel = (uint32_t *)_pfbtft;
		uint32_t *pfbtft_end = (uint32_t *)((uint16_t *)&_pfbtft[(ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT)]); // setup
		while (pfbPixel < pfbtft_end) {
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
			*pfbPixel++ = color32; *pfbPixel++ = color32; *pfbPixel++ = color32;*pfbPixel++ = color32;
		}

	} else 
	#endif
	{
		fillRect(0, 0, _width, _height, color);
	}
}

// fill a rectangle
void ILI9341_t3n::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	x+=_originx;
	y+=_originy;

	// Rectangular clipping (drawChar w/big text requires this)
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;
	if(x < _displayclipx1) {	w -= (_displayclipx1-x); x = _displayclipx1; 	}
	if(y < _displayclipy1) {	h -= (_displayclipy1 - y); y = _displayclipy1; 	}
	if((x + w - 1) >= _displayclipx2)  w = _displayclipx2  - x;
	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
			for (;h>0; h--) {
				uint16_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color;
				}
				pfbPixel_row += _width;
			}
		} else {
			// Horizontal is even number so try 32 bit writes instead
			uint32_t color32 = (color << 16) | color;
			uint32_t * pfbPixel_row = (uint32_t *)((uint16_t*)&_pfbtft[ y*_width + x]);
			w = w/2;	// only iterate half the times
			for (;h>0; h--) {
				uint32_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color32;
				}
				pfbPixel_row += (_width/2);
			}
		}
	} else 
	#endif
	{

		// TODO: this can result in a very long transaction time
		// should break this into multiple transactions, even though
		// it'll cost more overhead, so we don't stall other SPI libs
		beginSPITransaction();
		setAddr(x, y, x+w-1, y+h-1);
		writecommand_cont(ILI9341_RAMWR);
		for(y=h; y>0; y--) {
			for(x=w; x>1; x--) {
				writedata16_cont(color);
			}
			writedata16_last(color);
#if 0
			if (y > 1 && (y & 1)) {
				endSPITransaction();
				beginSPITransaction();
			}
#endif			
		}
		endSPITransaction();
	}
}

// fillRectVGradient	- fills area with vertical gradient
void ILI9341_t3n::fillRectVGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color1, uint16_t color2)
{
	x+=_originx;
	y+=_originy;

	// Rectangular clipping 
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if(x < _displayclipx1) {	w -= (_displayclipx1-x); x = _displayclipx1; 	}
	if(y < _displayclipy1) {	h -= (_displayclipy1 - y); y = _displayclipy1; 	}
	if((x + w - 1) >= _displayclipx2)  w = _displayclipx2  - x;
	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;
	
	int16_t r1, g1, b1, r2, g2, b2, dr, dg, db, r, g, b;
	color565toRGB14(color1,r1,g1,b1);
	color565toRGB14(color2,r2,g2,b2);
	dr=(r2-r1)/h; dg=(g2-g1)/h; db=(b2-b1)/h;
	r=r1;g=g1;b=b1;	

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
			for (;h>0; h--) {
				uint16_t color = RGB14tocolor565(r,g,b);
				uint16_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color;
				}
				r+=dr;g+=dg; b+=db;
				pfbPixel_row += _width;
			}
		} else {
			// Horizontal is even number so try 32 bit writes instead
			uint32_t * pfbPixel_row = (uint32_t *)((uint16_t*)&_pfbtft[ y*_width + x]);
			w = w/2;	// only iterate half the times
			for (;h>0; h--) {
				uint32_t * pfbPixel = pfbPixel_row;
				uint16_t color = RGB14tocolor565(r,g,b);
				uint32_t color32 = (color << 16) | color;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color32;
				}
				pfbPixel_row += (_width/2);
				r+=dr;g+=dg; b+=db;
			}
		}
	} else 
	#endif
	{		
		beginSPITransaction();
		setAddr(x, y, x+w-1, y+h-1);
		writecommand_cont(ILI9341_RAMWR);
		for(y=h; y>0; y--) {
			uint16_t color = RGB14tocolor565(r,g,b);

			for(x=w; x>1; x--) {
				writedata16_cont(color);
			}
			writedata16_last(color);
			if (y > 1 && (y & 1)) {
				endSPITransaction();
				beginSPITransaction();
			}
			r+=dr;g+=dg; b+=db;
		}
		endSPITransaction();
	}
}

// fillRectHGradient	- fills area with horizontal gradient
void ILI9341_t3n::fillRectHGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color1, uint16_t color2)
{
	x+=_originx;
	y+=_originy;

	// Rectangular clipping 
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if(x < _displayclipx1) {	w -= (_displayclipx1-x); x = _displayclipx1; 	}
	if(y < _displayclipy1) {	h -= (_displayclipy1 - y); y = _displayclipy1; 	}
	if((x + w - 1) >= _displayclipx2)  w = _displayclipx2  - x;
	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;
	
	int16_t r1, g1, b1, r2, g2, b2, dr, dg, db, r, g, b;
	uint16_t color;
	color565toRGB14(color1,r1,g1,b1);
	color565toRGB14(color2,r2,g2,b2);
	dr=(r2-r1)/w; dg=(g2-g1)/w; db=(b2-b1)/w;
	r=r1;g=g1;b=b1;	
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			for (int i = 0 ;i < w; i++) {
				*pfbPixel++ = RGB14tocolor565(r,g,b);
				r+=dr;g+=dg; b+=db;
			}
			pfbPixel_row += _width;
			r=r1;g=g1;b=b1;
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x+w-1, y+h-1);
		writecommand_cont(ILI9341_RAMWR);
		for(y=h; y>0; y--) {
			for(x=w; x>1; x--) {
				color = RGB14tocolor565(r,g,b);
				writedata16_cont(color);
				r+=dr;g+=dg; b+=db;
			}
			color = RGB14tocolor565(r,g,b);
			writedata16_last(color);
			if (y > 1 && (y & 1)) {
				endSPITransaction();
				beginSPITransaction();
			}
			r=r1;g=g1;b=b1;
		}
		endSPITransaction();
	}
}

// fillScreenVGradient - fills screen with vertical gradient
void ILI9341_t3n::fillScreenVGradient(uint16_t color1, uint16_t color2)
{
	fillRectVGradient(0,0,_width,_height,color1,color2);
}

// fillScreenHGradient - fills screen with horizontal gradient
void ILI9341_t3n::fillScreenHGradient(uint16_t color1, uint16_t color2)
{
	fillRectHGradient(0,0,_width,_height,color1,color2);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ILI9341_t3n::setRotation(uint8_t m)
{
	rotation = m % 4; // can't be higher than 3
	beginSPITransaction();
	writecommand_cont(ILI9341_MADCTL);
	switch (rotation) {
	case 0:
		writedata8_last(MADCTL_MX | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 1:
		writedata8_last(MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	case 2:
		writedata8_last(MADCTL_MY | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 3:
		writedata8_last(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	}
	endSPITransaction();
	setClipRect();
	setOrigin();
	
	cursor_x = 0;
	cursor_y = 0;
}

void ILI9341_t3n::setScroll(uint16_t offset)
{
	beginSPITransaction();
	writecommand_cont(ILI9341_VSCRSADD);
	writedata16_last(offset);
	endSPITransaction();
}

void ILI9341_t3n::invertDisplay(boolean i)
{
	beginSPITransaction();
	writecommand_last(i ? ILI9341_INVON : ILI9341_INVOFF);
	endSPITransaction();
}










/*
uint8_t ILI9341_t3n::readdata(void)
{
  uint8_t r;
       // Try to work directly with SPI registers...
       // First wait until output queue is empty
        uint16_t wTimeout = 0xffff;
        while (((_pkinetisk_spi->SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        
//       	_pkinetisk_spi->MCR |= SPI_MCR_CLR_RXF; // discard any received data
//		_pkinetisk_spi->SR = SPI_SR_TCF;
        
        // Transfer a 0 out... 
        writedata8_cont(0);   
        
        // Now wait until completed. 
        wTimeout = 0xffff;
        while (((_pkinetisk_spi->SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        r = _pkinetisk_spi->POPR;  // get the received byte... should check for it first...
    return r;
}
 */

uint8_t ILI9341_t3n::readcommand8(uint8_t c, uint8_t index)
{
    // Bail if not valid miso
    if (_miso == 0xff) return 0;

 #ifdef KINETISK
    uint16_t wTimeout = 0xffff;
    uint8_t r=0;

    beginSPITransaction();
	if (_pspin->sizeFIFO() >= 4) {
	    while (((_pkinetisk_spi->SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

	    // Make sure the last frame has been sent...
	    _pkinetisk_spi->SR = SPI_SR_TCF;   // dlear it out;
	    wTimeout = 0xffff;
	    while (!((_pkinetisk_spi->SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

	    // clear out any current received bytes
	    wTimeout = 0x10;    // should not go more than 4...
	    while ((((_pkinetisk_spi->SR) >> 4) & 0xf) && (--wTimeout))  {
	        r = _pkinetisk_spi->POPR;
	    }

	    //writecommand(0xD9); // sekret command
		_pkinetisk_spi->PUSHR = 0xD9 | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
	//	while (((_pkinetisk_spi->SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

	    // writedata(0x10 + index);
		_pkinetisk_spi->PUSHR = (0x10 + index) | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
	//	while (((_pkinetisk_spi->SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	    // writecommand(c);
	   	_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
	//	while (((_pkinetisk_spi->SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

	    // readdata
		_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
	//	while (((_pkinetisk_spi->SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

	    // Now wait until completed.
	    wTimeout = 0xffff;
	    while (((_pkinetisk_spi->SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

	    // Make sure the last frame has been sent...
	    _pkinetisk_spi->SR = SPI_SR_TCF;   // dlear it out;
	    wTimeout = 0xffff;
	    while (!((_pkinetisk_spi->SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

	    wTimeout = 0x10;    // should not go more than 4...
	    // lets get all of the values on the FIFO
	    while ((((_pkinetisk_spi->SR) >> 4) & 0xf) && (--wTimeout))  {
	        r = _pkinetisk_spi->POPR;
	    }
	} else {
		while (((_pkinetisk_spi->SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

	    // Make sure the last frame has been sent...
	    _pkinetisk_spi->SR = SPI_SR_TCF;   // dlear it out;
	    wTimeout = 0xffff;
	    while (!((_pkinetisk_spi->SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

	    // clear out any current received bytes
	    wTimeout = 0x10;    // should not go more than 4...
	    while ((((_pkinetisk_spi->SR) >> 4) & 0xf) && (--wTimeout))  {
	        r = _pkinetisk_spi->POPR;
	    }

	    //writecommand(0xD9); // sekret command
		_pkinetisk_spi->PUSHR = 0xD9 | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
		while (((_pkinetisk_spi->SR) & (15 << 12)) > (0 << 12)) ; // wait if FIFO full

	    // writedata(0x10 + index);
		_pkinetisk_spi->PUSHR = (0x10 + index) | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > (0 << 12)) ; // wait if FIFO full

		// See if there are any return values to pop 
	    while (((_pkinetisk_spi->SR) >> 4) & 0xf)  {
	        r = _pkinetisk_spi->POPR;
	    }

	    // writecommand(c);
	   	_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
		while (((_pkinetisk_spi->SR) & (15 << 12)) > (0 << 12)) ; // wait if FIFO full

		// See if there are any return values to pop 
	    while (((_pkinetisk_spi->SR) >> 4) & 0xf)  {
	        r = _pkinetisk_spi->POPR;
	    }

	    // readdata
		_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > (0 << 12)) ; // wait if FIFO full

		// See if there are any return values to pop 
	    while (((_pkinetisk_spi->SR) >> 4) & 0xf)  {
	        r = _pkinetisk_spi->POPR;
	    }
	    // Now wait until completed.
	    wTimeout = 0xffff;
	    while (((_pkinetisk_spi->SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

	    // Make sure the last frame has been sent...
	    _pkinetisk_spi->SR = SPI_SR_TCF;   // dlear it out;
	    wTimeout = 0xffff;
	    while (!((_pkinetisk_spi->SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

	    wTimeout = 0x10;    // should not go more than 4...
	    // lets get all of the values on the FIFO
	    while ((((_pkinetisk_spi->SR) >> 4) & 0xf) && (--wTimeout))  {
	        r = _pkinetisk_spi->POPR;
	    }

	}  
    endSPITransaction();
    return r;  // get the received byte... should check for it first...
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
    uint16_t wTimeout = 0xffff;
    uint8_t r=0;

    beginSPITransaction(ILI9341_SPICLOCK_READ);
    // Lets assume that queues are empty as we just started transaction.
	_pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF;   // actually clear both...
    //writecommand(0xD9); // sekret command
    maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
	_pimxrt_spi->TDR = 0xD9;

    // writedata(0x10 + index);
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
	_pimxrt_spi->TDR = 0x10 + index;

    // writecommand(c);
    maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
	_pimxrt_spi->TDR = c;

    // readdata
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));
	_pimxrt_spi->TDR = 0;

    // Now wait until completed.
    wTimeout = 0xffff;
    uint8_t rx_count = 4;
    while (rx_count && wTimeout) {
        if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)  {
            r =_pimxrt_spi->RDR;  // Read any pending RX bytes in
            rx_count--; //decrement count of bytes still levt
        }
    }
    endSPITransaction();
    return r;  // get the received byte... should check for it first...
#else
	beginSPITransaction();
	writecommand_cont(0xD9);
	writedata8_cont(0x10 + index);

	writecommand_cont(c);
	writedata8_cont(0);
	uint8_t r = waitTransmitCompleteReturnLast();
	endSPITransaction();
	return r;

#endif   
}

// Read Pixel at x,y and get back 16-bit packed color
#define READ_PIXEL_PUSH_BYTE 0x3f
uint16_t ILI9341_t3n::readPixel(int16_t x, int16_t y)
{
#ifdef KINETISK	
	//BUGBUG:: Should add some validation of X and Y
	// Now if we are in buffer mode can return real fast
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		x+=_originx;
		y+=_originy;

		return _pfbtft[y*_width + x] ;
	}
	#endif	

   if (_miso == 0xff) return 0xffff;	// bail if not valid miso

	// First pass for other SPI busses use readRect to handle the read... 
	if (_pspin->sizeFIFO() < 4) {
		uint16_t colors;
		readRect(x, y, 1, 1, &colors);
		return colors;
	}

	uint8_t dummy __attribute__((unused));
	uint8_t r,g,b;

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	// Update our origin. 
	x+=_originx;
	y+=_originy;

	setAddr(x, y, x, y);
	writecommand_cont(ILI9341_RAMRD); // read from RAM
	_pspin->waitTransmitComplete();

	// Push 4 bytes over SPI
	_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
	_pspin->waitFifoEmpty();    // wait for both queues to be empty.

	_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
	_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
	_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_EOQ;

	// Wait for End of Queue
	while ((_pkinetisk_spi->SR & SPI_SR_EOQF) == 0) ;
	_pkinetisk_spi->SR = SPI_SR_EOQF;  // make sure it is clear

	// Read Pixel Data
	dummy = _pkinetisk_spi->POPR;	// Read a DUMMY byte of GRAM
	r = _pkinetisk_spi->POPR;		// Read a RED byte of GRAM
	g = _pkinetisk_spi->POPR;		// Read a GREEN byte of GRAM
	b = _pkinetisk_spi->POPR;		// Read a BLUE byte of GRAM

	endSPITransaction();
	return color565(r,g,b);
#else
	// Kinetisk
	uint16_t colors = 0;
	readRect(x, y, 1, 1, &colors);
	return colors;
#endif	
}

// Now lets see if we can read in multiple pixels
#ifdef KINETISK
void ILI9341_t3n::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
	// Use our Origin. 
	x+=_originx;
	y+=_originy;
	//BUGBUG:: Should add some validation of X and Y

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			for (int i = 0 ;i < w; i++) {
				*pcolors++ = *pfbPixel++;
			}
			pfbPixel_row += _width;
		}
		return;	
	}
	#endif	

   if (_miso == 0xff) return;		// bail if not valid miso

	uint8_t rgb[3];               // RGB bytes received from the display
	uint8_t rgbIdx = 0;
	uint32_t txCount = w * h * 3; // number of bytes we will transmit to the display
	uint32_t rxCount = txCount;   // number of bytes we will receive back from the display

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMRD); // read from RAM

	// transmit a DUMMY byte before the color bytes
	_pkinetisk_spi->PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;

	// skip values returned by the queued up transfers and the current in-flight transfer
	uint32_t sr = _pkinetisk_spi->SR;
	uint8_t skipCount = ((sr >> 4) & 0xF) + ((sr >> 12) & 0xF) + 1;

	while (txCount || rxCount) {
		// transmit another byte if possible
		if (txCount && ((_pkinetisk_spi->SR & 0xF000) >> 12) < _pspin->sizeFIFO()) {
			txCount--;
			if (txCount) {
				_pkinetisk_spi->PUSHR = READ_PIXEL_PUSH_BYTE | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
			} else {
				_pkinetisk_spi->PUSHR = READ_PIXEL_PUSH_BYTE | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_EOQ;
			}
		}

		// receive another byte if possible, and either skip it or store the color
		if (rxCount && (_pkinetisk_spi->SR & 0xF0)) {
			rgb[rgbIdx] = _pkinetisk_spi->POPR;

			if (skipCount) {
				skipCount--;
			} else {
				rxCount--;
				rgbIdx++;
				if (rgbIdx == 3) {
					rgbIdx = 0;
					*pcolors++ = color565(rgb[0], rgb[1], rgb[2]);
				}
			}
		}
	}

	// wait for End of Queue
	while ((_pkinetisk_spi->SR & SPI_SR_EOQF) == 0) ;
	_pkinetisk_spi->SR = SPI_SR_EOQF;  // make sure it is clear
	endSPITransaction();

}
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
void ILI9341_t3n::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
	// Use our Origin. 
	x+=_originx;
	y+=_originy;
	//BUGBUG:: Should add some validation of X and Y

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			for (int i = 0 ;i < w; i++) {
				*pcolors++ = *pfbPixel++;
			}
			pfbPixel_row += _width;
		}
		return;	
	}
	#endif	

   if (_miso == 0xff) return;		// bail if not valid miso

	uint8_t rgb[3];               // RGB bytes received from the display
	uint8_t rgbIdx = 0;
	uint32_t txCount = w * h * 3; // number of bytes we will transmit to the display
	uint32_t rxCount = txCount;   // number of bytes we will receive back from the display

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMRD); // read from RAM


	// transmit a DUMMY byte before the color bytes
	writedata8_last(0);		// BUGBUG:: maybe fix this as this will wait until the byte fully transfers through.

	while (txCount || rxCount) {
		// transmit another byte if possible
		if (txCount && (_pimxrt_spi->SR & LPSPI_SR_TDF)) {
			txCount--;
			if (txCount) {
				_pimxrt_spi->TDR = 0;
			} else {
				maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7)); // remove the CONTINUE...
				while ((_pimxrt_spi->SR & LPSPI_SR_TDF) == 0) ;		// wait if queue was full
				_pimxrt_spi->TDR = 0;
			}
		}

		// receive another byte if possible, and either skip it or store the color
		if (rxCount && !(_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY)) {
			rgb[rgbIdx] = _pimxrt_spi->RDR;

			rxCount--;
			rgbIdx++;
			if (rgbIdx == 3) {
				rgbIdx = 0;
				*pcolors++ = color565(rgb[0], rgb[1], rgb[2]);
			}
		}
	}

	// We should have received everything so should be done
	endSPITransaction();
}

#else

// Teensy LC version
void ILI9341_t3n::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
	// Use our Origin. 
	x+=_originx;
	y+=_originy;
	//BUGBUG:: Should add some validation of X and Y

   if (_miso == 0xff) return;		// bail if not valid miso

	uint8_t rgb[3];               // RGB bytes received from the display
	uint8_t rgbIdx = 0;
	uint32_t txCount = w * h * 3; // number of bytes we will transmit to the display
	uint32_t rxCount = txCount;   // number of bytes we will receive back from the display

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMRD); // read from RAM

	// transmit a DUMMY byte before the color bytes
	writedata8_cont(0);

	// Wait until that one returns, Could do a little better and double buffer but this is easer for now.
	waitTransmitComplete();

	// Since double buffer setup lets try keeping read/write in sync
#define RRECT_TIMEOUT 0xffff	
#undef 	READ_PIXEL_PUSH_BYTE
#define READ_PIXEL_PUSH_BYTE 0 // try with zero to see... 	
	uint16_t timeout_countdown = RRECT_TIMEOUT;
	uint16_t dl_in;
	// Write out first byte:

	while (!(_pkinetisl_spi->S & SPI_S_SPTEF)) ; // Not worried that this can completely hang?
	_pkinetisl_spi->DL = READ_PIXEL_PUSH_BYTE;

	while (rxCount && timeout_countdown) {
		// Now wait until we can output something
		dl_in = 0xffff;
		if (rxCount > 1) {
			while (!(_pkinetisl_spi->S & SPI_S_SPTEF)) ; // Not worried that this can completely hang?
			if (_pkinetisl_spi->S & SPI_S_SPRF)
				dl_in = _pkinetisl_spi->DL;  
			_pkinetisl_spi->DL = READ_PIXEL_PUSH_BYTE;
		}

		// Now wait until there is a byte available to receive
		while ((dl_in != 0xffff) && !(_pkinetisl_spi->S & SPI_S_SPRF) && --timeout_countdown) ;
		if (timeout_countdown) {   // Make sure we did not get here because of timeout 
			rxCount--;
			rgb[rgbIdx] = (dl_in != 0xffff)? dl_in : _pkinetisl_spi->DL;
			rgbIdx++;
			if (rgbIdx == 3) {
				rgbIdx = 0;
				*pcolors++ = color565(rgb[0], rgb[1], rgb[2]);
			}
			timeout_countdown = timeout_countdown;
		}
	}

	// Debug code. 
/*	if (timeout_countdown == 0) {
		Serial.print("RRect Timeout ");
		Serial.println(rxCount, DEC);
	} */
	endSPITransaction();
}
#endif		

// Now lets see if we can writemultiple pixels
void ILI9341_t3n::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors)
{

	x+=_originx;
	y+=_originy;
	uint16_t x_clip_left = 0;  // How many entries at start of colors to skip at start of row
	uint16_t x_clip_right = 0;    // how many color entries to skip at end of row for clipping
	// Rectangular clipping 

	// See if the whole thing out of bounds...
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

	// In these cases you can not do simple clipping, as we need to synchronize the colors array with the
	// We can clip the height as when we get to the last visible we don't have to go any farther. 
	// also maybe starting y as we will advance the color array. 
 	if(y < _displayclipy1) {
 		int dy = (_displayclipy1 - y);
 		h -= dy; 
 		pcolors += (dy*w); // Advance color array to 
 		y = _displayclipy1; 	
 	}

	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

	// For X see how many items in color array to skip at start of row and likewise end of row 
	if(x < _displayclipx1) {
		x_clip_left = _displayclipx1-x; 
		w -= x_clip_left; 
		x = _displayclipx1; 	
	}
	if((x + w - 1) >= _displayclipx2) {
		x_clip_right = w;
		w = _displayclipx2  - x;
		x_clip_right -= w; 
	} 

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			pcolors += x_clip_left;
			for (int i = 0 ;i < w; i++) {
				*pfbPixel++ = *pcolors++;
			}
			pfbPixel_row += _width;
			pcolors += x_clip_right;

		}
		return;	
	}
	#endif

   	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		pcolors += x_clip_left;
		for(x=w; x>1; x--) {
			writedata16_cont(*pcolors++);
		}
		writedata16_last(*pcolors++);
		pcolors += x_clip_right;
	}
	endSPITransaction();
}

// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
//					bitmap data in array at pixels, one byte per pixel
//					color palette data in array at palette
void ILI9341_t3n::writeRect8BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
	//Serial.printf("\nWR8: %d %d %d %d %x\n", x, y, w, h, (uint32_t)pixels);
	x+=_originx;
	y+=_originy;

	uint16_t x_clip_left = 0;  // How many entries at start of colors to skip at start of row
	uint16_t x_clip_right = 0;    // how many color entries to skip at end of row for clipping
	// Rectangular clipping 

	// See if the whole thing out of bounds...
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

	// In these cases you can not do simple clipping, as we need to synchronize the colors array with the
	// We can clip the height as when we get to the last visible we don't have to go any farther. 
	// also maybe starting y as we will advance the color array. 
 	if(y < _displayclipy1) {
 		int dy = (_displayclipy1 - y);
 		h -= dy; 
 		pixels += (dy*w); // Advance color array to 
 		y = _displayclipy1; 	
 	}

	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

	// For X see how many items in color array to skip at start of row and likewise end of row 
	if(x < _displayclipx1) {
		x_clip_left = _displayclipx1-x; 
		w -= x_clip_left; 
		x = _displayclipx1; 	
	}
	if((x + w - 1) >= _displayclipx2) {
		x_clip_right = w;
		w = _displayclipx2  - x;
		x_clip_right -= w; 
	} 
	//Serial.printf("WR8C: %d %d %d %d %x- %d %d\n", x, y, w, h, (uint32_t)pixels, x_clip_right, x_clip_left);
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			pixels += x_clip_left;
			uint16_t * pfbPixel = pfbPixel_row;
			for (int i = 0 ;i < w; i++) {
				*pfbPixel++ = palette[*pixels++];
			}
			pixels += x_clip_right;
			pfbPixel_row += _width;
		}
		return;	
	}
	#endif

	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		pixels += x_clip_left;
		//Serial.printf("%x: ", (uint32_t)pixels);
		for(x=w; x>1; x--) {
			//Serial.print(*pixels, DEC);
			writedata16_cont(palette[*pixels++]);
		}
		//Serial.println(*pixels, DEC);
		writedata16_last(palette[*pixels++]);
		pixels += x_clip_right;
	}
	endSPITransaction();
}

// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per pixel
//					color palette data in array at palette
//					width must be at least 2 pixels
void ILI9341_t3n::writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
	// Simply call through our helper
	writeRectNBPP(x, y, w, h,  4, pixels, palette );
}

// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per pixel
//					color palette data in array at palette
//					width must be at least 4 pixels
void ILI9341_t3n::writeRect2BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
	// Simply call through our helper
	writeRectNBPP(x, y, w, h,  2, pixels, palette );

}

///============================================================================
// writeRect1BPP - 	write 1 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per pixel
//					color palette data in array at palette
//					width must be at least 8 pixels
void ILI9341_t3n::writeRect1BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
	// Simply call through our helper
	writeRectNBPP(x, y, w, h,  1, pixels, palette );
}



///============================================================================
// writeRectNBPP - 	write N(1, 2, 4, 8) bit per pixel paletted bitmap
//					bitmap data in array at pixels
//  Currently writeRect1BPP, writeRect2BPP, writeRect4BPP use this to do all of the work. 
void ILI9341_t3n::writeRectNBPP(int16_t x, int16_t y, int16_t w, int16_t h,  uint8_t bits_per_pixel, 
		const uint8_t *pixels, const uint16_t * palette )
{
	//Serial.printf("\nWR8: %d %d %d %d %x\n", x, y, w, h, (uint32_t)pixels);
	x+=_originx;
	y+=_originy;
	uint8_t pixels_per_byte = 8/ bits_per_pixel;
	uint16_t count_of_bytes_per_row = (w + pixels_per_byte -1)/pixels_per_byte;		// Round up to handle non multiples
	uint8_t row_shift_init = 8 - bits_per_pixel;				// We shift down 6 bits by default 
	uint8_t pixel_bit_mask = (1 << bits_per_pixel) - 1; 		// get mask to use below
	// Rectangular clipping 

	// See if the whole thing out of bounds...
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

	// In these cases you can not do simple clipping, as we need to synchronize the colors array with the
	// We can clip the height as when we get to the last visible we don't have to go any farther. 
	// also maybe starting y as we will advance the color array. 
	// Again assume multiple of 8 for width
 	if(y < _displayclipy1) {
 		int dy = (_displayclipy1 - y);
 		h -= dy; 
 		pixels += dy * count_of_bytes_per_row;
 		y = _displayclipy1; 	
 	}

	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

	// For X see how many items in color array to skip at start of row and likewise end of row 
	if(x < _displayclipx1) {
		uint16_t x_clip_left = _displayclipx1-x; 
		w -= x_clip_left; 
		x = _displayclipx1; 
		// Now lets update pixels to the rigth offset and mask
		uint8_t x_clip_left_bytes_incr = x_clip_left / pixels_per_byte;
		pixels += x_clip_left_bytes_incr;
		row_shift_init = 8 - (x_clip_left - (x_clip_left_bytes_incr * pixels_per_byte) + 1) * bits_per_pixel; 	
	}

	if((x + w - 1) >= _displayclipx2) {
		w = _displayclipx2  - x;
	} 

	const uint8_t * pixels_row_start = pixels;  // remember our starting position offset into row

	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			pixels = pixels_row_start;				// setup for this row
			uint8_t pixel_shift = row_shift_init;			// Setup mask

			for (int i = 0 ; i < w; i++) {
				*pfbPixel++ = palette[((*pixels)>>pixel_shift) & pixel_bit_mask];
				if (!pixel_shift) {
					pixel_shift = 8 - bits_per_pixel;	//setup next mask
					pixels++;
				} else {
					pixel_shift -= bits_per_pixel;
				}
			}
			pfbPixel_row += _width;
			pixels_row_start += count_of_bytes_per_row;
		}
		return;	

	}
	#endif

	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for (;h>0; h--) {
		pixels = pixels_row_start;				// setup for this row
		uint8_t pixel_shift = row_shift_init;			// Setup mask

		for (int i = 0 ;i < w; i++) {
			writedata16_cont(palette[((*pixels)>>pixel_shift) & pixel_bit_mask]);
			if (!pixel_shift) {
				pixel_shift = 8 - bits_per_pixel;	//setup next mask
				pixels++;
			} else {
				pixel_shift -= bits_per_pixel;
			}
		}
		pixels_row_start += count_of_bytes_per_row;
	}
	writecommand_last(ILI9341_NOP);
	endSPITransaction();
}

static const uint8_t init_commands[] = {
	4, 0xEF, 0x03, 0x80, 0x02,
	4, 0xCF, 0x00, 0XC1, 0X30,
	5, 0xED, 0x64, 0x03, 0X12, 0X81,
	4, 0xE8, 0x85, 0x00, 0x78,
	6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
	2, 0xF7, 0x20,
	3, 0xEA, 0x00, 0x00,
	2, ILI9341_PWCTR1, 0x23, // Power control
	2, ILI9341_PWCTR2, 0x10, // Power control
	3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
	2, ILI9341_VMCTR2, 0x86, // VCM control2
	2, ILI9341_MADCTL, 0x48, // Memory Access Control
	2, ILI9341_PIXFMT, 0x55,
	3, ILI9341_FRMCTR1, 0x00, 0x18,
	4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
	2, 0xF2, 0x00, // Gamma Function Disable
	2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
	16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
		0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
	16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
		0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
	0
};

void ILI9341_t3n::begin(void)
{
    // verify SPI pins are valid;
	// allow user to say use current ones...
#ifdef DEBUG_ASYNC_UPDATE
	//Serial.printf("????? _dmasettings[0] %x %x\n", (uint32_t)&_dmasettings[0], (uint32_t)_dmasettings[0].TCD); Serial.flush();
#endif
	if ((_mosi != 255) || (_miso != 255) || (_sclk != 255)) {
		if (!(_pspin->pinIsMOSI(_mosi)) || !(_pspin->pinIsMISO(_miso)) || !(_pspin->pinIsSCK(_sclk))) {
			#ifdef SPIN1_OBJECT_CREATED			
			if (SPIN1.pinIsMOSI(_mosi) && SPIN1.pinIsMISO(_miso) && SPIN1.pinIsSCK(_sclk)) {
				_pspin = &SPIN1;
#ifdef KINETISK
				_pkinetisk_spi = &_pspin->port();
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
				_pimxrt_spi = &_pspin->port();
#else
				_pkinetisl_spi = &_pspin->port();
#endif				
				//Serial.println("ILI9341_t3n: SPIN1 automatically selected");
			} else {
				#ifdef SPIN2_OBJECT_CREATED			
				if (SPIN2.pinIsMOSI(_mosi) && SPIN2.pinIsMISO(_miso) && SPIN2.pinIsSCK(_sclk)) {
					_pspin = &SPIN2;
					_pkinetisk_spi = &_pspin->port();
					//Serial.println("ILI9341_t3n: SPIN2 automatically selected");
				} else {
				#endif
			#endif
					uint8_t mosi_sck_bad = false;
					Serial.print("ILI9341_t3n: Error not valid SPI pins:");
					if(!(_pspin->pinIsMOSI(_mosi)))  {
						Serial.print(" MOSI");
						mosi_sck_bad = true;
					}
					if (!_pspin->pinIsSCK(_sclk)) {
						Serial.print(" SCLK");
						mosi_sck_bad = true;
					}

					// Maybe allow us to limp with only MISO bad
					if(!(_pspin->pinIsMISO(_miso))) {
						Serial.print(" MISO");
						_miso = 0xff;	// set miso to 255 as flag it is bad
					}
					Serial.println();
					
					if (mosi_sck_bad) {
	    				return; // not valid pins...
					}
				#ifdef SPIN2_OBJECT_CREATED			
	    		}
	    		#endif
			#ifdef SPIN1_OBJECT_CREATED			
			}
			#endif

		}
		//Serial.printf("MOSI:%d MISO:%d SCK:%d\n\r", _mosi, _miso, _sclk);			
        _pspin->setMOSI(_mosi);
        if (_miso != 0xff) _pspin->setMISO(_miso);
        _pspin->setSCK(_sclk);
	}

	_pspin->begin();
#ifdef KINETISK
	if (_pspin->pinIsChipSelect(_cs, _dc)) {
		pcs_data = _pspin->setCS(_cs);
		pcs_command = pcs_data | _pspin->setCS(_dc);
	} else {
		// See if at least DC is on chipselect pin, if so try to limp along...
		if (_pspin->pinIsChipSelect(_dc)) {
			pcs_data = 0;
			pcs_command = pcs_data | _pspin->setCS(_dc);
			pinMode(_cs, OUTPUT);
			_csport    = portOutputRegister(digitalPinToPort(_cs));
			_cspinmask = digitalPinToBitMask(_cs);
			*_csport |= _cspinmask;
		} else {
			pcs_data = 0;
			pcs_command = 0;
  			Serial.println("ILI9341_t3n: Error not DC is not valid hardware CS pin");
			return;
		}
	}
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
	_csport = portOutputRegister(_cs);
	_cspinmask = digitalPinToBitMask(_cs);
	pinMode(_cs, OUTPUT);	
	DIRECT_WRITE_HIGH(_csport, _cspinmask);
	_spi_tcr_current = _pimxrt_spi->TCR; // get the current TCR value 

	// TODO:  Need to setup DC to actually work.
	if (_pspin->pinIsChipSelect(_dc)) {
	 	_pspin->setCS(_dc);
	 	_dcport = 0;
	 	_dcpinmask = 0;
	} else {
		//Serial.println("ILI9341_t3n: Error not DC is not valid hardware CS pin");
		_dcport = portOutputRegister(_dc);
		_dcpinmask = digitalPinToBitMask(_dc);
		pinMode(_dc, OUTPUT);	
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
	}
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));

#else
	// TLC
	pcs_data = 0;
	pcs_command = 0;
	pinMode(_cs, OUTPUT);
	_csport    = portOutputRegister(digitalPinToPort(_cs));
	_cspinmask = digitalPinToBitMask(_cs);
	*_csport |= _cspinmask;
	pinMode(_dc, OUTPUT);
	_dcport    = portOutputRegister(digitalPinToPort(_dc));
	_dcpinmask = digitalPinToBitMask(_dc);
	*_dcport |= _dcpinmask;
	_dcpinAsserted = 0;
#endif	

	// toggle RST low to reset
	if (_rst < 255) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(5);
		digitalWrite(_rst, LOW);
		delay(20);
		digitalWrite(_rst, HIGH);
		delay(150);
	}
/*
	uint8_t x = readcommand8(ILI9341_RDMODE);
	Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDMADCTL);
	Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDPIXFMT);
	Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDIMGFMT);
	Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDSELFDIAG);
	Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
*/	
	beginSPITransaction();
	const uint8_t *addr = init_commands;
	while (1) {
		uint8_t count = *addr++;
		if (count-- == 0) break;
		writecommand_cont(*addr++);
		while (count-- > 0) {
			writedata8_cont(*addr++);
		}
	}
	writecommand_last(ILI9341_SLPOUT);    // Exit Sleep
	endSPITransaction();
	delay(120); 		
	beginSPITransaction();
	writecommand_last(ILI9341_DISPON);    // Display on
	endSPITransaction();

#ifdef DEBUG_ASYNC_LEDS
	pinMode(DEBUG_PIN_1, OUTPUT);
	pinMode(DEBUG_PIN_2, OUTPUT);
	pinMode(DEBUG_PIN_3, OUTPUT);
#endif
}




/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

//#include "glcdfont.c"
extern "C" const unsigned char glcdfont[];

// Draw a circle outline
void ILI9341_t3n::drawCircle(int16_t x0, int16_t y0, int16_t r,
    uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void ILI9341_t3n::drawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void ILI9341_t3n::fillCircle(int16_t x0, int16_t y0, int16_t r,
			      uint16_t color) {
  drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void ILI9341_t3n::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}


// Bresenham's algorithm - thx wikpedia
void ILI9341_t3n::drawLine(int16_t x0, int16_t y0,
	int16_t x1, int16_t y1, uint16_t color)
{
	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		} else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		} else {
			drawPixel(x0, y0, color);
		}
		return;
	} else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		} else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	beginSPITransaction();
	int16_t xbegin = x0;
	if (steep) {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					VLine(y0, xbegin, len + 1, color);
				} else {
					Pixel(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			VLine(y0, xbegin, x0 - xbegin, color);
		}

	} else {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					HLine(xbegin, y0, len + 1, color);
				} else {
					Pixel(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			HLine(xbegin, y0, x0 - xbegin, color);
		}
	}
	writecommand_last(ILI9341_NOP);
	endSPITransaction();
}

// Draw a rectangle
void ILI9341_t3n::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	#ifdef ENABLE_ILI9341_FRAMEBUFFER
	if (_use_fbtft) {
		drawFastHLine(x, y, w, color);
		drawFastHLine(x, y+h-1, w, color);
		drawFastVLine(x, y, h, color);
		drawFastVLine(x+w-1, y, h, color);
	} else 
	#endif
	{
	    beginSPITransaction();
		HLine(x, y, w, color);
		HLine(x, y+h-1, w, color);
		VLine(x, y, h, color);
		VLine(x+w-1, y, h, color);
		writecommand_last(ILI9341_NOP);
		endSPITransaction();
	}
}

// Draw a rounded rectangle
void ILI9341_t3n::drawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color) {
  // smarter version
  drawFastHLine(x+r  , y    , w-2*r, color); // Top
  drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  drawFastVLine(x    , y+r  , h-2*r, color); // Left
  drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void ILI9341_t3n::fillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a triangle
void ILI9341_t3n::drawTriangle(int16_t x0, int16_t y0,
				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void ILI9341_t3n::fillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int32_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }
}

void ILI9341_t3n::drawBitmap(int16_t x, int16_t y,
			      const uint8_t *bitmap, int16_t w, int16_t h,
			      uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
	drawPixel(x+i, y+j, color);
      }
    }
  }
}

size_t ILI9341_t3n::write(uint8_t c)
{
	if (font) {
		if (c == '\n') {
			cursor_y += font->line_space;
			if(scrollEnable && isWritingScrollArea){
				cursor_x  = scroll_x;
			}else{
				cursor_x  = 0;
			}
		} else {
			drawFontChar(c);
		}
	} else {
		if (c == '\n') {
			cursor_y += textsize*8;
			if(scrollEnable && isWritingScrollArea){
				cursor_x  = scroll_x;
			}else{
				cursor_x  = 0;
			}
		} else if (c == '\r') {
			// skip em
		} else {
			if(scrollEnable && isWritingScrollArea && (cursor_y > (scroll_y+scroll_height - textsize*8))){
				scrollTextArea(textsize*8);
				cursor_y -= textsize*8;
				cursor_x = scroll_x;
			}
			drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
			cursor_x += textsize*6;
			if(wrap && scrollEnable && isWritingScrollArea && (cursor_x > (scroll_x+scroll_width - textsize*6))){
				cursor_y += textsize*8;
				cursor_x = scroll_x;
			}
			else if (wrap && (cursor_x > (_width - textsize*6))) {
				cursor_y += textsize*8;
				cursor_x = 0;
			}
		}
	}
	return 1;
}

// Draw a character
void ILI9341_t3n::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t fgcolor, uint16_t bgcolor, uint8_t size)
{
	if((x >= _width)            || // Clip right
	   (y >= _height)           || // Clip bottom
	   ((x + 6 * size - 1) < 0) || // Clip left  TODO: is this correct?
	   ((y + 8 * size - 1) < 0))   // Clip top   TODO: is this correct?
		return;

	if (fgcolor == bgcolor) {
		// This transparent approach is only about 20% faster
		if (size == 1) {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (glcdfont[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						drawFastHLine(x + xoff, y + yoff, 5, fgcolor);
						break;
					} else if (line == 0x1E) {
						drawFastHLine(x + xoff, y + yoff, 4, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						drawFastHLine(x + xoff, y + yoff, 3, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						drawFastHLine(x + xoff, y + yoff, 2, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						drawPixel(x + xoff, y + yoff, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		} else {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (glcdfont[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						fillRect(x + xoff * size, y + yoff * size,
							5 * size, size, fgcolor);
						break;
					} else if (line == 0x1E) {
						fillRect(x + xoff * size, y + yoff * size,
							4 * size, size, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						fillRect(x + xoff * size, y + yoff * size,
							3 * size, size, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						fillRect(x + xoff * size, y + yoff * size,
							2 * size, size, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						fillRect(x + xoff * size, y + yoff * size,
							size, size, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		}
	} else {
		// This solid background approach is about 5 time faster
		uint8_t xc, yc;
		uint8_t xr, yr;
		uint8_t mask = 0x01;
		uint16_t color;

		// We need to offset by the origin.
		x+=_originx;
		y+=_originy;
		int16_t x_char_start = x;  // remember our X where we start outputting...

		if((x >= _displayclipx2)            || // Clip right
			 (y >= _displayclipy2)           || // Clip bottom
			 ((x + 6 * size - 1) < _displayclipx1) || // Clip left  TODO: this is not correct
			 ((y + 8 * size - 1) < _displayclipy1))   // Clip top   TODO: this is not correct
			return;


		#ifdef ENABLE_ILI9341_FRAMEBUFFER
		if (_use_fbtft) {

			uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
			for (yc=0; (yc < 8) && (y < _displayclipy2); yc++) {
				for (yr=0; (yr < size) && (y < _displayclipy2); yr++) {
					x = x_char_start; 		// get our first x position...
					if (y >= _displayclipy1) {
						uint16_t * pfbPixel = pfbPixel_row;
						for (xc=0; xc < 5; xc++) {
							if (glcdfont[c * 5 + xc] & mask) {
								color = fgcolor;
							} else {
								color = bgcolor;
							}
							for (xr=0; xr < size; xr++) {
								if ((x >= _displayclipx1) && (x < _displayclipx2)) {
									*pfbPixel = color;
								}
								pfbPixel++;
								x++;
							}
						}
						for (xr=0; xr < size; xr++) {
							if ((x >= _displayclipx1) && (x < _displayclipx2)) {
								*pfbPixel = bgcolor;
							}
							pfbPixel++;
							x++;
						}
					}
					pfbPixel_row += _width; // setup pointer to 
					y++;
				}
				mask = mask << 1;
			}

		} else 
		#endif
		{
			// need to build actual pixel rectangle we will output into.
			int16_t y_char_top = y;	// remember the y
			int16_t w =  6 * size;
			int16_t h = 8 * size;

			if(x < _displayclipx1) {	w -= (_displayclipx1-x); x = _displayclipx1; 	}
			if((x + w - 1) >= _displayclipx2)  w = _displayclipx2  - x;
			if(y < _displayclipy1) {	h -= (_displayclipy1 - y); y = _displayclipy1; 	}
			if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

			beginSPITransaction();
			setAddr(x, y, x + w -1, y + h - 1);

			y = y_char_top;	// restore the actual y.
			writecommand_cont(ILI9341_RAMWR);
			for (yc=0; (yc < 8) && (y < _displayclipy2); yc++) {
				for (yr=0; (yr < size) && (y < _displayclipy2); yr++) {
					x = x_char_start; 		// get our first x position...
					if (y >= _displayclipy1) {
						for (xc=0; xc < 5; xc++) {
							if (glcdfont[c * 5 + xc] & mask) {
								color = fgcolor;
							} else {
								color = bgcolor;
							}
							for (xr=0; xr < size; xr++) {
								if ((x >= _displayclipx1) && (x < _displayclipx2)) {
									writedata16_cont(color);
								}
								x++;
							}
						}
						for (xr=0; xr < size; xr++) {
							if ((x >= _displayclipx1) && (x < _displayclipx2)) {
								writedata16_cont(bgcolor);
							}
							x++;
						}
					}
					y++;
				}
				mask = mask << 1;
			}
			writecommand_last(ILI9341_NOP);
			endSPITransaction();
		}
	}
}

void ILI9341_t3n::setFont(const ILI9341_t3_font_t &f) {
	font = &f;
	fontbpp = 1;
	// Calculate additional metrics for Anti-Aliased font support (BDF extn v2.3)
	if (font && font->version==23){
		fontbpp = (font->reserved & 0b000011)+1;
		fontbppindex = (fontbpp >> 2)+1;
		fontbppmask = (1 << (fontbppindex+1))-1;
		fontppb = 8/fontbpp;
		fontalphamx = 31/((1<<fontbpp)-1);
		// Ensure text and bg color are different. Note: use setTextColor to set actual bg color
		if (textcolor == textbgcolor) textbgcolor = (textcolor==0x0000)?0xFFFF:0x0000;
	}
}

static uint32_t fetchbit(const uint8_t *p, uint32_t index)
{
	if (p[index >> 3] & (1 << (7 - (index & 7)))) return 1;
	return 0;
}

static uint32_t fetchbits_unsigned(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = 0;
	do {
		uint8_t b = p[index >> 3];
		uint32_t avail = 8 - (index & 7);
		if (avail <= required) {
			val <<= avail;
			val |= b & ((1 << avail) - 1);
			index += avail;
			required -= avail;
		} else {
			b >>= avail - required;
			val <<= required;
			val |= b & ((1 << required) - 1);
			break;
		}
	} while (required);
	return val;
}

static uint32_t fetchbits_signed(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = fetchbits_unsigned(p, index, required);
	if (val & (1 << (required - 1))) {
		return (int32_t)val - (1 << required);
	}
	return (int32_t)val;
}

uint32_t ILI9341_t3n::fetchpixel(const uint8_t *p, uint32_t index, uint32_t x)
{
	// The byte
	uint8_t b = p[index >> 3];
	// Shift to LSB position and mask to get value
	uint8_t s = ((fontppb-(x % fontppb)-1)*fontbpp);
	// Mask and return
	return (b >> s) & fontbppmask;
}

void ILI9341_t3n::drawFontChar(unsigned int c)
{
	uint32_t bitoffset;
	const uint8_t *data;

	//Serial.printf("drawFontChar(%c) %d\n", c, c);

	if (c >= font->index1_first && c <= font->index1_last) {
		bitoffset = c - font->index1_first;
		bitoffset *= font->bits_index;
	} else if (c >= font->index2_first && c <= font->index2_last) {
		bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
		bitoffset *= font->bits_index;
	} else if (font->unicode) {
		return; // TODO: implement sparse unicode
	} else {
		return;
	}
	//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
	data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

	uint32_t encoding = fetchbits_unsigned(data, 0, 3);
	if (encoding != 0) return;
	uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
	bitoffset = font->bits_width + 3;
	uint32_t height = fetchbits_unsigned(data, bitoffset, font->bits_height);
	bitoffset += font->bits_height;
	//Serial.printf("  size =   %d,%d\n", width, height);
	//Serial.printf("  line space = %d\n", font->line_space);

	int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
	bitoffset += font->bits_xoffset;
	int32_t yoffset = fetchbits_signed(data, bitoffset, font->bits_yoffset);
	bitoffset += font->bits_yoffset;
	//Serial.printf("  offset = %d,%d\n", xoffset, yoffset);

	uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
	bitoffset += font->bits_delta;
	//Serial.printf("  delta =  %d\n", delta);

	//Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);

	 //horizontally, we draw every pixel, or none at all
	if (cursor_x < 0) cursor_x = 0;
	int32_t origin_x = cursor_x + xoffset;
	if (origin_x < 0) {
		cursor_x -= xoffset;
		origin_x = 0;
	}
	if (origin_x + (int)width > _width) {
		if (!wrap) return;
		origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = 0;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
	}
	if(wrap && scrollEnable && isWritingScrollArea && ((origin_x + (int)width) > (scroll_x+scroll_width))){
    	origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = scroll_x;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
    }
	
	if(scrollEnable && isWritingScrollArea && (cursor_y > (scroll_y+scroll_height - font->cap_height))){
		scrollTextArea(font->line_space);
		cursor_y -= font->line_space;
		cursor_x = scroll_x;
	} 
	if (cursor_y >= _height) return;

	// vertically, the top and/or bottom can be clipped
	int32_t origin_y = cursor_y + font->cap_height - height - yoffset;
	//Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

	// TODO: compute top skip and number of lines
	int32_t linecount = height;
	//uint32_t loopcount = 0;
	int32_t y = origin_y;
	bool opaque = (textbgcolor != textcolor);

	// Going to try a fast Opaque method which works similar to drawChar, which is near the speed of writerect
	if (!opaque) {

		// Anti-alias support
		if (fontbpp>1){
			// This branch should, in most cases, never happen. This is because if an
			// anti-aliased font is being used, textcolor and textbgcolor should always
			// be different. Even though an anti-alised font is being used, pixels in this
			// case will all be solid because pixels are rendered on same colour as themselves!
			// This won't look very good.
			bitoffset = ((bitoffset + 7) & (-8)); // byte-boundary
			uint32_t xp = 0;
			uint8_t halfalpha = 1<<(fontbpp-1);
			while (linecount) {
				uint32_t x = 0;
				while(x<width) {
					// One pixel at a time, either on (if alpha > 0.5) or off
					if (fetchpixel(data, bitoffset, xp)>=halfalpha){
						Pixel(origin_x + x,y,textcolor);
					}
					bitoffset += fontbpp;
					x++;
					xp++;
				}
				y++;
				linecount--;
			}

		}
		// Soild pixels
		else{

			while (linecount > 0) {
				//Serial.printf("    linecount = %d\n", linecount);
				uint32_t n = 1;
				if (fetchbit(data, bitoffset++) != 0) {
					n = fetchbits_unsigned(data, bitoffset, 3) + 2;
					bitoffset += 3;
				}
				uint32_t x = 0;
				do {
					int32_t xsize = width - x;
					if (xsize > 32) xsize = 32;
					uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
					//Serial.printf("    multi line %d %d %x\n", n, x, bits);
					drawFontBits(opaque, bits, xsize, origin_x + x, y, n);
					bitoffset += xsize;
					x += xsize;
				} while (x < width);


				y += n;
				linecount -= n;
				//if (++loopcount > 100) {
					//Serial.println("     abort draw loop");
					//break;
				//}
			}
		} // 1bpp
	}

	// opaque
	else {
		// Now opaque mode... 
		// Now write out background color for the number of rows above the above the character
		// figure out bounding rectangle... 
		// In this mode we need to update to use the offset and bounding rectangles as we are doing it it direct.
		// also update the Origin 
		int cursor_x_origin = cursor_x + _originx;
		int cursor_y_origin = cursor_y + _originy;
		origin_x += _originx;
		origin_y += _originy;

		int start_x = (origin_x < cursor_x_origin) ? origin_x : cursor_x_origin; 	
		if (start_x < 0) start_x = 0;
		
		int start_y = (origin_y < cursor_y_origin) ? origin_y : cursor_y_origin; 
		if (start_y < 0) start_y = 0;
		int end_x = cursor_x_origin + delta; 
		if ((origin_x + (int)width) > end_x)
			end_x = origin_x + (int)width;
		if (end_x >= _displayclipx2)  end_x = _displayclipx2;	
		int end_y = cursor_y_origin + font->line_space; 
		if ((origin_y + (int)height) > end_y)
			end_y = origin_y + (int)height;
		if (end_y >= _displayclipy2) end_y = _displayclipy2;	
		end_x--;	// setup to last one we draw
		end_y--;
		int start_x_min = (start_x >= _displayclipx1) ? start_x : _displayclipx1;
		int start_y_min = (start_y >= _displayclipy1) ? start_y : _displayclipy1;

		// See if anything is in the display area.
		if((end_x < _displayclipx1) ||(start_x >= _displayclipx2) || (end_y < _displayclipy1) || (start_y >= _displayclipy2)) {
			cursor_x += delta;	// could use goto or another indent level...
		 	return;
		}
/*
		Serial.printf("drawFontChar(%c) %d\n", c, c);
		Serial.printf("  size =   %d,%d\n", width, height);
		Serial.printf("  line space = %d\n", font->line_space);
		Serial.printf("  offset = %d,%d\n", xoffset, yoffset);
		Serial.printf("  delta =  %d\n", delta);
		Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);
		Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

		Serial.printf("  Bounding: (%d, %d)-(%d, %d)\n", start_x, start_y, end_x, end_y);
		Serial.printf("  mins (%d %d),\n", start_x_min, start_y_min);
*/
		#ifdef ENABLE_ILI9341_FRAMEBUFFER
		if (_use_fbtft) {
			uint16_t * pfbPixel_row = &_pfbtft[ start_y*_width + start_x];
			uint16_t * pfbPixel;
			int screen_y = start_y;
			int screen_x;

			// Clear above character
			while (screen_y < origin_y) {
				pfbPixel = pfbPixel_row;
				// only output if this line is within the clipping region.
				if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
					for (screen_x = start_x; screen_x <= end_x; screen_x++) {
						if (screen_x >= _displayclipx1) {
							*pfbPixel = textbgcolor;
						}
						pfbPixel++;
					}
				}
				screen_y++;
				pfbPixel_row += _width;
			}

			// Anti-aliased font
			if (fontbpp>1){
				screen_y = origin_y;
				bitoffset = ((bitoffset + 7) & (-8)); // byte-boundary
				uint32_t xp = 0;
				uint32_t glyphend_x = origin_x+width;
				while (linecount) {
					pfbPixel = pfbPixel_row;
					uint32_t screen_x = start_x;
					while(screen_x<=end_x) {
						// XXX: I'm sure clipping could be done way more efficiently than just chekcing every single pixel, but let's just get this going
						if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2) && (screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// Clear before or after pixel
							if ((screen_x<origin_x) || (screen_x>=glyphend_x)){
								*pfbPixel = textbgcolor;
							}
							// Draw alpha-blended character
							else{
								uint8_t alpha = fetchpixel(data, bitoffset, xp);
								*pfbPixel = alphaBlendRGB565Premultiplied( textcolorPrexpanded, textbgcolorPrexpanded, (uint8_t)(alpha * fontalphamx) );
								bitoffset += fontbpp;
								xp++;
							}
						} // clip
						screen_x++;
						pfbPixel++;
					}
					pfbPixel_row += _width;
					screen_y++;
					linecount--;
				}

			} // anti-aliased

			// 1bpp solid font
			else{

				// Now lets process each of the data lines (draw character)
				screen_y = origin_y;
				while (linecount > 0) {
					//Serial.printf("    linecount = %d\n", linecount);
					uint32_t b = fetchbit(data, bitoffset++);
					uint32_t n;
					if (b == 0) {
						//Serial.println("Single");
						n = 1;
					} else {
						//Serial.println("Multi");
						n = fetchbits_unsigned(data, bitoffset, 3) + 2;
						bitoffset += 3;
					}
					uint32_t bitoffset_row_start = bitoffset;
					while (n--) {
						pfbPixel = pfbPixel_row;

						// Clear to left
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							bitoffset = bitoffset_row_start;	// we will work through these bits maybe multiple times
							for (screen_x = start_x; screen_x < origin_x; screen_x++) {
								if (screen_x >= _displayclipx1) {
									*pfbPixel = textbgcolor;
								} // make sure not clipped
								pfbPixel++;
							}
						}

						// Pixel bits
						screen_x = origin_x;
						uint32_t x = 0;
						do {
							uint32_t xsize = width - x;
							if (xsize > 32) xsize = 32;
							uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
							uint32_t bit_mask = 1 << (xsize-1);
							//Serial.printf(" %d %d %x %x\n", x, xsize, bits, bit_mask);
							if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
								while (bit_mask && (screen_x <= end_x)) {
									if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2)) {
										*pfbPixel = (bits & bit_mask) ? textcolor : textbgcolor;
									}
									pfbPixel++;	
									bit_mask = bit_mask >> 1;
									screen_x++;	// increment our pixel position. 
								}
							}
								bitoffset += xsize;
							x += xsize;
						} while (x < width);

						// Clear to right
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// output bg color and right hand side
							while (screen_x++ <= end_x) {
								*pfbPixel++ = textbgcolor;
							}
						}			 
			 			screen_y++;
						pfbPixel_row += _width;
						linecount--;
					}
				}

			} // 1bpp

			// clear below character
	 		while (screen_y++ <= end_y) {
				if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
					pfbPixel = pfbPixel_row;
					for (screen_x = start_x; screen_x <= end_x; screen_x++) {
						if (screen_x >= _displayclipx1) {
							*pfbPixel = textbgcolor;
						}
						pfbPixel++;
					}
				}
				pfbPixel_row += _width;
			}

		} else 
		#endif
		{

			beginSPITransaction();
			//Serial.printf("SetAddr %d %d %d %d\n", start_x_min, start_y_min, end_x, end_y);
			// output rectangle we are updating... We have already clipped end_x/y, but not yet start_x/y
			setAddr( start_x, start_y_min, end_x, end_y);
			writecommand_cont(ILI9341_RAMWR);
			int screen_y = start_y_min;
			int screen_x;

			// Clear above character
			while (screen_y < origin_y) {
				for (screen_x = start_x_min; screen_x <= end_x; screen_x++) {
					writedata16_cont(textbgcolor);
				}
				screen_y++;
			}

			// Anti-aliased font
			if (fontbpp>1){
				screen_y = origin_y;
				bitoffset = ((bitoffset + 7) & (-8)); // byte-boundary
				uint32_t glyphend_x = origin_x+width;
				uint32_t xp = 0;
				while (linecount) {
					uint32_t screen_x = start_x;
					while(screen_x<=end_x) {
						// XXX: I'm sure clipping could be done way more efficiently than just chekcing every single pixel, but let's just get this going
						if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2) && (screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// Clear before or after pixel
							if ((screen_x<origin_x) || (screen_x>=glyphend_x)){
								writedata16_cont(textbgcolor);
							}
							// Draw alpha-blended character
							else{
								uint8_t alpha = fetchpixel(data, bitoffset, xp);
								writedata16_cont( alphaBlendRGB565Premultiplied( textcolorPrexpanded, textbgcolorPrexpanded, (uint8_t)(alpha * fontalphamx) ) );
								bitoffset += fontbpp;
								xp++;
							}
						} // clip
						screen_x++;
					}
					screen_y++;
					linecount--;
				}

			} // anti-aliased

			// 1bpp
			else{

				// Now lets process each of the data lines. 
				screen_y = origin_y;
				while (linecount > 0) {
					//Serial.printf("    linecount = %d\n", linecount);
					uint32_t b = fetchbit(data, bitoffset++);
					uint32_t n;
					if (b == 0) {
						//Serial.println("    Single");
						n = 1;
					} else {
						//Serial.println("    Multi");
						n = fetchbits_unsigned(data, bitoffset, 3) + 2;
						bitoffset += 3;
					}
					uint32_t bitoffset_row_start = bitoffset;
					while (n--) {
						// do some clipping here. 
						bitoffset = bitoffset_row_start;	// we will work through these bits maybe multiple times
						// We need to handle case where some of the bits may not be visible, but we still need to
						// read through them
						//Serial.printf("y:%d  %d %d %d %d\n", screen_y, start_x, origin_x, _displayclipx1, _displayclipx2);
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							for (screen_x = start_x; screen_x < origin_x; screen_x++) {
								if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2)) {
									//Serial.write('-');
									writedata16_cont(textbgcolor);
								}
							}
						}	
						uint32_t x = 0;
						screen_x = origin_x;
						do {
							uint32_t xsize = width - x;
							if (xsize > 32) xsize = 32;
							uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
							uint32_t bit_mask = 1 << (xsize-1);
							//Serial.printf("     %d %d %x %x - ", x, xsize, bits, bit_mask);
							if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
								while (bit_mask) {
									if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2)) {
										writedata16_cont((bits & bit_mask) ? textcolor : textbgcolor);
										//Serial.write((bits & bit_mask) ? '*' : '.');
									}
									bit_mask = bit_mask >> 1;
									screen_x++ ; // Current actual screen X
								}
								//Serial.println();
								bitoffset += xsize;
							}
							x += xsize;
						} while (x < width) ;
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// output bg color and right hand side
							while (screen_x++ <= end_x) {
								writedata16_cont(textbgcolor);
								//Serial.write('+');
							}
							//Serial.println();
						}
			 			screen_y++;
						linecount--;
					}
				}
			} // 1bpp

			// clear below character - note reusing xcreen_x for this
			screen_x = (end_y + 1 - screen_y) * (end_x + 1 - start_x_min); // How many bytes we need to still output
			//Serial.printf("Clear Below: %d\n", screen_x);
			while (screen_x-- > 1) {
				writedata16_cont(textbgcolor);
			}
			writedata16_last(textbgcolor);
			endSPITransaction();
		}

	}
	// Increment to setup for the next character.
	cursor_x += delta;

}

//strPixelLen			- gets pixel length of given ASCII string
int16_t ILI9341_t3n::strPixelLen(char * str)
{
//	//Serial.printf("strPixelLen %s\n", str);
	if (!str) return(0);
	uint16_t len=0, maxlen=0;
	while (*str)
	{
		if (*str=='\n')
		{
			if ( len > maxlen )
			{
				maxlen=len;
				len=0;
			}
		}
		else
		{
			if (!font)
			{
				len+=textsize*6;
			}
			else
			{

				uint32_t bitoffset;
				const uint8_t *data;
				uint16_t c = *str;

//				//Serial.printf("char %c(%d)\n", c,c);

				if (c >= font->index1_first && c <= font->index1_last) {
					bitoffset = c - font->index1_first;
					bitoffset *= font->bits_index;
				} else if (c >= font->index2_first && c <= font->index2_last) {
					bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
					bitoffset *= font->bits_index;
				} else if (font->unicode) {
					continue;
				} else {
					continue;
				}
				//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
				data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

				uint32_t encoding = fetchbits_unsigned(data, 0, 3);
				if (encoding != 0) continue;
//				uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
//				//Serial.printf("  width =  %d\n", width);
				bitoffset = font->bits_width + 3;
				bitoffset += font->bits_height;

//				int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
//				//Serial.printf("  xoffset =  %d\n", xoffset);
				bitoffset += font->bits_xoffset;
				bitoffset += font->bits_yoffset;

				uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
				bitoffset += font->bits_delta;
//				//Serial.printf("  delta =  %d\n", delta);

				len += delta;//+width-xoffset;
//				//Serial.printf("  len =  %d\n", len);
			}

			if ( len > maxlen )
			{
				maxlen=len;
//					//Serial.printf("  maxlen =  %d\n", maxlen);
			}
		}
		str++;
	}
//	//Serial.printf("Return  maxlen =  %d\n", maxlen);
	return( maxlen );
}

void ILI9341_t3n::drawFontPixel( uint8_t alpha, uint32_t x, uint32_t y ){
	// Adjust alpha based on the number of alpha levels supported by the font (based on bpp)
	// Note: Implemented look-up table for alpha, but made absolutely no difference in speed (T3.6)
	alpha = (uint8_t)(alpha * fontalphamx);
	uint32_t result = ((((textcolorPrexpanded - textbgcolorPrexpanded) * alpha) >> 5) + textbgcolorPrexpanded) & 0b00000111111000001111100000011111;
	Pixel(x,y,(uint16_t)((result >> 16) | result));
}

void ILI9341_t3n::drawFontBits(bool opaque, uint32_t bits, uint32_t numbits, int32_t x, int32_t y, uint32_t repeat)
{
	if (bits == 0) {
		if (opaque) {
			fillRect(x, y, numbits, repeat, textbgcolor);
		}
	} else {
		int32_t x1 = x;
		uint32_t n = numbits;
		int w;
		int bgw;

		w = 0;
		bgw = 0;

		do {
			n--;
			if (bits & (1 << n)) {
				if (bgw>0) {
					if (opaque) {
						fillRect(x1 - bgw, y, bgw, repeat, textbgcolor);
					}
					bgw=0;
				}
				w++;
			} else {
				if (w>0) {
					fillRect(x1 - w, y, w, repeat, textcolor);
					w = 0;
				}
				bgw++;
			}
			x1++;
		} while (n > 0);

		if (w > 0) {
			fillRect(x1 - w, y, w, repeat, textcolor);
		}

		if (bgw > 0) {
			if (opaque) {
				fillRect(x1 - bgw, y, bgw, repeat, textbgcolor);
			}
		}
	}
}

void ILI9341_t3n::setCursor(int16_t x, int16_t y) {
	if (x < 0) x = 0;
	else if (x >= _width) x = _width - 1;
	cursor_x = x;
	if (y < 0) y = 0;
	else if (y >= _height) y = _height - 1;
	cursor_y = y;
	
	if(x>=scroll_x && x<=(scroll_x+scroll_width) && y>=scroll_y && y<=(scroll_y+scroll_height)){
		isWritingScrollArea	= true;
	} else {
		isWritingScrollArea = false;
	}
}

void ILI9341_t3n::getCursor(int16_t *x, int16_t *y) {
  *x = cursor_x;
  *y = cursor_y;
}

void ILI9341_t3n::setTextSize(uint8_t s) {
  textsize = (s > 0) ? s : 1;
}

uint8_t ILI9341_t3n::getTextSize() {
	return textsize;
}

void ILI9341_t3n::setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void ILI9341_t3n::setTextColor(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b;
  // pre-expand colors for fast alpha-blending later
  textcolorPrexpanded = (textcolor | (textcolor << 16)) & 0b00000111111000001111100000011111;
  textbgcolorPrexpanded = (textbgcolor | (textbgcolor << 16)) & 0b00000111111000001111100000011111;
}

void ILI9341_t3n::setTextWrap(boolean w) {
  wrap = w;
}

boolean ILI9341_t3n::getTextWrap()
{
	return wrap;
}

uint8_t ILI9341_t3n::getRotation(void) {
  return rotation;
}

void ILI9341_t3n::sleep(bool enable) {
	beginSPITransaction();
	if (enable) {
		writecommand_cont(ILI9341_DISPOFF);		
		writecommand_last(ILI9341_SLPIN);	
		  endSPITransaction();
	} else {
		writecommand_cont(ILI9341_DISPON);
		writecommand_last(ILI9341_SLPOUT);
		endSPITransaction();
		delay(5);
	}
}



/***************************************************************************************
** Function name:           setTextDatum
** Description:             Set the text position reference datum
***************************************************************************************/
void ILI9341_t3n::setTextDatum(uint8_t d)
{
  textdatum = d;
}


/***************************************************************************************
** Function name:           drawNumber
** Description:             draw a long integer
***************************************************************************************/
int16_t ILI9341_t3n::drawNumber(long long_num, int poX, int poY)
{
  char str[14];
  ltoa(long_num, str, 10);
  return drawString(str, poX, poY);
}


int16_t ILI9341_t3n::drawFloat(float floatNumber, int dp, int poX, int poY)
{
  char str[14];               // Array to contain decimal string
  uint8_t ptr = 0;            // Initialise pointer for array
  int8_t  digits = 1;         // Count the digits to avoid array overflow
  float rounding = 0.5;       // Round up down delta

  if (dp > 7) dp = 7; // Limit the size of decimal portion

  // Adjust the rounding value
  for (uint8_t i = 0; i < dp; ++i) rounding /= 10.0;

  if (floatNumber < -rounding)    // add sign, avoid adding - sign to 0.0!
  {
    str[ptr++] = '-'; // Negative number
    str[ptr] = 0; // Put a null in the array as a precaution
    digits = 0;   // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding; // Round up or down

  // For error put ... in string and return (all TFT_ILI9341_ESP library fonts contain . character)
  if (floatNumber >= 2147483647) {
    strcpy(str, "...");
    //return drawString(str, poX, poY);
  }
  // No chance of overflow from here on

  // Get integer part
  unsigned long temp = (unsigned long)floatNumber;

  // Put integer part into array
  ltoa(temp, str + ptr, 10);

  // Find out where the null is to get the digit count loaded
  while ((uint8_t)str[ptr] != 0) ptr++; // Move the pointer along
  digits += ptr;                  // Count the digits

  str[ptr++] = '.'; // Add decimal point
  str[ptr] = '0';   // Add a dummy zero
  str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

  // Get the decimal portion
  floatNumber = floatNumber - temp;

  // Get decimal digits one by one and put in array
  // Limit digit count so we don't get a false sense of resolution
  uint8_t i = 0;
  while ((i < dp) && (digits < 9)) // while (i < dp) for no limit but array size must be increased
  {
    i++;
    floatNumber *= 10;       // for the next decimal
    temp = floatNumber;      // get the decimal
    ltoa(temp, str + ptr, 10);
    ptr++; digits++;         // Increment pointer and digits count
    floatNumber -= temp;     // Remove that digit
  }

  // Finally we can plot the string and return pixel length
  return drawString(str, poX, poY);
}

/***************************************************************************************
** Function name:           drawString (with or without user defined font)
** Description :            draw string with padding if it is defined
***************************************************************************************/
// Without font number, uses font set by setTextFont()
int16_t ILI9341_t3n::drawString(const String& string, int poX, int poY)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return drawString1(buffer, len, poX, poY);
}

int16_t ILI9341_t3n::drawString1(char string[], int16_t len, int poX, int poY)
{
  int16_t sumX = 0;
  uint8_t padding = 1/*, baseline = 0*/;
  
  uint16_t cwidth = strPixelLen(string); // Find the pixel width of the string in the font
  uint16_t cheight = textsize*8;

  
  if (textdatum || padX)
  {
    switch(textdatum) {
      case TC_DATUM:
        poX -= cwidth/2;
        padding += 1;
        break;
      case TR_DATUM:
        poX -= cwidth;
        padding += 2;
        break;
      case ML_DATUM:
        poY -= cheight/2;
        //padding += 0;
        break;
      case MC_DATUM:
        poX -= cwidth/2;
        poY -= cheight/2;
        padding += 1;
        break;
      case MR_DATUM:
        poX -= cwidth;
        poY -= cheight/2;
        padding += 2;
        break;
      case BL_DATUM:
        poY -= cheight;
        //padding += 0;
        break;
      case BC_DATUM:
        poX -= cwidth/2;
        poY -= cheight;
        padding += 1;
        break;
      case BR_DATUM:
        poX -= cwidth;
        poY -= cheight;
        padding += 2;
        break;
	 /*
      case L_BASELINE:
        poY -= baseline;
        //padding += 0;
        break;
      case C_BASELINE:
        poX -= cwidth/2;
        poY -= baseline;
        //padding += 1;
        break;
      case R_BASELINE:
        poX -= cwidth;
        poY -= baseline;
        padding += 2;
        break;
	*/
    }
    // Check coordinates are OK, adjust if not
    if (poX < 0) poX = 0;
    if (poX+cwidth > width())   poX = width() - cwidth;
    if (poY < 0) poY = 0;
    //if (poY+cheight-baseline >_height) poY = _height - cheight;
  }
  if(font == NULL){
	  for(uint8_t i = 0; i < len-2; i++){
		drawChar((int16_t) (poX+sumX), (int16_t) poY, string[i], textcolor, textbgcolor, textsize);
		sumX += cwidth/(len-2) + padding;
	  }
  } else {
	  setCursor(poX, poY);
	  for(uint8_t i = 0; i < len-2; i++){
		drawFontChar(string[i]);
		setCursor(cursor_x, cursor_y);
	  }
  }
return sumX;
}


void ILI9341_t3n::scrollTextArea(uint8_t scrollSize){
	uint16_t awColors[scroll_width];
	for (int y=scroll_y+scrollSize; y < (scroll_y+scroll_height); y++) { 
		readRect(scroll_x, y, scroll_width, 1, awColors); 
		writeRect(scroll_x, y-scrollSize, scroll_width, 1, awColors);  
	}
	fillRect(scroll_x, (scroll_y+scroll_height)-scrollSize, scroll_width, scrollSize, scrollbgcolor);
}

void ILI9341_t3n::setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h){
	scroll_x = x; 
	scroll_y = y;
	scroll_width = w; 
	scroll_height = h;
}

void ILI9341_t3n::setScrollBackgroundColor(uint16_t color){
	scrollbgcolor=color;
	fillRect(scroll_x,scroll_y,scroll_width,scroll_height,scrollbgcolor);
}

void ILI9341_t3n::enableScroll(void){
	scrollEnable = true;
}

void ILI9341_t3n::disableScroll(void){
	scrollEnable = false;
}

void ILI9341_t3n::resetScrollBackgroundColor(uint16_t color){
	scrollbgcolor=color;
}	

void Adafruit_GFX_Button::initButton(ILI9341_t3n *gfx,
	int16_t x, int16_t y, uint8_t w, uint8_t h,
	uint16_t outline, uint16_t fill, uint16_t textcolor,
	const char *label, uint8_t textsize)
{
	_x = x;
	_y = y;
	_w = w;
	_h = h;
	_outlinecolor = outline;
	_fillcolor = fill;
	_textcolor = textcolor;
	_textsize = textsize;
	_gfx = gfx;
	strncpy(_label, label, 9);
	_label[9] = 0;
}

void Adafruit_GFX_Button::drawButton(bool inverted)
{
	uint16_t fill, outline, text;

	if (! inverted) {
		fill = _fillcolor;
		outline = _outlinecolor;
		text = _textcolor;
	} else {
		fill =  _textcolor;
		outline = _outlinecolor;
		text = _fillcolor;
	}
	_gfx->fillRoundRect(_x - (_w/2), _y - (_h/2), _w, _h, min(_w,_h)/4, fill);
	_gfx->drawRoundRect(_x - (_w/2), _y - (_h/2), _w, _h, min(_w,_h)/4, outline);
	_gfx->setCursor(_x - strlen(_label)*3*_textsize, _y-4*_textsize);
	_gfx->setTextColor(text);
	_gfx->setTextSize(_textsize);
	_gfx->print(_label);
}

bool Adafruit_GFX_Button::contains(int16_t x, int16_t y)
{
	if ((x < (_x - _w/2)) || (x > (_x + _w/2))) return false;
	if ((y < (_y - _h/2)) || (y > (_y + _h/2))) return false;
	return true;
}

