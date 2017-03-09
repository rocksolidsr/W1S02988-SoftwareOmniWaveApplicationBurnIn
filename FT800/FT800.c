//###########################################################################
//
// FILE:	FT800.c
//
// TITLE:	FT800 Library
// Copyright © Cybersonics Inc., 2015
//################################################################################################################################################################################################
//  Ver | mm dd yyyy | Who  | Description of changes
// =====|============|======|===============================================
//  01  | 03 20 2015 | S.R. | First version of FT800 library derived from FTDI's source
//  02  | 03 24 2015 | S.R. | Added Writefromflash function
//  03  | 04 20 2015 | S.R. | Added more modularity to the code to allow different SPI pins etc.
//  04  | 05 06 2015 | S.R. | Changed BitmapSource to accept signed long values
//  05  | 08 04 2015 | S.R. | Updated to add support for Kyocera display
//							  Allows multiple tries to verify FT800 device is present
//							  Added backlight function
//                            Updated initialization routine
//							  Added GPIO for Kyocera display CS
//  06  | 09 30 2015 | S.R. | Changed DISPLAY CS to GPIO19
//################################################################################################################################################################################################
#include "FT800.h"
#include "DSP28x_Project.h"     																		// Device Headerfile and Examples Include File
#include <string.h>

// Global variables
uint8_t  GInit=0,TrnsFlag=0;																			//Global flag to indicate that initialization is done
int32_t  GError = FT_OK;																				//Global error flag
uint8_t  DispGpioPin = FT_GPIO7, AudioGpioPin = FT_GPIO1;												//default FT_GC pin assignments for diaplay and audio control

int32_t CmdFifoWp=0,FreeSpace = FT_CMDFIFO_SIZE - 4;													//command fifo write pointer
unsigned int vsync0, vsync1, voffset, vcycle, hsync0, hsync1, hoffset, hcycle, hsize ,vsize, pclkpol, swizzle, pclk;


/* Structure definitions */

typedef struct sTagXY
{
	int16_t y;																							//y coordinate of touch object
	int16_t x;																							//x coordinate of touch object
	uint16_t tag;																						//TAG value of the object
}sTagXY;

typedef struct sTrackTag
{
	uint16_t tag;																						//TAG value of the object
	uint16_t track;																						//track value of the object
}sTrackTag;

//FT80x font table structure
//Font table address in ROM can be found by reading 32bit value from FT_FONT_TABLE_POINTER location.
//16 font tables are present at the address read from location FT_FONT_TABLE_POINTER
typedef struct FT_Fonts
{
	//All the values are in bytes */
	uint8_t		FontWidth[FT_NUMCHAR_PERFONT];															// Width of each character font from 0 to 127
	uint32_t	FontBitmapFormat;																		// Bitmap format of font wrt bitmap formats supported by FT800 - L1, L4, L8
	uint32_t	FontLineStride;																			// Font line stride in FT800 ROM
	uint32_t	FontWidthInPixels;																		// Font width in pixels
	uint32_t	FontHeightInPixels;																		// Font height in pixels
	uint32_t	PointerToFontGraphicsData;																// Pointer to font graphics raw data
}FT_Fonts_t;


/************************************************************************************************************************************************************************
 * Function:        Init(ResType, rotation)
 * PreCondition:    None
 * Input:           ResType = Display Type and Resolution, rotation = rotation of display (0 or 180)
 * Output:          None
 * Side Effects:    None
 * Overview:        Initializes the display
 ***********************************************************************************************************************************************************************/
FT_Status Init(uint8_t ResType, uint8_t rotation)
{
#ifndef RELEASE
	unsigned long chipid, regid;
	unsigned char ft800IDCounter=0,ft800ID=0;
#endif
#ifdef SPIA_28069
	setup_spi_gpio();
	ft800_spi_fifo_init();
	ft800_spi_init();
#endif
#ifdef SPIB_28069
	setup_spi_gpio();
	ft800_spi_fifo_init();
	ft800_spi_init();
#endif
#ifdef McBSPA_28335
	setup_spi_gpio();
	ft800_spi_init();
#endif

#ifndef RELEASE
	Reset();																							// Bootup of graphics controller
#endif
	DisplayConfigExternalClock(ResType);																// Set the display configurations followed by external clock set, spi clock change wrt FT800
#ifndef RELEASE
	DELAY_US(20);

	while(ft800IDCounter<5&&!ft800ID)
	{
		regid=0;
		chipid=0;
		DELAY_US(10000);
		regid = rd32(REG_ID);
		DELAY_US(10000);
		chipid = rd32(FT_ROM_CHIPID);
		if(FT800_CHIPID != chipid)																			// Identify the chip
		{
			ft800IDCounter++;
		}
		if(regid != 0x7c)
		{
			ft800IDCounter++;
		}
		else
			ft800ID=1;
		if(ft800IDCounter>=5)
			while(1);
	}

	wr8(REG_ROTATE,rotation);																			// Rotate display 0
#endif
	SetDisplayEnablePin(FT_GPIO7);
	Finish();
	DELAY_US(500000);
	BacklightOn();

	return FT_OK;
}


/************************************************************************************************************************************************************************
 * Function:        wrxx(ftAddress, ftLength)
 * PreCondition:    None
 * Input:           ftAddress = FT800 memory space address
 * Output:          None
 * Side Effects:    None
 * Overview:        Writes data to FT800 internal address space
 * Note:            "xx" is one of 8, 16 or 32
 ***********************************************************************************************************************************************************************/
void wr8(unsigned long ftAddress, unsigned char ftData8)
{
	// Initialize the data to send.
	CS_LOW;																								// Set CS# low
	SDATA = ((ftAddress>>16) | MEM_WRITE)<<SPI_SHIFT;													// Send data (MEM_WRITE plus high data address
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ftAddress>>8)<<SPI_SHIFT;																	// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ftAddress<<SPI_SHIFT;																		// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ftData8<<SPI_SHIFT;																			// Send data byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO
	CS_HIGH;																							// Set CS# high
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void wr16(unsigned long ftAddress, unsigned int data)
{
	// Initialize the data to send.
	CS_LOW;																								// Set CS# low
	SDATA = ((ftAddress>>16) | MEM_WRITE)<<SPI_SHIFT;													// Send Memory write plus high address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ftAddress>>8)<<SPI_SHIFT;																	// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ftAddress<<SPI_SHIFT;																		// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = data<<SPI_SHIFT;																			// Send data low byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (data>>8)<<SPI_SHIFT;																		// Send data high byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO
	CS_HIGH;																							// Set CS# high
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void wr32(unsigned long ftAddress, unsigned long ftData32)
{
	// Initialize the data to send.
	CS_LOW;																								// Set CS# low
	SDATA = ((ftAddress>>16) | MEM_WRITE)<<SPI_SHIFT;													// Send Memory write plus high address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ftAddress>>8)<<SPI_SHIFT;																	// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ftAddress<<SPI_SHIFT;																		// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ftData32<<SPI_SHIFT;																		// Send data low byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ftData32>>8)<<SPI_SHIFT;
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ftData32>>16)<<SPI_SHIFT;
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ftData32>>24)<<SPI_SHIFT;																	// Send data high byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	CS_HIGH;																							// Set CS# high
}


/************************************************************************************************************************************************************************
 * Function:        rdxx(ftAddress)
 * PreCondition:    None
 * Input:           ftAddress = FT800 memory space address
 * Output:          ftDataxx (byte, int or long)
 * Side Effects:    None
 * Overview:        Reads FT800 internal address space
 * Note:            "xx" is one of 8, 16 or 32
 ***********************************************************************************************************************************************************************/
unsigned int rd16(unsigned long ftAddress)
{
	unsigned int ftData16;
	CS_LOW;																								// Set CS# low
	SDATA = ((char)(ftAddress >> 16) | MEM_READ)<<SPI_SHIFT; 											// Send Memory Write plus high address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ((char)(ftAddress >> 8))<<SPI_SHIFT;														// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ((char)(ftAddress))<<SPI_SHIFT;																// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ZERO);																						// Send dummy byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ZERO);																						// Send dummy byte to initiate Receive
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ftData16 = RDATA;																					// Read low byte

	SDATA = (ZERO);																						// Send dummy byte to initiate Receive
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ftData16 = (RDATA << 8) | ftData16; 																// Read high byte
	CS_HIGH;																							// Set CS# High
	return ftData16;																					// Return integer read
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long rd32(unsigned long ftAddress)
{
	unsigned long ftData32;
	CS_LOW;																								// Set CS# low
	SDATA = ((char)(ftAddress >> 16) | MEM_READ)<<SPI_SHIFT; 											// Send Memory Write plus high address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ((char)(ftAddress >> 8))<<SPI_SHIFT;														// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ((char)(ftAddress))<<SPI_SHIFT;																// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ZERO);																						// Send dummy byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ZERO);																						// Send dummy byte to initiate Receive
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ftData32 = RDATA;																					// Read low byte

	SDATA = (ZERO);																						// Send dummy byte to initiate Receive
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ftData32 = (RDATA << 8) | ftData32;

	SDATA = (ZERO);																						// Send dummy byte to initiate Receive
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ftData32 = ((unsigned long)RDATA << 16) | ftData32;

	SDATA = (ZERO);																						// Send dummy byte to initiate Receive
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ftData32 = ((unsigned long)RDATA << 24) | ftData32; 												// Read high byte
	CS_HIGH;																							// Set CS# high
	return ftData32;																					// Return long read
}


/************************************************************************************************************************************************************************
 * Function:        min(a, b)
 * PreCondition:    None
 * Input:           a = number, b =  number
 * Output:          which ever value is the minimum
 * Side Effects:    None
 * Overview:        returns the minimum value of a and b
 ***********************************************************************************************************************************************************************/
unsigned int min(unsigned int a, unsigned int b)
{
	return !(b<a)?a:b;
}


/************************************************************************************************************************************************************************
 * Function:        Transfer(Value8)
 * PreCondition:    None
 * Input:           Value8 = 8 bit value
 * Output:          none
 * Side Effects:    None
 * Overview:        transmits an 8-bit value over SPI
 ***********************************************************************************************************************************************************************/
void Transfer(uint8_t Value8)
{
	SDATA = Value8<<SPI_SHIFT;																			// Send command
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;
}


/************************************************************************************************************************************************************************
 * Function:        Transfer32(Value32)
 * PreCondition:    None
 * Input:           Value32 = 32 bit value
 * Output:          none
 * Side Effects:    None
 * Overview:        transmits a 32-bit value over SPI
 ***********************************************************************************************************************************************************************/
void Transfer32(unsigned long Value32)
{
	SDATA = Value32<<SPI_SHIFT;																			// Send data low byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (Value32>>8)<<SPI_SHIFT;
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (Value32>>16)<<SPI_SHIFT;
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (Value32>>24)<<SPI_SHIFT;																	// Send data high byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO
}


/************************************************************************************************************************************************************************
 * Function:        StartWrite(Addr)
 * PreCondition:    None
 * Input:           Addr = FT800 address to write to
 * Output:          none
 * Side Effects:    None
 * Overview:        transmits the start address to the FT800 to begin a write instruction
 ***********************************************************************************************************************************************************************/
void StartWrite(unsigned long Addr)
{
	CS_LOW;
	SDATA = ((Addr>>16) | MEM_WRITE)<<SPI_SHIFT;														// Send Memory write plus high address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (Addr>>8)<<SPI_SHIFT;																		// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = Addr<<SPI_SHIFT;																			// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;
}


/************************************************************************************************************************************************************************
 * Function:        Write(Addr, Value8)
 * PreCondition:    None
 * Input:           Addr = FT800 address to write to, Value8 = 8 bit value to write
 * Output:          none
 * Side Effects:    None
 * Overview:        transmits the start address to the FT800 to begin a write instruction and the write value
 ***********************************************************************************************************************************************************************/
void Write(unsigned long Addr, uint8_t Value8)
{
	StartWrite(Addr);
	SDATA = Value8<<SPI_SHIFT;																			// Send data byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;
	CS_HIGH;
}


/************************************************************************************************************************************************************************
 * Function:        SetDisplayEnablePin(GpioBit)
 * PreCondition:    None
 * Input:           GpioBit = FT800 display enable bit
 * Output:          GpioBit on FT800, 1 = output, 0 = input
 * Side Effects:    None
 * Overview:        Enables the display
 ***********************************************************************************************************************************************************************/
void SetDisplayEnablePin(uint8_t GpioBit)
{
	DispGpioPin = GpioBit;																				// update the display enable pin gpio bit number
	Write(REG_GPIO_DIR,(1 << DispGpioPin) | Read(REG_GPIO_DIR));										// set the direction of this bit to output. 1 is output and 0 is input
}


/************************************************************************************************************************************************************************
 * Function:        SetAudioEnablePin(GpioBit)
 * PreCondition:    None
 * Input:           GpioBit = FT800 audio enable bit
 * Output:          GpioBit on FT800, 1 = output, 0 = input
 * Side Effects:    None
 * Overview:        Enables the audio
 ***********************************************************************************************************************************************************************/
void SetAudioEnablePin(uint8_t GpioBit)
{
	AudioGpioPin = GpioBit;																				// update the audio enable pin gpio bit number
	Write(REG_GPIO_DIR,(1 << AudioGpioPin) | Read(REG_GPIO_DIR));										// set the direction of this bit to output. 1 is output and 0 is input
}


/************************************************************************************************************************************************************************
 * Function:        DisplayOn()
 * PreCondition:    None
 * Input:           None
 * Output:          Turns on display
 * Side Effects:    None
 * Overview:        Turns on display
 ***********************************************************************************************************************************************************************/
void DisplayOn(void)
{
	Write(REG_GPIO,(1 << DispGpioPin) | Read(REG_GPIO));												// switch on the display, 1 means enable and 0 means disable
}


/************************************************************************************************************************************************************************
 * Function:        DisplayOff()
 * PreCondition:    None
 * Input:           None
 * Output:          Turns off display
 * Side Effects:    None
 * Overview:        Turns off display
 ***********************************************************************************************************************************************************************/
void DisplayOff(void)
{
	Write(REG_GPIO,(~(1 << DispGpioPin)) & Read(REG_GPIO));												// switch off the display
}


/************************************************************************************************************************************************************************
 * Function:        BacklightOn()
 * PreCondition:    None
 * Input:           None
 * Output:          Turns on backlight
 * Side Effects:    None
 * Overview:        Turns on backlight
 ***********************************************************************************************************************************************************************/
void BacklightOn(void)
{
    wr8(REG_PWM_DUTY, 127);
}


/************************************************************************************************************************************************************************
 * Function:        BacklightOff()
 * PreCondition:    None
 * Input:           None
 * Output:          Turns off backlight
 * Side Effects:    None
 * Overview:        Turns off backlight
 ***********************************************************************************************************************************************************************/
void BacklightOff(void)
{
    wr8(REG_PWM_DUTY, 0);
}


//Not currently using
/*void SetInterruptPin(uint16_t Intpin)
{
	// update the interrupt pin
	IntPin = Intpin;
}*/


/************************************************************************************************************************************************************************
 * Function:        AudioOn()
 * PreCondition:    None
 * Input:           None
 * Output:          Turns on audio
 * Side Effects:    None
 * Overview:        Turns on audio
 ***********************************************************************************************************************************************************************/
void AudioOn(void)
{
	Write(REG_GPIO,(1 << AudioGpioPin) | Read(REG_GPIO));												// switch on the audio , 1 means enable and 0 means disable
}


/************************************************************************************************************************************************************************
 * Function:        AudioOff()
 * PreCondition:    None
 * Input:           None
 * Output:          Turns off audio
 * Side Effects:    None
 * Overview:        Turns off audio
 ***********************************************************************************************************************************************************************/
void AudioOff(void)
{
	Write(REG_GPIO,(~(1 << AudioGpioPin)) & Read(REG_GPIO));											// switch off the audio , 1 means enable and 0 means disable
}


/************************************************************************************************************************************************************************
 * Function:        ResetCopro()
 * PreCondition:    coprocessor returns error
 * Input:           None
 * Output:          Resets the coprocessor
 * Side Effects:    None
 * Overview:        reset coprocessor only - do this only when coprocessor returns error. for graphic processor error, utilize reset() api
 ***********************************************************************************************************************************************************************/
void ResetCopro(void)
{
	Write(REG_CPURESET,FT_RESET_HOLD_COPROCESSOR);														// first hold the coprocessor in reset
	CmdFifoWp = 0;																						// make the cmd read write pointers to 0
	FreeSpace = FT_CMDFIFO_SIZE - 4;
	wr16(REG_CMD_READ,0);
	wr16(REG_CMD_WRITE,0);
	DELAY_US(10);																						// just to make sure reset is fine
	Write(REG_CPURESET,FT_RESET_RELEASE_COPROCESSOR);													// release the coprocessors from reset
	//ideally delay of 25ms is required for audio engine to playback mute sound to avoid pop sound
}


/************************************************************************************************************************************************************************
 * Function:        Reset()
 * PreCondition:    None
 * Input:           None
 * Output:          Resets the entire graphics processor
 * Side Effects:    None
 * Overview:        Resets the entire graphics processor
 ***********************************************************************************************************************************************************************/
void Reset(void)
{
	PDN_Cycle();																						// Reset of whole graphics controller
	ActiveInternalClock();																				// send active command to enable SPI, followed by download of default DL into graphics engine,
																										// followed by SPI settings wrt internal clock requirements during bootup stage
}


/************************************************************************************************************************************************************************
 * Function:        PDN_Cycle()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Power Cycles displays via PDN GPIO
 ***********************************************************************************************************************************************************************/
void PDN_Cycle(void)
{
	TURN_ON_DISP;
	DELAY_US(6000);
	TURN_OFF_DISP;
	DELAY_US(6000);
	TURN_ON_DISP;
	DELAY_US(6000);
}


/************************************************************************************************************************************************************************
 * Function:        Write_withSize()
 * PreCondition:    None
 * Input:           Addr = FT800 address, *Src = data to write, NBytes = size of data
 * Output:          None
 * Side Effects:    None
 * Overview:        Writes data to FT800
 ***********************************************************************************************************************************************************************/
void Write_withSize(uint32_t Addr, uint8_t *Src, uint32_t NBytes)
{
	uint32_t i;
	StartWrite(Addr);
	for(i=0;i<NBytes;i++)
	{
		SDATA = *Src++<<SPI_SHIFT;																		// Send data byte
		while(SPI_NOT_RECEIVED) { }																		// Wait for data to be received
		RDATA;
	}
	CS_HIGH;
}


/************************************************************************************************************************************************************************
 * Function:        ActiveInternalClock()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Sets active command, set internal clock and download first display list
 ***********************************************************************************************************************************************************************/
void ActiveInternalClock(void)
{
	uint8_t FT_DLCODE_BOOTUP[12] =
	{
	  0,0,0,2,																							// GPU instruction CLEAR_COLOR_RGB - black color
	  7,0,0,38, 																						// GPU instruction CLEAR
	  0,0,0,0,  																						// GPU instruction DISPLAY
	};

	//SpibRegs.SPIBRR = 0x0005;																			// set to <11Mhz ~4Mhz
	HostCommand(FT_ACTIVE);																				// wake up the processor from sleep state
	DELAY_US(20);

	Write_withSize(FT_RAM_DL,FT_DLCODE_BOOTUP,12);														// Download the first display list
	Write(REG_DLSWAP,FT_DLSWAP_FRAME);																	// perform first swap command
	CmdFifoWp = 0;																						// make the cmd read write pointers to 0
	FreeSpace = FT_CMDFIFO_SIZE - 4;
}


/************************************************************************************************************************************************************************
 * Function:        DisplayConfigExternalClock()
 * PreCondition:    None
 * Input:           ResType = display select (i.e. 320x240 vs 480x272 vs RiverDI)
 * Output:          None
 * Side Effects:    None
 * Overview:        Sets the display parameters and sets it to use external clock
 ***********************************************************************************************************************************************************************/
void DisplayConfigExternalClock(uint8_t ResType)
{
	if(ResType == FT_DISPLAY_QVGA_320x240)
	{
		vsync0 	=	FT_DISPLAY_VSYNC0_QVGA;
		vsync1 	=	FT_DISPLAY_VSYNC1_QVGA;
		voffset	=	FT_DISPLAY_VOFFSET_QVGA;
		vcycle 	=	FT_DISPLAY_VCYCLE_QVGA;
		hsync0 	=	FT_DISPLAY_HSYNC0_QVGA;
		hsync1 	=	FT_DISPLAY_HSYNC1_QVGA;
		hoffset	=	FT_DISPLAY_HOFFSET_QVGA;
		hcycle 	=	FT_DISPLAY_HCYCLE_QVGA;
		hsize  	=	FT_DISPLAY_HSIZE_QVGA;
		vsize  	=	FT_DISPLAY_VSIZE_QVGA;
		pclkpol	=	FT_DISPLAY_PCLKPOL_QVGA;
		swizzle	=	FT_DISPLAY_SWIZZLE_QVGA;
		pclk   	=	FT_DISPLAY_PCLK_QVGA;

	}
	else if(ResType == FT_DISPLAY_WQVGA_480x272)
	{
		vsync0 	=	FT_DISPLAY_VSYNC0_WQVGA;
		vsync1 	=	FT_DISPLAY_VSYNC1_WQVGA;
		voffset	=	FT_DISPLAY_VOFFSET_WQVGA;
		vcycle 	=	FT_DISPLAY_VCYCLE_WQVGA;
		hsync0 	=	FT_DISPLAY_HSYNC0_WQVGA;
		hsync1 	=	FT_DISPLAY_HSYNC1_WQVGA;
		hoffset	=	FT_DISPLAY_HOFFSET_WQVGA;
		hcycle 	=	FT_DISPLAY_HCYCLE_WQVGA;
		hsize  	=	FT_DISPLAY_HSIZE_WQVGA;
		vsize  	=	FT_DISPLAY_VSIZE_WQVGA;
		pclkpol	=	FT_DISPLAY_PCLKPOL_WQVGA;
		swizzle	=	FT_DISPLAY_SWIZZLE_WQVGA;
		pclk   	=	FT_DISPLAY_PCLK_WQVGA;

	}
	else if(ResType  == FT_DISPLAY_RIVERDI_320x240)
	{
		vsync0 	=	FT_DISPLAY_RIVERDI_VSYNC0;
		vsync1 	=	FT_DISPLAY_RIVERDI_VSYNC1;
		voffset	=	FT_DISPLAY_RIVERDI_VOFFSET;
		vcycle 	=	FT_DISPLAY_RIVERDI_VCYCLE;
		hsync0 	=	FT_DISPLAY_RIVERDI_HSYNC0;
		hsync1 	=	FT_DISPLAY_RIVERDI_HSYNC1;
		hoffset	=	FT_DISPLAY_RIVERDI_HOFFSET;
		hcycle 	=	FT_DISPLAY_RIVERDI_HCYCLE;
		hsize  	=	FT_DISPLAY_RIVERDI_HSIZE;
		vsize  	=	FT_DISPLAY_RIVERDI_VSIZE;
		pclkpol	=	FT_DISPLAY_RIVERDI_PCLKPOL;
		swizzle	=	FT_DISPLAY_RIVERDI_SWIZZLE;
		pclk   	=	FT_DISPLAY_RIVERDI_PCLK;

	}
	else if(ResType  == FT_DISPLAY_KYOCERA)
	{
		EALLOW;
		GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;      														//Display CS Pin
	    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     															//Enable pull-up on GPIO9 (DISPLAY_CS)
	    GpioDataRegs.GPASET.bit.GPIO19 = 1;     															//Set display to not selected to start
	    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     															//set GPIO as output, DISPLAY CS
	    EDIS;

		vsync0 	=	FT_DISPLAY_KYOCERA_VSYNC0;
		vsync1 	=	FT_DISPLAY_KYOCERA_VSYNC1;
		voffset	=	FT_DISPLAY_KYOCERA_VOFFSET;
		vcycle 	=	FT_DISPLAY_KYOCERA_VCYCLE;
		hsync0 	=	FT_DISPLAY_KYOCERA_HSYNC0;
		hsync1 	=	FT_DISPLAY_KYOCERA_HSYNC1;
		hoffset	=	FT_DISPLAY_KYOCERA_HOFFSET;
		hcycle 	=	FT_DISPLAY_KYOCERA_HCYCLE;
		hsize  	=	FT_DISPLAY_KYOCERA_HSIZE;
		vsize  	=	FT_DISPLAY_KYOCERA_VSIZE;
		pclkpol	=	FT_DISPLAY_KYOCERA_PCLKPOL;
		swizzle	=	FT_DISPLAY_KYOCERA_SWIZZLE;
		pclk   	=	FT_DISPLAY_KYOCERA_PCLK;

	}
	else
	{
		vsync0 	=	FT_DISPLAY_VSYNC0;
		vsync1 	=	FT_DISPLAY_VSYNC1;
		voffset	=	FT_DISPLAY_VOFFSET;
		vcycle 	=	FT_DISPLAY_VCYCLE;
		hsync0 	=	FT_DISPLAY_HSYNC0;
		hsync1 	=	FT_DISPLAY_HSYNC1;
		hoffset	=	FT_DISPLAY_HOFFSET;
		hcycle 	=	FT_DISPLAY_HCYCLE;
		hsize  	=	FT_DISPLAY_HSIZE;
		vsize  	=	FT_DISPLAY_VSIZE;
		pclkpol	=	FT_DISPLAY_PCLKPOL;
		swizzle	=	FT_DISPLAY_SWIZZLE;
		pclk   	=	FT_DISPLAY_PCLK;

	}

#ifndef RELEASE
	wr16(REG_VSYNC0, 	vsync0 );
	wr16(REG_VSYNC1, 	vsync1 );
	wr16(REG_VOFFSET, 	voffset);
	wr16(REG_VCYCLE, 	vcycle );
	wr16(REG_HSYNC0, 	hsync0 );
	wr16(REG_HSYNC1, 	hsync1 );
	wr16(REG_HOFFSET, 	hoffset);
	wr16(REG_HCYCLE, 	hcycle );
	wr16(REG_HSIZE,		hsize  );
	wr16(REG_VSIZE, 	vsize  );
	wr16(REG_PCLK_POL, 	pclkpol);
	wr16(REG_SWIZZLE, 	swizzle);
	wr16(REG_PCLK,		pclk   );																		// after configuring display parameters, configure pclk */



	BacklightOff();
	HostCommand(FT_CLKEXT);																				// send host command to change the clock source from internal to external
	DisplayOn();


#endif

	//SpibRegs.SPIBRR =0x0005;																			// change the clock to normal operating frequency
}


/************************************************************************************************************************************************************************
 For more information of functions below refer to the FT800 programmers guide
 http://www.ftdichip.com/Support/Documents/ProgramGuides/FT800%20Programmers%20Guide.pdf
 ***********************************************************************************************************************************************************************/
//Apis related to graphics processor
//api to enable or disable interrupts
void EnableInterrupts_FT(uint8_t GEnable,uint8_t Mask)
{
	Write(REG_INT_EN,GEnable);																			// 1 means enable global interrupts, 0 means disable global interrupts
	Write(REG_INT_MASK,Mask);																			// 0 means interrupts are masked, 1 means interrupts are not masked
}

//read the interrupt flag register - note that on ft800/ft801 the interrupts are clear by read
uint8_t ReadIntReg(void)
{
	return (Read(REG_INT_FLAGS));
}

//APIs related to graphics engine
FT_GEStatus AlphaFunc(uint8_t Func, uint8_t Ref)
{
  return ( WriteCmd((9UL << 24) | ((Func & 7L) << 8) | ((Ref & 0xFFL) << 0)) );
}

FT_GEStatus Begin(uint8_t Prim)
{
  return ( WriteCmd((31UL << 24) | Prim) );
}

FT_GEStatus BitmapHandle(uint8_t Handle)
{
  return ( WriteCmd((5UL << 24) | Handle) );
}

FT_GEStatus BitmapLayout(uint8_t Format, uint16_t Linestride, uint16_t Height)
{
  return WriteCmd((7UL << 24) | ((Format & 0x1FL) << 19) | ((Linestride & 0x3FFL) << 9) | ((Height & 0x1FFL) << 0));
}

FT_GEStatus BitmapSize(uint8_t filter, uint8_t wrapx, uint8_t wrapy, uint16_t width, uint16_t height)
{
	return WriteCmd(((8UL << 24) | (((filter)&1UL)<<20)|(((wrapx)&1UL)<<19)|(((wrapy)&1UL)<<18)|(((width)&511UL)<<9)|(((height)&511UL)<<0)));
}

FT_GEStatus BitmapSource(signed long Addr)
{
	return ( WriteCmd((1UL << 24) | ((Addr & 0xFFFFFL) << 0)) );
}

FT_GEStatus BitmapTransformA(int32_t A)
{
	return ( WriteCmd((21UL << 24) | ((A & 0x1FFFFL) << 0)) );
}

FT_GEStatus BitmapTransformB(int32_t B)
{
	return ( WriteCmd((22UL << 24) | ((B & 0x1FFFFL) << 0)) );
}

FT_GEStatus BitmapTransformC(int32_t C)
{
	return ( WriteCmd((23UL << 24) | ((C & 0xFFFFFFL) << 0)) );
}

FT_GEStatus BitmapTransformD(int32_t D)
{
	return ( WriteCmd((24UL << 24) | ((D & 0x1FFFFL) << 0)) );
}

FT_GEStatus BitmapTransformE(int32_t E)
{
	return ( WriteCmd((25UL << 24) | ((E & 0x1FFFFL) << 0)) );
}

FT_GEStatus BitmapTransformF(int32_t F)
{
	return ( WriteCmd((26UL << 24) | ((F & 0xFFFFFFL) << 0)) );
}

FT_GEStatus BlendFunc(uint8_t Src, uint8_t Dst)
{
	return ( WriteCmd((11UL << 24) | ((Src & 7L) << 3) | ((Dst & 7L) << 0)) );
}

FT_GEStatus Call(uint16_t Dest)
{
	return ( WriteCmd((29UL << 24) | ((Dest & 0xFFFFL) << 0)) );
}

FT_GEStatus Cell(uint8_t Cell)
{
	return ( WriteCmd((6UL << 24) | ((Cell & 0x7FL) << 0)) );
}

FT_GEStatus ClearColorA(uint8_t Alpha)
{
	return ( WriteCmd((15UL << 24) | ((Alpha & 0xFFL) << 0)) );
}

FT_GEStatus ClearColorRGB(uint8_t red, uint8_t green, uint8_t blue)
{
	return ( WriteCmd((2UL << 24) | ((red & 0xFFL) << 16) | ((green & 0xFFL) << 8) | ((blue & 0xFFL) << 0)) );
}

FT_GEStatus ClearColorRGB_Single(uint32_t rgb)
{
	return ( WriteCmd((2UL << 24) | (rgb & 0xFFFFFFL)) );
}

FT_GEStatus Clear(uint8_t c, uint8_t s, uint8_t t)
{
	uint8_t m = (c << 2) | (s << 1) | t;
	return ( WriteCmd((38UL << 24) | m) );
}

FT_GEStatus Clear_void(void)
{
	return ( WriteCmd((38UL << 24) | 7) );
}

FT_GEStatus ClearStencil(uint8_t s)
{
	return ( WriteCmd((17UL << 24) | ((s & 0xFFL) << 0)) );
}

FT_GEStatus ClearTag(uint8_t s)
{
	return ( WriteCmd((18UL << 24) | ((s & 0xFFL) << 0)) );
}

FT_GEStatus ColorA(uint8_t Alpha)
{
	return ( WriteCmd((16UL << 24) | ((Alpha & 0xFFL) << 0)) );
}

FT_GEStatus ColorMask(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
	return ( WriteCmd((32UL << 24) | ((r & 1L) << 3) | ((g & 1L) << 2) | ((b & 1L) << 1) | ((a & 1L) << 0)) );
}

FT_GEStatus ColorRGB(uint8_t red, uint8_t green, uint8_t blue)
{
	return (WriteCmd((4UL << 24) | ((red & 0xFFL) << 16) | ((green & 0xFFL) << 8) | ((blue & 0xFFL) << 0)));
}

FT_GEStatus Display(void)
{
	return ( WriteCmd((0UL << 24)) );
}

FT_GEStatus End(void) {
	return ( WriteCmd((33UL << 24)) );
}

FT_GEStatus Jump(uint16_t Dest)
{
	return ( WriteCmd((30UL << 24) | ((Dest & 0x7FFL) << 0)) );
}

FT_GEStatus LineWidth(uint16_t Width)
{
	return ( WriteCmd((14UL << 24) | ((Width & 0xFFFL) << 0)) );
}

FT_GEStatus Macro(uint8_t m)
{
	return ( WriteCmd((37UL << 24) | ((m & 1L) << 0)) );
}

FT_GEStatus PointSize(uint16_t Size)
{
	return ( WriteCmd((13UL << 24) | ((Size & 0x1FFFL) << 0)) );
}

FT_GEStatus RestoreContext(void)
{
	return ( WriteCmd((35UL << 24)) );
}

FT_GEStatus Return(void)
{
	return ( WriteCmd((36UL << 24)) );
}

FT_GEStatus SaveContext(void)
{
	return ( WriteCmd((34UL << 24)) );
}

FT_GEStatus ScissorSize(uint16_t Width, uint16_t Height)
{
	return ( WriteCmd((28UL << 24) | ((Width & 0x3FFL) << 10) | ((Height & 0x3FFL) << 0)) );
}

FT_GEStatus ScissorXY(uint16_t x, uint16_t y)
{
	return ( WriteCmd((27UL << 24) | ((x & 0x1FFL) << 9) | ((y & 0x1FFL) << 0)) );
}

FT_GEStatus StencilFunc(uint8_t Func, uint8_t Ref, uint8_t Mask)
{
	return ( WriteCmd((10UL << 24) | ((Func & 7L) << 16) | ((Ref & 0xFFL) << 8) | ((Mask & 0xFFL) << 0)) );
}

FT_GEStatus StencilMask(uint8_t Mask)
{
	return ( WriteCmd((19UL << 24) | ((Mask & 0xFFL) << 0)) );
}

FT_GEStatus StencilOp(uint8_t Sfail, uint8_t Spass)
{
	return ( WriteCmd((12UL << 24) | ((Sfail & 7L) << 3) | ((Spass & 7L) << 0)) );
}

FT_GEStatus TagMask(uint8_t Mask)
{
	return ( WriteCmd((20UL << 24) | ((Mask & 1L) << 0)) );
}

FT_GEStatus Tag(uint8_t s)
{
	return ( WriteCmd((3UL << 24) | ((s & 0xFFL) << 0)) );
}

FT_GEStatus Vertex2f(int16_t x, int16_t y)
{
	return ( WriteCmd((1UL << 30) | ((x & 0x7FFFL) << 15) | ((y & 0x7FFFL) << 0)) );
}

FT_GEStatus Vertex2ii(uint16_t x, uint16_t y, uint8_t Handle, uint8_t Cell)
{
	return (WriteCmd((2UL << 30) | ((x & 0x1FFL) << 21) | ((y & 0x1FFL) << 12) | ((Handle & 0x1FL) << 7) | ((Cell & 0x7FL) << 0)) );
}


//graphics helper apis
FT_GEStatus ColorRGB_Single(uint32_t rgb)
{
	return ( WriteCmd((4UL << 24) | (rgb & 0xFFFFFFL)) );
}

//Form two commands, one for rgb and the other for a
FT_GEStatus ColorARGB(unsigned long argb)
{
	FT_GEStatus Status;
	WriteCmd((4UL << 24) | (argb & 0xFFFFFFL));
	Status = WriteCmd((16UL << 24) | ((argb>>24) & 0xFFL) );
	return Status;
}

FT_GEStatus Cmd_Logo(void)
{
	FT_GEStatus Status;
	Status = WriteCmd(CMD_LOGO);
	return Status;
}

FT_GEStatus Cmd_Append(uint32_t Ptr, uint32_t Num)
{
	FT_GEStatus Status;
	WriteCmd(CMD_APPEND);
	WriteCmd(Ptr);
	Status = WriteCmd(Num);//checking only for the last command
	return Status;
}

FT_GEStatus Cmd_BGColor(uint32_t c)
{
	FT_GEStatus Status;
	WriteCmd(CMD_BGCOLOR);
	Status = WriteCmd(c);
	return Status;
}

FT_GEStatus Cmd_Button(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t Font, uint16_t Options, const char *s)
{
	FT_GEStatus Status;
	WriteCmd(CMD_BUTTON);
	WriteCmd(((y & 0xFFFFL) <<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (Font & 0xFFFFL));
	Status = WriteCmd_withSize((uint8_t *)s,strlen((const char *)s) + 1);//make sure last byte is added into the
	return Status;
}

//Check the result of command calibrate by cmd_GetResult
FT_GEStatus Cmd_Calibrate(uint32_t Result)
{
	FT_GEStatus Status;
	WriteCmd(CMD_CALIBRATE);
	Status = WriteCmd(Result);//write extra word for result
	return Status;
}

FT_GEStatus Cmd_Clock(int16_t x, int16_t y, int16_t r, uint16_t Options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms)
{
	FT_GEStatus Status;
	WriteCmd(CMD_CLOCK);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (r & 0xFFFFL));
	WriteCmd(((m & 0xFFFFL)<<16) | (h & 0xFFFFL));
	Status = WriteCmd(((ms & 0xFFFFL)<<16) | (s & 0xFFFFL));
	return Status;
}


FT_GEStatus Cmd_ColdStart(void)
{
	return ( WriteCmd(CMD_COLDSTART) );
}

FT_GEStatus Cmd_Dial(int16_t x, int16_t y, int16_t r, uint16_t Options, uint16_t Val)
{
	FT_GEStatus Status;
	WriteCmd(CMD_DIAL);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (r & 0xFFFFL));
	Status = WriteCmd(Val);
	return Status;
}

FT_GEStatus Cmd_DLStart(void)
{
	return ( WriteCmd(CMD_DLSTART) );
}

FT_GEStatus Cmd_FGColor(uint32_t c)
{
	FT_GEStatus Status;
	WriteCmd(CMD_FGCOLOR);
	Status = WriteCmd(c);
	return Status;
}


FT_GEStatus Cmd_Gauge(int16_t x, int16_t y, int16_t r, uint16_t Options, uint16_t Major, uint16_t Minor, uint16_t Val, uint16_t Range)
{
	FT_GEStatus Status;
	WriteCmd(CMD_GAUGE);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (r & 0xFFFFL));
	WriteCmd(((Minor & 0xFFFFL)<<16) | (Major & 0xFFFFL));
	Status = WriteCmd(((Range & 0xFFFFL)<<16) | (Val & 0xFFFFL));
	return Status;
}

//Results are available from getresults api
FT_GEStatus Cmd_GetMatrix(void)
{
	FT_GEStatus Status;
	WriteCmd(CMD_GETMATRIX);
	WriteCmd(0);
	WriteCmd(0);
	WriteCmd(0);
	WriteCmd(0);
	WriteCmd(0);
	Status = WriteCmd(0);
	return Status;
}

//perform this api and wait for the completion by finish and use getresults api  for the result
/*FT_GEStatus Cmd_GetProps(uint32_t &Ptr, uint32_t &w, uint32_t &h)
{
	FT_GEStatus Status;
	WriteCmd(CMD_GETPROPS);
	WriteCmd(0);
	WriteCmd(0);
	Status = WriteCmd(0);
	return Status;
}*/

//perform this api and wait for the completion by finish and use cmd_getresult for the result
FT_GEStatus Cmd_GetPtr(uint32_t Result)
{
	FT_GEStatus Status;
	WriteCmd(CMD_GETPTR);
	Status = WriteCmd(Result);
	return Status;
}

FT_GEStatus Cmd_GradColor(uint32_t c)
{
	FT_GEStatus Status;
	WriteCmd(CMD_GRADCOLOR);
	Status = WriteCmd(c);
	return Status;
}

FT_GEStatus Cmd_Gradient(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1)
{
	FT_GEStatus Status;
	WriteCmd(CMD_GRADIENT);
	WriteCmd(((y0 & 0xFFFFL)<<16)|(x0 & 0xFFFFL));
	WriteCmd(rgb0);
	WriteCmd(((y1 & 0xFFFFL)<<16)|(x1 & 0xFFFFL));
	Status = WriteCmd(rgb1);
	return Status;
}

//after calling this api copy the raw content which is output from deflate
FT_GEStatus Cmd_Inflate(uint32_t Ptr)
{
	FT_GEStatus Status;
	WriteCmd(CMD_INFLATE);
	Status = WriteCmd(Ptr);
	return Status;
}

FT_GEStatus Cmd_Interrupt(uint32_t ms)
{
	FT_GEStatus Status;
	WriteCmd(CMD_INTERRUPT);
	Status = WriteCmd(ms);
	return Status;
}

FT_GEStatus Cmd_Keys(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t Font, uint16_t Options, const char *s)
{
	FT_GEStatus Status;
	WriteCmd(CMD_KEYS);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (Font & 0xFFFFL));
	Status = WriteCmd_withSize((uint8_t *)s,strlen((const char *)s) + 1);
	return Status;
}

FT_GEStatus Cmd_LoadIdentity(void)
{
	return ( WriteCmd(CMD_LOADIDENTITY) );
}

//after this api, copy the jpeg data into fifo
FT_GEStatus Cmd_LoadImage(uint32_t Ptr, int32_t Options)
{
	FT_GEStatus Status;
	WriteCmd(CMD_LOADIMAGE);
	WriteCmd(Ptr);
	Status = WriteCmd(Options);
	return Status;
}

FT_GEStatus Cmd_Memcpy(uint32_t Dest, uint32_t Src, uint32_t Num)
{
	FT_GEStatus Status;
	WriteCmd(CMD_MEMCPY);
	WriteCmd(Dest);
	WriteCmd(Src);
	Status = WriteCmd(Num);
	return Status;
}

FT_GEStatus Cmd_Memset(uint32_t Ptr, uint8_t Value, uint32_t Num)
{
	FT_GEStatus Status;
	WriteCmd(CMD_MEMSET);
	WriteCmd(Ptr);
	WriteCmd(Value);
	Status = WriteCmd(Num);
	return Status;
}

//perform this api, wait for the completion and use cmd_getresult for the result
/*FT_GEStatus Cmd_Memcrc(uint32_t Ptr, uint32_t Num,uint32_t &Result)
{
	FT_GEStatus Status;
	WriteCmd(CMD_MEMCRC);
	WriteCmd(Ptr);
	WriteCmd(Num);
	Status = WriteCmd(Result);
	return Status;
}*/

FT_GEStatus Cmd_Memwrite(uint32_t Ptr, uint32_t Num)
{
	FT_GEStatus Status;
	WriteCmd(CMD_MEMWRITE);
	WriteCmd(Ptr);
	Status = WriteCmd(Num);
	return Status;
}

FT_GEStatus Cmd_Memzero(uint32_t Ptr, uint32_t Num)
{
	FT_GEStatus Status;
	WriteCmd(CMD_MEMZERO);
	WriteCmd(Ptr);
	Status = WriteCmd(Num);
	return Status;
}


FT_GEStatus Cmd_Number(int16_t x, int16_t y, uint8_t Font, uint16_t Options, int32_t n)
{
	FT_GEStatus Status;
	WriteCmd(CMD_NUMBER);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (Font & 0xFFFFL));
	Status = WriteCmd(n);
	return Status;
}

FT_GEStatus Cmd_Progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t Options, uint16_t Val, uint16_t Range)
{
	FT_GEStatus Status;
	WriteCmd(CMD_PROGRESS);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(((Val & 0xFFFFL)<<16) | (Options & 0xFFFFL));
	Status = WriteCmd(Range);
	return Status;
}

//perform this api, wait for the completion and use cmd_getresult for the result
FT_GEStatus Cmd_RegRead(uint32_t Ptr,uint32_t Result)
{
	FT_GEStatus Status;
	WriteCmd(CMD_REGREAD);
	WriteCmd(Ptr);
	Status = WriteCmd(Result);
	return Status;
}

FT_GEStatus Cmd_Rotate(int32_t a)
{
	FT_GEStatus Status;
	WriteCmd(CMD_ROTATE);
	Status = WriteCmd(a);
	return Status;
}

FT_GEStatus Cmd_Scale(int32_t sx, int32_t sy)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SCALE);
	WriteCmd(sx);
	Status = WriteCmd(sy);
	return Status;
}

FT_GEStatus Cmd_ScreenSaver(void)
{
	return ( WriteCmd(CMD_SCREENSAVER) );
}

FT_GEStatus Cmd_Scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t Options, uint16_t Val, uint16_t Size, uint16_t Range)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SCROLLBAR);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(((Val & 0xFFFFL)<<16) | (Options & 0xFFFFL));
	Status = WriteCmd(((Range & 0xFFFFL)<<16) | (Size & 0xFFFFL));
	return Status;
}

//make sure ptr is pointing to table and in turn table has pointer to the actual bitmap data
FT_GEStatus Cmd_SetFont(uint8_t Font, uint32_t Ptr)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SETFONT);
	WriteCmd(Font);
	Status = WriteCmd(Ptr);
	return Status;
}

FT_GEStatus Cmd_SetMatrix(void)
{
	return ( WriteCmd(CMD_SETMATRIX) );
}


FT_GEStatus Cmd_Sketch(int16_t x, int16_t y, uint16_t w, uint16_t h, uint32_t Ptr, uint16_t Format)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SKETCH);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(Ptr);
	Status = WriteCmd(Format);
	return Status;
}

FT_GEStatus Cmd_Slider(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t Options, uint16_t Val, uint16_t Range)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SLIDER);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(((Val & 0xFFFFL)<<16) | (Options & 0xFFFFL));
	Status = WriteCmd(Range);
	return Status;
}

//perform this api and wait for the completion
FT_GEStatus Cmd_Snapshot(uint32_t OutputAddr)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SNAPSHOT);
	Status = WriteCmd(OutputAddr);
	return Status;
}

//note that macro 0 and macro 1 are modified by the coprocessor when performing this function
FT_GEStatus Cmd_Spinner(int16_t x, int16_t y, uint8_t Style, uint8_t Scale)
{
	FT_GEStatus Status;
	WriteCmd(CMD_SPINNER);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	Status = WriteCmd(((Scale & 0xFFFFL)<<16) | (Style & 0xFFFFL));
	return Status;
}

FT_GEStatus Cmd_Stop(void)
{
	return ( WriteCmd(CMD_STOP) );
}

FT_GEStatus Cmd_Swap(void)
{
	return ( WriteCmd(CMD_SWAP) );
}

FT_GEStatus Cmd_Text(int16_t x, int16_t y, uint8_t Font, uint16_t Options, const char *s)
{
	FT_GEStatus Status;
	WriteCmd(CMD_TEXT);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((Options & 0xFFFFL)<<16) | (Font & 0xFFFFL));
	Status = WriteCmd_withSize((uint8_t *)s,strlen((const char *)s) + 1);
	return Status;
}

FT_GEStatus Cmd_Toggle(int16_t x, int16_t y, int16_t w, uint8_t Font, uint16_t Options, uint16_t State, const char *s)
{
	FT_GEStatus Status;
	WriteCmd(CMD_TOGGLE);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((Font & 0xFFFFL)<<16) | (w & 0xFFFFL));
	WriteCmd(((State & 0xFFFFL)<<16) | (Options & 0xFFFFL));
	Status = WriteCmd_withSize((uint8_t *)s,strlen((const char *)s) + 1);
	return Status;
}

FT_GEStatus Cmd_Track(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t Tag)
{
	FT_GEStatus Status;
	WriteCmd(CMD_TRACK);
	WriteCmd(((y & 0xFFFFL)<<16) | (x & 0xFFFFL));
	WriteCmd(((h & 0xFFFFL)<<16) | (w & 0xFFFFL));
	Status = WriteCmd(Tag);
	return Status;
}

FT_GEStatus Cmd_Translate(int32_t tx, int32_t ty)
{
	FT_GEStatus Status;

	WriteCmd(CMD_TRANSLATE);
	WriteCmd(tx);
	Status = WriteCmd(ty);
	return Status;
}

//Apis related to audio engine
FT_AEStatus PlaySound(uint8_t Volume,uint16_t SoundNote)
{
	Write(REG_VOL_SOUND,Volume);//change the volume of synthesized sound, 0 means off, 255 means max on
	wr16(REG_SOUND,SoundNote);
	Write(REG_PLAY,FT_SOUND_PLAY);
	return FT_AE_OK;
}
//higher byte is the note and lower byte is the sound
FT_AEStatus PlaySound_withoutVolume(uint16_t SoundNote)
{
	wr16(REG_SOUND,SoundNote);
	Write(REG_PLAY,FT_SOUND_PLAY);
	return FT_AE_OK;
}

//volume will not be modified
void StopSound(void)
{
	wr16(REG_SOUND,FT_SILENCE);																			//configure silence
	Write(REG_PLAY,FT_SOUND_PLAY);																		//play the silence
}

void SetSoundVolume(uint8_t Volume)
{
	Write(REG_VOL_SOUND,Volume);
}

uint8_t GetSoundVolume(void)
{
	return Read(REG_VOL_SOUND);
}

//one shot or continuous, sampling frequency is from 8k to 48k
FT_AEStatus PlayAudio(uint8_t Volume,uint8_t Format,uint16_t SamplingFreq,uint32_t BufferAddr,uint32_t BufferSize,uint8_t Loop)
{
	if((SamplingFreq*1L < FT_AUDIO_SAMPLINGFREQ_MIN*1L) | (SamplingFreq*1L > FT_AUDIO_SAMPLINGFREQ_MAX*1L))
	{
		return FT_AE_ERROR_SAMPLINGFREQ_OUTOFRANGE;
	}
	if(Format > FT_ADPCM_SAMPLES)
	{
		return FT_AE_ERROR_FORMAT;
	}
	Write(REG_VOL_PB,Volume);
	wr32(REG_PLAYBACK_START,BufferAddr);
	wr32(REG_PLAYBACK_LENGTH,BufferSize);
	wr32(REG_PLAYBACK_FREQ,SamplingFreq);
	Write(REG_PLAYBACK_FORMAT,Format);
	Write(REG_PLAYBACK_LOOP,Loop);//0 means one shot and 1 means loop
	Write(REG_PLAYBACK_PLAY,FT_AUDIO_PLAY);
	return FT_AE_OK;
}

void SetAudioVolume(uint8_t Volume)
{
	Write(REG_VOL_PB,Volume);
}

//returns playback stopped or continue
/*FT_AEStatus GetAudioStats(uint32_t &CurrPlayAddr)
{
	CurrPlayAddr = Read32(REG_PLAYBACK_READPTR);

	//in case of loop, check for the playback status
	if(0 == Read(REG_PLAYBACK_PLAY))
	{
		return FT_AE_PLAYBACK_STOPPED;
	}

	return FT_AE_PLAYBACK_CONTINUE;
}*/

uint8_t GetAudioVolume(void)
{
	return  (Read(REG_VOL_PB));
}

void StopAudio(void)
{
	wr32(REG_PLAYBACK_LENGTH,0);																		//configure audio with length o and play
	Write(REG_PLAYBACK_LOOP,0);																			//0 means one shot and 1 means loop
	Write(REG_PLAYBACK_PLAY,FT_AUDIO_PLAY);
}

//Apis related to touch engine
//one of 0ff/oneshot/frame/continuous. default being continuous
void SetTouchMode(uint8_t TMode)
{
	Write(REG_TOUCH_MODE,TMode);
}

//api to set coordinates for host specific tag query
/*void SetHostTagXY(uint16_t xoffset,uint16_t yoffset)
{
	uint8_t A[6];
	//little endian specific
	A[0] = xoffset & 0xFF;
	A[1] = xoffset >> 8;
	A[4] = yoffset & 0xFF;
	A[5] = yoffset >> 8;
	Write_withSize(REG_TAG_X,A,6);
}*/

//api to get TAG from FT_GC for coordinates set by  SetHostTagXY() api - host needs to wait for at least 1 frame to get these query values
uint8_t GetHostTagXY(void)
{
	return ( Read(REG_TAG) );
}

//get the touched object tag and repective xy coordinates
/*void GetTagXY(sTagXY &sTagxy)
{
	Read(REG_TOUCH_TAG_XY,(uint8_t *)&sTagxy,6);
}*/

//get the track value and the tag value
/*void GetTrackTag(sTrackTag &sTracktag)
{
	uint32_t *ptr = (uint32_t *)&sTracktag;
	*ptr = Read32(REG_TRACKER);
}*/



/************************************************************************************************************************************************************************
 * Function:        StartRead()
 * PreCondition:    None
 * Input:           Addr = address to read
 * Output:          None
 * Side Effects:    None
 * Overview:        Starts the process of reading FT800 memory
 ***********************************************************************************************************************************************************************/
void StartRead(unsigned long Addr)
{
	CS_LOW;
	SDATA = ((char)(Addr >> 16) | MEM_READ)<<SPI_SHIFT; 												// Send Memory Write plus high address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ((char)(Addr >> 8))<<SPI_SHIFT;																// Send middle address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = ((char)(Addr))<<SPI_SHIFT;																	// Send low address byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO

	SDATA = (ZERO);																						// Send dummy byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	RDATA;																								// Read data to clear FIFO
}


/************************************************************************************************************************************************************************
 * Function:        Read(Addr)
 * PreCondition:    None
 * Input:           Addr = address to read
 * Output:          ReadByte = read byte from FT800
 * Side Effects:    None
 * Overview:        Reads the data at the specified address
 ***********************************************************************************************************************************************************************/
uint8_t Read(unsigned long Addr)
{
	uint8_t ReadByte;
	StartRead(Addr);
	SDATA = (ZERO);																						// Read data byte
	while(SPI_NOT_RECEIVED) { }																			// Wait for data to be received
	ReadByte = RDATA;																					// Read data byte
	CS_HIGH;
	return (ReadByte);
}


/************************************************************************************************************************************************************************
 * Function:        Read_withSize(Addr, *Src, NBytes)
 * PreCondition:    None
 * Input:           Addr = address to read, *Src = , NBytes = size of data to read
 * Output:
 * Side Effects:    None
 * Overview:        Reads the data at the specified address of the FT800 with a given size
 ***********************************************************************************************************************************************************************/
void Read_withSize(uint32_t Addr, uint8_t *Src, uint32_t NBytes)
{
	uint32_t i;
	StartRead(Addr);
	for(i=0;i<NBytes;i++)
	{
		SDATA = (ZERO);																					// Read data byte
		while(SPI_NOT_RECEIVED) { }																		// Wait for data to be received
		*Src++ = RDATA;
	}
	CS_HIGH;
}


/************************************************************************************************************************************************************************
 * Function:        HostCommand(HostCommand)
 * PreCondition:    None
 * Input:           HostCommand = FT800 host command
 * Output:          None
 * Side Effects:    None
 * Overview:        Sends host command to FT800
 ***********************************************************************************************************************************************************************/
void HostCommand(uint32_t HostCommand)
{
	uint32_t Addr;
	Addr = (unsigned long)HostCommand<<16;																//construct host command and send to graphics controller
	Read(Addr);																							//ideally sending 3 bytes is sufficient
	DELAY_US(20);																						//worst scenario
}


/************************************************************************************************************************************************************************
 * Function:        UpdateFreeSpace()
 * PreCondition:    None
 * Input:           None
 * Output:          Status of graphics processor
 * Side Effects:    None
 * Overview:        Executes the current display list to free up space in coprocessor
 ***********************************************************************************************************************************************************************/
FT_GEStatus UpdateFreeSpace()
{
	if(TrnsFlag)
	{
		EndTransferCmd();
		wr16(REG_CMD_WRITE,CmdFifoWp);																	//update the write pointer
		StartTransferCmd();
	}
	else
	{
		wr16(REG_CMD_WRITE,CmdFifoWp); 																	//update the write pointer
	}
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        ChkGetFreeSpace(NBytes)
 * PreCondition:    None
 * Input:           NBytes = size of command
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        updates the FreeSpace variable
 ***********************************************************************************************************************************************************************/
FT_GEStatus ChkGetFreeSpace(uint16_t NBytes)
{
	if(FreeSpace < NBytes)																				//return busy if no space
	{
		if(TrnsFlag)
		{
			EndTransferCmd();
			TrnsFlag = 1;																				//because EndTransferCmd will make it 0
		}
		wr16(REG_CMD_WRITE,CmdFifoWp);																	//update the write pointer
		while(FreeSpace < NBytes)
		{
			uint16_t rdptr = rd16(REG_CMD_READ);
			if(rdptr == FT_COPRO_ERROR)
			{
				return FT_GE_ERROR;
			}
			FreeSpace = ((CmdFifoWp - rdptr)&0xffc);													//update the freespace by reading the register
			FreeSpace = (FT_CMDFIFO_SIZE - 4) - FreeSpace;
		}
		if(TrnsFlag)
		{
			StartTransferCmd();
		}
	}
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        StartTransferCmd()
 * PreCondition:    None
 * Input:           None
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        assert CSpin and send write command
 ***********************************************************************************************************************************************************************/
FT_GEStatus StartTransferCmd()
{
	StartWrite(FT_RAM_CMD + CmdFifoWp);																	//start write transaction
	TrnsFlag = 1;
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        TransferCmd(Cmd)
 * PreCondition:    None
 * Input:           Cmd = command to transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        Transfer the command if there is enough free space
 ***********************************************************************************************************************************************************************/
FT_GEStatus TransferCmd(unsigned long Cmd)
{
	if(FreeSpace <4)
	{
		if(FT_GE_ERROR == ChkGetFreeSpace(4))															//blocking call till freespace is available
		{
			return FT_GE_ERROR;
		}
	}
	Transfer32(Cmd);
	CmdFifoWp = (CmdFifoWp + 4)&0xfff;
	FreeSpace -= 4;

	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        TransferCmd_withSize(*Src, NBytes)
 * PreCondition:    None
 * Input:           *Src = data to transfer, NBytes = size of transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        Transfer the command if there is enough free space
 ***********************************************************************************************************************************************************************/
FT_GEStatus TransferCmd_withSize(uint8_t *Src,uint32_t NBytes)
{
	uint32_t i,Count;
	NBytes = (NBytes + 3)&(~3);																			//align the NBytes to multiple of 4
	while(NBytes)																						//transfer the whole buffer into command buffer
	{
		Count = NBytes;
		if(Count > FreeSpace)
		{
			UpdateFreeSpace();																			//first update the free space
			Count = min(FreeSpace,Count);																//then transfer the data
			for(i = 0;i<Count;i++)
				Transfer(*Src++);
			CmdFifoWp = (CmdFifoWp + Count)&0xfff;
			FreeSpace -= Count;
			NBytes -= Count;																			//get the free space
			Count = min(NBytes,FT_CMDFIFO_SIZE/2);														//atleast wait for half the buffer completion
			if(FT_GE_ERROR == ChkGetFreeSpace(Count))
			{
				return FT_GE_ERROR;
			}
		}
		else
		{
			for(i = 0;i<Count;i++)																		//transfer of data to command buffer
				Transfer(*Src++);
			CmdFifoWp = (CmdFifoWp + Count)&0xfff;
			FreeSpace -= Count;
			NBytes -= Count;
		}
	}
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        TransferCmdfromflash(*Src, NBytes)
 * PreCondition:    None
 * Input:           *Src = data to transfer, NBytes = size of transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        Transfer the command if there is enough free space
 ***********************************************************************************************************************************************************************/
FT_GEStatus TransferCmdfromflash( prog_uchar *Src,uint32_t NBytes)
{
	uint32_t i,Count;
	NBytes = (NBytes + 3)&(~3);																			//align the NBytes to multiple of 4
	while(NBytes)																						//transfer the whole buffer into command buffer
	{
		Count = NBytes;
		if(Count > FreeSpace)
		{
			UpdateFreeSpace();																			//first update the free space
			Count = min(FreeSpace,Count);
			for(i = 0;i<Count;i++)																		//then transfer the data
			{
				Transfer(pgm_read_byte_near(Src));
				Src++;
			}
			CmdFifoWp = (CmdFifoWp + Count)&0xfff;
			FreeSpace -= Count;
			NBytes -= Count;																			//get the free space
			Count = min(NBytes,FT_CMDFIFO_SIZE/2);														//atleast wait for half the buffer completion
			if(FT_GE_ERROR == ChkGetFreeSpace(Count))
			{
				return FT_GE_ERROR;
			}
		}
		else
		{
			for(i = 0;i<Count;i++)																		//transfer of data to command buffer
			{
				Transfer(pgm_read_byte_near(Src));
				Src++;
			}
			CmdFifoWp = (CmdFifoWp + Count)&0xfff;
			FreeSpace -= Count;
			NBytes -= Count;
		}
	}
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        EndTransferCmd()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        De-assert CS line and execute display list
 ***********************************************************************************************************************************************************************/
void EndTransferCmd(void)
{
	CS_HIGH;
	wr32(REG_CMD_WRITE,CmdFifoWp);																		//update the write pointer of fifo
	TrnsFlag = 0;
}

/************************************************************************************************************************************************************************
 * Function:        WriteCmd(Cmd)
 * PreCondition:    None
 * Input:           Cmd = data to transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        Start writing a command
 ***********************************************************************************************************************************************************************/
FT_GEStatus WriteCmd(unsigned long Cmd)
{
	FT_GEStatus Status;
#if 0
	if(0 == TrnsFlag)
		StartTransferCmd();
	Status = TransferCmd(Cmd);
	if(0 == TrnsFlag)
		EndTransferCmd();
#else
	if(0 == TrnsFlag)
	{
		StartTransferCmd();
		Status = TransferCmd(Cmd);
		EndTransferCmd();
	}
	else
	{
		Status = TransferCmd(Cmd);
	}

#endif
	return Status;
}


/************************************************************************************************************************************************************************
 * Function:        WriteCmd_withSize(*Src, NBytes)
 * PreCondition:    None
 * Input:           *Src = data to transfer, NBytes = size of transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        start the write command with the size
 ***********************************************************************************************************************************************************************/
FT_GEStatus WriteCmd_withSize(uint8_t *Src,uint32_t NBytes)
{
	FT_GEStatus Status;
	if(0 == TrnsFlag)
	{
		StartTransferCmd();
		Status = TransferCmd_withSize(Src,NBytes);
		EndTransferCmd();																				//here transflag is made to 0
	}
	else
		Status = TransferCmd_withSize(Src,NBytes);

	return Status;
}


/************************************************************************************************************************************************************************
 * Function:        WriteCmdfromflash(*Src, NBytes)
 * PreCondition:    None
 * Input:           *Src = data to transfer, NBytes = size of transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        start the write command with the sizes
 ***********************************************************************************************************************************************************************/
FT_GEStatus WriteCmdfromflash( prog_uchar *Src,uint32_t NBytes)
{
	FT_GEStatus Status;
	if(0 == TrnsFlag)
	{
		StartTransferCmd();
		Status =  TransferCmdfromflash(Src,NBytes);
		EndTransferCmd();																				//here transflag is made to 0
	}
	else
		Status = TransferCmdfromflash(Src,NBytes);

	return Status;
}

//reads the result of the previous commands such as cmd_memcrc,cmd_calibration, cmd_regread which has return values. if busy returns busy status
/*FT_GEStatus Cmd_GetResult(uint32_t &Result)
{
	Result = Read32(FT_RAM_CMD + ((CmdFifoWp - 4)&0xFFC));//make sure no other commands are issued after cmd_memcrc/cmd_calibration/cmd_regread

	return FT_GE_OK;
}*/


/************************************************************************************************************************************************************************
 * Function:        Cmd_GetResults(*pA, NBytes)
 * PreCondition:    None
 * Input:           *pA = pointer to results, NBytes = size of transfer
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        api to read n bytes from the current write pointer location
					make sure no other commands are issued after cmd_memcrc/cmd_calibration/cmd_regread
					assumed that number of bytes in array is allocated/managed by application
 ***********************************************************************************************************************************************************************/
FT_GEStatus Cmd_GetResults(int8_t *pA, int32_t NBytes)
{
	if((CmdFifoWp - NBytes) < 0)																		//handling of circular buffer
	{
		uint16_t ReadLen = NBytes - CmdFifoWp;
		Read_withSize((FT_RAM_CMD + (FT_CMDFIFO_SIZE - ReadLen)),(uint8_t *)pA,ReadLen);				//first read the end bytes
		pA += ReadLen;
		ReadLen = NBytes - ReadLen;
		Read_withSize(FT_RAM_CMD,(uint8_t *)pA,ReadLen);												//later read the starting bytes
	}
	else
	{
		Read_withSize(FT_RAM_CMD + (CmdFifoWp - NBytes),(uint8_t *)pA,NBytes);
	}

	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        DLStart()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        API to make starting a display list easier
 ***********************************************************************************************************************************************************************/
void DLStart(void)
{
	Cmd_DLStart();
	Clear(1,1,1);
}


/************************************************************************************************************************************************************************
 * Function:        DLEnd()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Sends the Display command followed to the swap command to update the display
 ***********************************************************************************************************************************************************************/
void DLEnd(void)
{
	Display();
	Cmd_Swap();
}


/************************************************************************************************************************************************************************
 * Function:        CheckLogo()
 * PreCondition:    None
 * Input:           None
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        Checks to make sure logo generation is complete (built in FTDI Logo)
 ***********************************************************************************************************************************************************************/
FT_GEStatus CheckLogo(void)
{
	uint16_t ReadCmdPtr = rd16(REG_CMD_READ) ;
	if((ReadCmdPtr == rd16(REG_CMD_WRITE)) && (ReadCmdPtr == 0))
	{
		CmdFifoWp = 0;
		return FT_GE_FINISHED;
	}
	else if(FT_COPRO_ERROR == ReadCmdPtr)
	{
		return FT_GE_ERROR;
	}
	else
	{
		return FT_GE_BUSY;
	}

}


/************************************************************************************************************************************************************************
 * Function:        Flush()
 * PreCondition:    None
 * Input:           None
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        flush out all the commands to FT_GC, does not wait for the completion of the rendering
 ***********************************************************************************************************************************************************************/
FT_GEStatus Flush(void)
{
	if(TrnsFlag)
	{
		EndTransferCmd();
	}
	wr16(REG_CMD_WRITE,CmdFifoWp);

	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        Finish()
 * PreCondition:    None
 * Input:           None
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        flushes out all the commands to FT_GC and waits for the completion of execution
 ***********************************************************************************************************************************************************************/
FT_GEStatus Finish(void)
{
	uint16_t ReadPrt;

	if(TrnsFlag)
	{
		EndTransferCmd();
	}
	wr16(REG_CMD_WRITE,CmdFifoWp);

	while((ReadPrt = rd16(REG_CMD_READ)) != CmdFifoWp)
	{
		if(FT_COPRO_ERROR == ReadPrt)
		{
			return FT_GE_ERROR;
		}
	}
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        CheckFinish()
 * PreCondition:    None
 * Input:           None
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        checks fifo and returns the status
 ***********************************************************************************************************************************************************************/
FT_GEStatus CheckFinish(void)
{
	uint16_t ReadPrt = rd16(REG_CMD_READ);


	if (FT_COPRO_ERROR == ReadPrt)
	{
		return FT_GE_ERROR;
	}
	else if(ReadPrt != CmdFifoWp)
	{
		return FT_GE_BUSY;
	}	//success case return finished
	return FT_GE_FINISHED;
}


/************************************************************************************************************************************************************************
 * Function:        GetError()
 * PreCondition:    None
 * Input:           None
 * Output:          status of graphics processor
 * Side Effects:    None
 * Overview:        error from graphics controller library
 ***********************************************************************************************************************************************************************/
uint32_t GetError(void)
{
	if(FT_COPRO_ERROR == rd16(REG_CMD_READ))
	{
		return FT_GE_ERROR;
	}
	return FT_GE_OK;
}


/************************************************************************************************************************************************************************
 * Function:        Writefromflash(Addr, *Src, NBytes)
 * PreCondition:    None
 * Input:           Addr = address, *Src = data to write, NBytes = size of data
 * Output:          None
 * Side Effects:    None
 * Overview:        Write data from flash to FT800 address
 ***********************************************************************************************************************************************************************/
void Writefromflash(unsigned long Addr, const unsigned char *Src, uint32_t NBytes)
{
	uint32_t i;
	StartWrite(Addr);
	for(i=0;i<NBytes;i++)
	{
		SDATA = (pgm_read_byte_near(Src))<<SPI_SHIFT;													// Send byte
		while(SPI_NOT_RECEIVED) { }																		// Wait for data to be received
		RDATA;
		Src++;
	}
	CS_HIGH;
}


/************************************************************************************************************************************************************************
 * Function:        ft800_spi_init()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Initializes the SPI functionality of the desired platform
 * 					To change platforms see FT800.h
 ***********************************************************************************************************************************************************************/
void ft800_spi_init(void)
{
#ifdef SPIA_28069
	SpiaRegs.SPICCR.all =0x0007;	             														// Reset on, rising edge, (8-bit char bits, 1 byte, 0x0007)

	SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;		 														// Disable Overrun interrupt
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;           														// Clock Delayed 1/2 cycle
	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;    	 														// Enable master mode
	SpiaRegs.SPICTL.bit.TALK = 1;					 													// Transmission enabled
	SpiaRegs.SPICTL.bit.SPIINTENA = 0;			 														// Disable SPI interrupt


	SpiaRegs.SPIBRR =0x0005;
	SpiaRegs.SPICCR.all =0x0087;		         														// Relinquish SPI from Reset, loopback mode disabled, (8-bits, 1 byte, 0x0007), rising edge
	SpiaRegs.SPIPRI.bit.FREE = 1;                														// Set so breakpoints don't disturb xmission
#endif
#ifdef SPIB_28069
	SpibRegs.SPICCR.all =0x0007;	             														// Reset on, rising edge, (8-bit char bits, 1 byte, 0x0007)

	SpibRegs.SPICTL.bit.OVERRUNINTENA = 0;		 														// Disable Overrun interrupt
	SpibRegs.SPICTL.bit.CLK_PHASE = 1;           														// Clock Delayed 1/2 cycle
	SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;    	 														// Enable master mode
	SpibRegs.SPICTL.bit.TALK = 1;				 														// Transmission enabled
	SpibRegs.SPICTL.bit.SPIINTENA = 0;			 														// Disable SPI interrupt


	SpibRegs.SPIBRR =0x0005;
	SpibRegs.SPICCR.all =0x0087;		         														// Relinquish SPI from Reset, loopback mode disabled, (8-bits, 1 byte, 0x0007), rising edge
	SpibRegs.SPIPRI.bit.FREE = 1;                														// Set so breakpoints don't disturb xmission
#endif
#ifdef McBSPA_28335
	EALLOW;
	McbspaRegs.SPCR1.all=0x0000;		 																// Reset Receiver, Right justify word, Digital loopback dis.
    McbspaRegs.SPCR1.bit.DLB = 0;
    McbspaRegs.SPCR1.bit.CLKSTP = 3;     																// Together with CLKXP/CLKRP determines clocking scheme
    McbspaRegs.SPCR1.bit.RRST=1;         																// Release RX from Reset

    McbspaRegs.SPCR2.all=0x0000;		 																// Reset FS generator, sample rate generator & transmitter
    McbspaRegs.SPCR2.bit.GRST=1;         																// Enable the sample rate generator
	McbspaRegs.SPCR2.bit.XRST=1;         																// Release TX from Reset
    McbspaRegs.SPCR2.bit.FRST=1;         																// Frame Sync Generator reset

    McbspaRegs.PCR.all=0x0F08;           																//(CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
    McbspbRegs.PCR.bit.SCLKME = 0;																		// 0: McBSP clock generator derived from LSPCLK
	McbspaRegs.PCR.bit.CLKXP = 0;		 																// CPOL = 0, CPHA = 0 rising edge no delay
	McbspaRegs.PCR.bit.CLKRP = 0;

	McbspaRegs.SRGR1.all= 0x0004;	     																// Frame Width = 1 CLKG period, CLKGDV=4
	McbspaRegs.SRGR2.all=0x2000; 	 	 																// CLKSM=1, FPER = 1 CLKG periods
	McbspbRegs.SRGR2.bit.CLKSM = 1;																		// 1: McBSP clock generator derived from LSPCLK
    McbspaRegs.RCR2.bit.RDATDLY=01;      																// FSX setup time 1 in master mode. 0 for slave mode (Receive)
    McbspaRegs.XCR2.bit.XDATDLY=01;     	 															// FSX setup time 1 in master mode. 0 for slave mode (Transmit)

	McbspaRegs.RCR1.bit.RWDLEN1=0;     																	// 8-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=0;     																	// 8-bit word
    EDIS;
   /*
	EALLOW;
   	McbspbRegs.SPCR1.all=0x0000;		// Reset Receiver
	McbspbRegs.SPCR1.bit.CLKSTP = 3;	// 3: Clockstop mode set to inactive low.
										// McBSP transmits data 1/2 cycle ahead of CLKX
	McbspbRegs.SPCR1.bit.RRST = 1;		// 1: Enables receiver

	McbspbRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
 	McbspbRegs.SPCR2.bit.FREE  = 1;		// Free run in break event
	McbspbRegs.SPCR2.bit.GRST = 1;		// 1: Enables sample rate generator
	McbspbRegs.SPCR2.bit.XRST = 1;		// 1: Enables transmitter
	McbspbRegs.SPCR2.bit.FRST=1;		// 1: release frame logic from reset

	McbspbRegs.PCR.bit.CLKXP = 0;			// 0: Transmit data sampled on rising edge of CLKX
	McbspbRegs.PCR.bit.CLKRP = 0;			// 1: Receive data sampled on falling edge of MCLKR
	McbspbRegs.PCR.bit.CLKXM = 1;			// 1: McBSP is master in SPI-mode;
											//    CLKX drives pin MCLKX and MCLKR(internally)
	McbspbRegs.PCR.bit.SCLKME = 0;			// 0: McBSP clock generator derived from LSPCLK
	McbspbRegs.PCR.bit.FSXM = 1;			// 1: Transmit frame sync generated internally by CLKG
	McbspbRegs.PCR.bit.FSXP = 1;			// 1: Transmit frame sync pulses are active low

	McbspbRegs.SRGR1.bit.CLKGDV = 37;		// 37: CLKG = LSPCLK/(37+1) = 1 MHz on McBSP

	McbspbRegs.SRGR2.bit.CLKSM = 1;			// 1: McBSP clock generator derived from LSPCLK
	McbspbRegs.SRGR2.bit.FPER = 1;
	McbspbRegs.SRGR2.bit.FSGM = 0;			// 0: McBSP generates a frame sync when DXR is copied into XSR

	McbspbRegs.XCR1.bit.XFRLEN1 = 0;		// 0: One word per frame for transmit
	McbspbRegs.XCR1.bit.XWDLEN1 = 0;		// 0: 8 bit transmit word

	McbspbRegs.XCR2.bit.XPHASE = 0;			// 0: Single phase transmit
	McbspbRegs.XCR2.bit.XDATDLY = 0;		// 0: No data delay between sync and first data bit

 	McbspbRegs.RCR1.bit.RFRLEN1 = 0;		// 0: One word per frame for receive
	McbspbRegs.RCR1.bit.RWDLEN1 = 0;		// 0: 8 bit receive word

	McbspbRegs.RCR2.bit.RDATDLY = 0;		// 0: No data delay because FSX not needed for chip select
	McbspbRegs.RCR2.bit.RPHASE = 0;			// 0: Single phase receive

    EDIS;*/
#endif
}


/************************************************************************************************************************************************************************
 * Function:        ft800_spi_fifo_init()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Initializes the SPI FIFO functionality of the desired platform
 * 					To change platforms see FT800.h
 ***********************************************************************************************************************************************************************/
void ft800_spi_fifo_init(void)
{
// Initialize SPI FIFO registers
#ifdef SPIA_28069
	SpiaRegs.SPIFFTX.all=0xE040;
	SpiaRegs.SPIFFRX.all=0x2044;
	SpiaRegs.SPIFFCT.all=0x0;
#endif
#ifdef SPIB_28069
	SpibRegs.SPIFFTX.all=0xE040;
	SpibRegs.SPIFFRX.all=0x2044;
	SpibRegs.SPIFFCT.all=0x0;
#endif

}


/************************************************************************************************************************************************************************
 * Function:        setup_spi_gpio()
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Initializes the GPIO's used for SPI functionality of the desired platform
 * 					To change platforms see FT800.h
 ***********************************************************************************************************************************************************************/
void setup_spi_gpio(void)
{
#ifdef SPIA_28069
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO16=0;   																// Enable pull-up on GPIO16 (SPISIMOA)
	GpioCtrlRegs.GPAPUD.bit.GPIO17=0;   																// Enable pull-up on GPIO17 (SPISOMIA)
	GpioCtrlRegs.GPAPUD.bit.GPIO18=0;   																// Enable pull-up on GPIO18 (SPICLKA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO16=3; 																// Asynch input GPIO24 (SPISIMOA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO17=3; 																// Asynch input GPIO25 (SPISOMIA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO18=3; 																// Asynch input GPIO26 (SPICLKA)
	GpioCtrlRegs.GPAMUX2.bit.GPIO16=3; 																	// Configure GPIO24 as SPISIMOA
	GpioCtrlRegs.GPAMUX2.bit.GPIO17=3; 																	// Configure GPIO25 as SPISOMIA
	GpioCtrlRegs.GPAMUX2.bit.GPIO18=3; 																	// Configure GPIO26 as SPICLKA
	GpioCtrlRegs.GPAMUX2.bit.GPIO27=0; 																	// Configure GPIO27 as Manual Chip Select SPISTEA
	GpioCtrlRegs.GPBMUX1.bit.GPIO32=0;  																// FT800 Display INT Pin
	GpioCtrlRegs.GPBMUX1.bit.GPIO33=0;  																// FT800 Display Power Pin
	GpioCtrlRegs.GPADIR.bit.GPIO27=1;																	// set GPIO as output manual chip select for SPI
	GpioCtrlRegs.GPBDIR.bit.GPIO32=1;   																// set GPIO as output
	GpioCtrlRegs.GPBDIR.bit.GPIO33=1;   																// set GPIO as output, FT800 Power Pin & LD1
	GpioDataRegs.GPASET.bit.GPIO27=1; 																	// Set high initially
	GpioDataRegs.GPBSET.bit.GPIO33=1;
	EDIS;
#endif

#ifdef SPIB_28069
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO24=0;   																// Enable pull-up on GPIO24 (SPISIMOB)
	GpioCtrlRegs.GPAPUD.bit.GPIO25=0;   																// Enable pull-up on GPIO25 (SPISOMIB)
	GpioCtrlRegs.GPAPUD.bit.GPIO26=0;   																// Enable pull-up on GPIO26 (SPICLKB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO24=3; 																// Asynch input GPIO24 (SPISIMOB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO25=3; 																// Asynch input GPIO25 (SPISOMIB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26=3; 																// Asynch input GPIO26 (SPICLKB)
	GpioCtrlRegs.GPAMUX2.bit.GPIO24=3; 																	// Configure GPIO24 as SPISIMOB
	GpioCtrlRegs.GPAMUX2.bit.GPIO25=3; 																	// Configure GPIO25 as SPISOMIB
	GpioCtrlRegs.GPAMUX2.bit.GPIO26=3; 																	// Configure GPIO26 as SPICLKB
	GpioCtrlRegs.GPAMUX2.bit.GPIO27=0; 																	// Configure GPIO27 as Manual Chip Select SPISTEB
	GpioCtrlRegs.GPBMUX1.bit.GPIO32=0;  																// FT800 Display INT Pin
	GpioCtrlRegs.GPBMUX1.bit.GPIO33=0;  																// FT800 Display Power Pin
	GpioCtrlRegs.GPADIR.bit.GPIO27=1;																	// set GPIO as output manual chip select for SPI
	GpioCtrlRegs.GPBDIR.bit.GPIO32=1;   																// set GPIO as output
	GpioCtrlRegs.GPBDIR.bit.GPIO33=1;   																// set GPIO as output, FT800 Power Pin & LD1
	GpioDataRegs.GPASET.bit.GPIO27=1; 																	// Set high initially
	GpioDataRegs.GPBSET.bit.GPIO33=1;
	EDIS;
#endif

#ifdef McBSPA_28335
	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;																// GPIO20 is MDXA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;																// GPIO21 is MDRA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;																// GPIO22 is MCLKXA pin
	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;     															// Enable pull-up on GPIO20 (MDXA)(SPIMOSI)
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     															// Enable pull-up on GPIO21 (MDRA)(SPIMISO)
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;     															// Enable pull-up on GPIO22 (MCLKXA)(SPICLK)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;   															// Asynch input GPIO21 (MDRA)(SPIMISO)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 3;   															// Asynch input GPIO22 (MCLKXA)(SPICLK)
	GpioCtrlRegs.GPAMUX2.bit.GPIO23=0; 																	// Configure GPIO30 as Manual Chip Select SPICS
	//GpioCtrlRegs.GPBMUX1.bit.GPIO32=0;  																// FT800 Display INT Pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO18=0;  																// FT800 Display Power Pin
	GpioCtrlRegs.GPADIR.bit.GPIO23=1;																	// set GPIO as output manual chip select for SPI
	GpioCtrlRegs.GPADIR.bit.GPIO18=1;   																// set GPIO as output, FT800 Power Pin & LD1
	//GpioCtrlRegs.GPBDIR.bit.GPIO32=1;   																// set GPIO as output
	GpioDataRegs.GPASET.bit.GPIO23=1; 																	// Set high initially
	GpioDataRegs.GPASET.bit.GPIO18=1;
	EDIS;
#endif
}
