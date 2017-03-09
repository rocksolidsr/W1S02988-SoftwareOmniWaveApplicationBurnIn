/*
 * FT800.h
 *
 *  Created on: Feb 2, 2015
 *      Author: srock
 */
#ifndef FT800_H_
#define FT800_H_

#include "..\..\project.h"

//Uncomment the specified platform for use with the FT800
//#define SPIA_28069
//#define SPIB_28069
//#define McBSPA_28335

#ifdef SPIA_28069
#define SDATA SpiaRegs.SPITXBUF
#define CS_HIGH	GpioDataRegs.GPASET.bit.GPIO27 	 = 1
#define CS_LOW	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1
#define RDATA 	SpiaRegs.SPIRXBUF
#define SPI_NOT_RECEIVED SpiaRegs.SPIFFRX.bit.RXFFST !=1
#define TURN_ON_DISP 	GpioDataRegs.GPBSET.bit.GPIO33 		= 1
#define TURN_OFF_DISP 	GpioDataRegs.GPBCLEAR.bit.GPIO33 	= 1
#define SPI_SHIFT 8
#endif

#ifdef SPIB_28069
#define SDATA SpibRegs.SPITXBUF
#define CS_HIGH	GpioDataRegs.GPASET.bit.GPIO27 	 = 1
#define CS_LOW	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1
#define DISPLAY_CS_HIGH GpioDataRegs.GPASET.bit.GPIO19 	 = 1
#define DISPLAY_CS_LOW GpioDataRegs.GPACLEAR.bit.GPIO19 	 = 1
#define RDATA 	SpibRegs.SPIRXBUF
#define SPI_NOT_RECEIVED SpibRegs.SPIFFRX.bit.RXFFST !=1
#define TURN_ON_DISP 	GpioDataRegs.GPBSET.bit.GPIO33 		= 1
#define TURN_OFF_DISP 	GpioDataRegs.GPBCLEAR.bit.GPIO33 	= 1
#define SPI_SHIFT 8
#endif

#ifdef McBSPA_28335
#define SDATA McbspaRegs.DXR1.all
#define CS_HIGH	GpioDataRegs.GPASET.bit.GPIO23 	 = 1
#define CS_LOW	GpioDataRegs.GPACLEAR.bit.GPIO23 = 1
#define RDATA 	McbspaRegs.DRR1.all
#define SPI_NOT_RECEIVED McbspaRegs.SPCR1.bit.RRDY == 0
#define TURN_ON_DISP 	GpioDataRegs.GPASET.bit.GPIO18 		= 1
#define TURN_OFF_DISP 	GpioDataRegs.GPACLEAR.bit.GPIO18 	= 1
#define SPI_SHIFT 0
#endif

typedef char char8_t;
typedef signed char schar8_t;
typedef unsigned char uchar8_t;
typedef uchar8_t uint8_t;
typedef int  int16_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef long int32_t;
typedef void void_t;
typedef long long int64_t;
typedef unsigned long long uint64_t;
typedef float float_t;
typedef double double_t;
typedef char bool_t;
typedef const unsigned char prog_uchar;
typedef signed char int8_t;


//FT_GC status enum - used for api return type, error type etc
typedef enum FT_Status
{
	FT_OK = 0,																							// Common enums
	FT_ERROR = 1,
	FT_WARNING = 2,
	FT_ERROR_INIT = 3,
	FT_ERROR_CHIPID = 4,

	FT_ERROR_NOPINASSIGNED = 50																			// Library related enums
}FT_Status;

//Audio coprocessor related enums
typedef enum FT_AEStatus
{
	FT_AE_OK = 0,
	FT_AE_ERROR_FORMAT = 1,
	FT_AE_ERROR_SAMPLINGFREQ_OUTOFRANGE = 2,															// assert for boundary
	FT_AE_PLAYBACK_STOPPED = 3,
	FT_AE_PLAYBACK_CONTINUE = 4
}FT_AEStatus;

//Status enums for graphics engine
typedef enum FT_GEStatus
{
	FT_GE_OK = 0,
	FT_GE_BUSY = 1,
	FT_GE_FINISHED = 2,

	FT_GE_ERROR_INVALID_PRIMITIVE = 20,																	// Graphics related error enums
	FT_GE_ERROR_INVALID_BITMAP_FORMAT = 21,
	FT_GE_ERROR_INVALID_BITMAP_HANDLENUM = 22,
	FT_GE_ERROR_VERTEX_OUTOFRANGE = 23,

	// Coprocessor related enums */
	FT_GE_ERROR = 50,																					//undefined error
	FT_GE_ERROR_JPEG = 51,																				//erranious jpeg data
	FT_GE_ERROR_DEFLATE = 52,																			//erranious deflated data
	FT_GE_ERROR_DISPLAYLIST_OVERFLOW = 53,																//DL buffer overflow
	FT_GE_ERROR_INVALID_WIDGET_PARAMS = 54,																//invalid input parameters - out of bound
	FT_GE_ERROR_DISPLAYPARAMS = 100																		//error in the display parameters
}FT_GEStatus;

//Touch coprocessor related enums
typedef enum FT_TEStatus
{
	FT_TE_OK = 0,
	FT_TE_ERROR_RZTHRESHOLD = 1,																		//threshold out of bound
	FT_TE_ERROR_FILTERPARAM = 2,																		//filter out of bound
	FT_TE_ERROR_MODE = 3,																				//mode out of range
	FT_TE_ERROR_INVALIDPARAM = 4																		//generic invalid param
}FT_TEStatus;

void 			wr8(unsigned long ftAddress, unsigned char ftData8);
void 			wr16(unsigned long ftAddress, unsigned int data);
void 			wr32(unsigned long ftAddress, unsigned long ftData32);
unsigned int 	rd16(unsigned long ftAddress);
unsigned long 	rd32(unsigned long ftAddress);

//Apis related to power up/power down functionality
void DisplayOn(void);																					//Apis to enable/disable backlight
void DisplayOff(void);
void BacklightOn(void);
void BacklightOff(void);
void AudioOn(void);
void AudioOff(void);
void SetInterruptPin(uint16_t Intpin);																	//apis to set interrupt pin
void ResetCopro(void);																					//api to reset only coprocessor
void Reset(void);																						//api to reset whole FT_GC via pdn
void DisplayConfigExternalClock(uint8_t ResType);
void ActiveInternalClock(void);
void PDN_Cycle(void);

//Apis related to graphics processor
void 	EnableInterrupts_FT(uint8_t GEnable,uint8_t Mask);
uint8_t ReadIntReg(void);																				//read the interrupt flag register - note that on FT_GC the interrupts are clear by read

//APIs related to graphics engine
FT_GEStatus AlphaFunc(uint8_t Func, uint8_t Ref);
FT_GEStatus Begin(uint8_t Prim);
FT_GEStatus BitmapHandle(uint8_t Handle);
FT_GEStatus BitmapLayout(uint8_t Format, uint16_t Linestride, uint16_t Height);
FT_GEStatus BitmapSize(uint8_t Filter, uint8_t wrapx, uint8_t wrapy, uint16_t width, uint16_t height);
FT_GEStatus BitmapSource(signed long Addr);
FT_GEStatus BitmapTransformA(int32_t A);
FT_GEStatus BitmapTransformB(int32_t B);
FT_GEStatus BitmapTransformC(int32_t C);
FT_GEStatus BitmapTransformD(int32_t D);
FT_GEStatus BitmapTransformE(int32_t E);
FT_GEStatus BitmapTransformF(int32_t F);
FT_GEStatus BlendFunc(uint8_t Src, uint8_t Dst);
FT_GEStatus Call(uint16_t Dest);
FT_GEStatus Cell(uint8_t Cell);
FT_GEStatus ClearColorA(uint8_t Alpha);
FT_GEStatus ClearColorRGB(uint8_t red, uint8_t green, uint8_t blue);
FT_GEStatus ClearColorRGB_Single(uint32_t rgb);
FT_GEStatus Clear(uint8_t c, uint8_t s, uint8_t t);
FT_GEStatus Clear_Void(void);
FT_GEStatus ClearStencil(uint8_t s);
FT_GEStatus ClearTag(uint8_t s);
FT_GEStatus ColorA(uint8_t Alpha);
FT_GEStatus ColorMask(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
FT_GEStatus ColorRGB(uint8_t red, uint8_t green, uint8_t blue);
FT_GEStatus Display(void);
FT_GEStatus End(void);
FT_GEStatus Jump(uint16_t Dest);
FT_GEStatus LineWidth(uint16_t Width);
FT_GEStatus Macro(uint8_t m);
FT_GEStatus PointSize(uint16_t Size);
FT_GEStatus RestoreContext(void);
FT_GEStatus Return(void);
FT_GEStatus SaveContext(void);
FT_GEStatus ScissorSize(uint16_t width, uint16_t height);
FT_GEStatus ScissorXY(uint16_t x, uint16_t y);
FT_GEStatus StencilFunc(uint8_t Func, uint8_t Ref, uint8_t Mask);
FT_GEStatus StencilMask(uint8_t Mask);
FT_GEStatus StencilOp(uint8_t Sfail, uint8_t Spass);
FT_GEStatus TagMask(uint8_t Mask);
FT_GEStatus Tag(uint8_t s);
FT_GEStatus Vertex2f(int16_t x, int16_t y);
FT_GEStatus Vertex2ii(uint16_t x, uint16_t y, uint8_t Handle, uint8_t Cell);
FT_GEStatus ColorRGB_Single(uint32_t rgb);
FT_GEStatus ColorARGB(unsigned long argb);

//APIs related to coprocessor commands, widgets etc
FT_GEStatus Cmd_Logo(void);
FT_GEStatus Cmd_Append(uint32_t Ptr, uint32_t Num);
FT_GEStatus Cmd_BGColor(uint32_t c);
FT_GEStatus Cmd_Button(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t Font, uint16_t Options, const char *s);
FT_GEStatus Cmd_Calibrate(uint32_t Result);
FT_GEStatus Cmd_Clock(int16_t x, int16_t y, int16_t r, uint16_t Options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms);
FT_GEStatus Cmd_ColdStart(void);
FT_GEStatus Cmd_Dial(int16_t x, int16_t y, int16_t r, uint16_t Options, uint16_t Val);
FT_GEStatus Cmd_DLStart(void);
FT_GEStatus Cmd_FGColor(uint32_t c);
FT_GEStatus Cmd_Gauge(int16_t x, int16_t y, int16_t r, uint16_t Options, uint16_t Major, uint16_t Minor, uint16_t Val, uint16_t Range);
FT_GEStatus Cmd_GetMatrix(void);
//FT_GEStatus Cmd_GetProps(uint32_t &Ptr, uint32_t &w, uint32_t &h);
FT_GEStatus Cmd_GetPtr(uint32_t Result);
FT_GEStatus Cmd_GradColor(uint32_t c);
FT_GEStatus Cmd_Gradient(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1);
FT_GEStatus Cmd_Inflate(uint32_t Ptr);
FT_GEStatus Cmd_Interrupt(uint32_t ms);
FT_GEStatus Cmd_Keys(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t Font, uint16_t Options, const char *s);
FT_GEStatus Cmd_LoadIdentity(void);
FT_GEStatus Cmd_LoadImage(uint32_t Ptr, int32_t Options);
FT_GEStatus Cmd_Memcpy(uint32_t Dest, uint32_t Src, uint32_t Num);
FT_GEStatus Cmd_Memset(uint32_t Ptr, uint8_t Value, uint32_t Num);
//FT_GEStatus Cmd_Memcrc(uint32_t Ptr, uint32_t Num,uint32_t &Result);
FT_GEStatus Cmd_Memwrite(uint32_t Ptr, uint32_t Num);
FT_GEStatus Cmd_Memzero(uint32_t Ptr, uint32_t Num);
FT_GEStatus Cmd_Number(int16_t x, int16_t y, uint8_t Font, uint16_t Options, int32_t n);
FT_GEStatus Cmd_Progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t Options, uint16_t Val, uint16_t Range);
FT_GEStatus Cmd_RegRead(uint32_t Ptr,uint32_t Result);
FT_GEStatus Cmd_Rotate(int32_t a);
FT_GEStatus Cmd_Scale(int32_t sx, int32_t sy);
FT_GEStatus Cmd_ScreenSaver(void);
FT_GEStatus Cmd_Scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t Options, uint16_t Val, uint16_t Size, uint16_t Range);
FT_GEStatus Cmd_SetFont(uint8_t Font, uint32_t Ptr);
FT_GEStatus Cmd_SetMatrix(void);
FT_GEStatus Cmd_Sketch(int16_t x, int16_t y, uint16_t w, uint16_t h, uint32_t Ptr, uint16_t Format);
FT_GEStatus Cmd_Slider(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t Options, uint16_t val, uint16_t Range);
FT_GEStatus Cmd_Snapshot(uint32_t OutputAddr);
FT_GEStatus Cmd_Spinner(int16_t x, int16_t y, uint8_t Style, uint8_t Scale);
FT_GEStatus Cmd_Stop(void);
FT_GEStatus Cmd_Swap(void);
FT_GEStatus Cmd_Text(int16_t x, int16_t y, uint8_t font, uint16_t Options, const char *s);
FT_GEStatus Cmd_Toggle(int16_t x, int16_t y, int16_t w, uint8_t font, uint16_t Options, uint16_t State, const char *s);
FT_GEStatus Cmd_Track(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t Tag);
FT_GEStatus Cmd_Translate(int32_t tx, int32_t ty);

//Apis related to audio engine
void 		StopSound(void);																			//volume will not be modified
void 		SetSoundVolume(uint8_t Volume);
void 		SetAudioVolume(uint8_t Volume);
void 		StopAudio(void);																			//volume will not be modified
uint8_t 	GetSoundVolume(void);
uint8_t 	GetAudioVolume(void);
FT_AEStatus PlaySound(uint8_t Volume,uint16_t SoundNote);
FT_AEStatus PlaySound_withoutVolume(uint16_t SoundNote);												//higher byte is the note and lower byte is the sound
FT_AEStatus PlayAudio(uint8_t Volume,uint8_t Format,uint16_t SamplingFreq,
					  uint32_t BufferAddr,uint32_t BufferSize,uint8_t Loop);							//one shot or continuous, sampling frequency is from 8k to 48k
//FT_AEStatus GetAudioStats(uint32_t &CurrPlayAddr);//returns playback stopped or continue

//Apis related to touch engine
void 	SetTouchMode(uint8_t TMode);																	//one of 0ff/oneshot/frame/continuous. default being continuous
void 	SetHostTagXY(uint16_t xoffset,uint16_t yoffset);												//api to set coordinates for host specific tag query
void 	HostCommand(uint32_t HostCommand);
uint8_t GetHostTagXY(void);																				//api to get TAG from FT_GC for coordinates set by  SetHostTagXY() api - host needs to wait for at least 1 frame to get these query values
uint8_t Read(unsigned long Addr);
//void GetTagXY(sTagXY &sTagxy);//get the touched object tag and repective xy coordinates
//void GetTrackTag(sTrackTag &sTracktag);//get the track value and the tag value

//Special apis for ease of usage in FT_GC
void 		DLStart(void);																				//inserts cmd_dlstart() followed by clear(1,1,1) graphics command
void 		DLEnd(void);																				//inserts display() gpu instruction at the end and inserts cmd_swap() command
FT_GEStatus CheckLogo(void);																			//special api to check logo completion

//apis to render all the commands to hardware
FT_GEStatus Flush(void);																				//api to flush out all the commands to FT_GC, does not wait for the completion of the rendering
FT_GEStatus Finish(void);																				//flushes out all the commands to FT_GC and waits for the completion of execution
FT_GEStatus CheckFinish(void);																			//checks fifo and returns the status
uint32_t 	GetError(void);																				//error from graphics controller library

//API related to coprocessor fifo buffer management
//please note that all the below apis are transfer commands
FT_GEStatus WriteCmd(unsigned long Cmd);
FT_GEStatus WriteCmd_withSize(uint8_t *Src,uint32_t NBytes);											//api to send N bytes to command
FT_GEStatus WriteCmdfromflash(prog_uchar *Src,uint32_t NBytes);
FT_GEStatus TransferCmd(unsigned long Cmd);
FT_GEStatus TransferCmd_withSize(uint8_t *Src,uint32_t NBytes);
FT_GEStatus TransferCmdfromflash(prog_uchar *Src,uint32_t NBytes);
void 		EndTransferCmd();																			//de assert CSpin
//FT_GEStatus Cmd_GetResult(uint32_t &Result);															//reads the result of the previous commands such as cmd_memcrc,
																										//cmd_calibration, cmd_regread which has return values. if busy returns busy status
FT_Status 	Init(uint8_t ResType, uint8_t rotation);
FT_GEStatus Cmd_GetResults(int8_t *pA,int32_t NBytes);													//reads N bytes of result bytes from current write pointer
FT_GEStatus UpdateFreeSpace();
FT_GEStatus ChkGetFreeSpace(uint16_t NBytes);
FT_GEStatus StartTransferCmd();
void Write(unsigned long Addr, uint8_t Value8);
void Write_withSize(uint32_t Addr, uint8_t *Src, uint32_t NBytes);
void StartWrite(unsigned long Addr);
void Read_withSize(uint32_t Addr, uint8_t *Src, uint32_t NBytes);
void StartRead(unsigned long Addr);
void SetDisplayEnablePin(uint8_t GpioBit);
void SetAudioEnablePin(uint8_t GpioBit);
void Transfer(uint8_t Value8);
void Transfer32(unsigned long Value32);
unsigned int min(unsigned int a, unsigned int b);
void Writefromflash(unsigned long Addr, const unsigned char *Src, uint32_t NBytes);
void ft800_spi_fifo_init(void);
void ft800_spi_init(void);
void setup_spi_gpio(void);

#define MEM_WRITE 0x80 																					// FT800 Host Memory Write
#define MEM_READ  0x00 																					// FT800 Host Memory Write

#define		FT_DISPLAY_QVGA_320x240		0UL
#define		FT_DISPLAY_WQVGA_480x272 	1UL
#define 	FT_DISPLAY_RIVERDI_320x240	2UL
#define		FT_DISPLAY_KYOCERA			3UL

#define FT_DISPLAY_VSYNC0_WQVGA 		(0L)
#define FT_DISPLAY_VSYNC1_WQVGA 		(10L)
#define FT_DISPLAY_VOFFSET_WQVGA		(12L)
#define FT_DISPLAY_VCYCLE_WQVGA 		(292L)
#define FT_DISPLAY_HSYNC0_WQVGA 		(0L)
#define FT_DISPLAY_HSYNC1_WQVGA 		(41L)
#define FT_DISPLAY_HOFFSET_WQVGA 		(43L)
#define FT_DISPLAY_HCYCLE_WQVGA 		(548L)
#define FT_DISPLAY_HSIZE_WQVGA 			(480L)
#define FT_DISPLAY_VSIZE_WQVGA 			(272L)
#define FT_DISPLAY_PCLKPOL_WQVGA 		(1L)
#define FT_DISPLAY_SWIZZLE_WQVGA 		(0L)
#define FT_DISPLAY_PCLK_WQVGA 			(5L)

#define FT_DISPLAY_VSYNC0_QVGA 			(0L)
#define FT_DISPLAY_VSYNC1_QVGA 			(2L)
#define FT_DISPLAY_VOFFSET_QVGA 		(13L)
#define FT_DISPLAY_VCYCLE_QVGA 			(263L)
#define FT_DISPLAY_HSYNC0_QVGA 			(0L)
#define FT_DISPLAY_HSYNC1_QVGA 			(10L)
#define FT_DISPLAY_HOFFSET_QVGA 		(70L)
#define FT_DISPLAY_HCYCLE_QVGA 			(408L)
#define FT_DISPLAY_HSIZE_QVGA 			(320L)
#define FT_DISPLAY_VSIZE_QVGA 			(240L)
#define FT_DISPLAY_PCLKPOL_QVGA 		(0L)
#define FT_DISPLAY_SWIZZLE_QVGA 		(2L)
#define FT_DISPLAY_PCLK_QVGA 			(8L)

#define FT_DISPLAY_RIVERDI_VSYNC0 		(0L)
#define FT_DISPLAY_RIVERDI_VSYNC1 		(2L)
#define FT_DISPLAY_RIVERDI_VOFFSET		(13L)
#define FT_DISPLAY_RIVERDI_VCYCLE 		(263L)
#define FT_DISPLAY_RIVERDI_HSYNC0 		(0L)
#define FT_DISPLAY_RIVERDI_HSYNC1 		(10L)
#define FT_DISPLAY_RIVERDI_HOFFSET 		(70L)
#define FT_DISPLAY_RIVERDI_HCYCLE 		(408L)
#define FT_DISPLAY_RIVERDI_HSIZE		(320L)
#define FT_DISPLAY_RIVERDI_VSIZE		(240L)
#define FT_DISPLAY_RIVERDI_PCLKPOL 		(1L)
#define FT_DISPLAY_RIVERDI_SWIZZLE 		(2L)
#define FT_DISPLAY_RIVERDI_PCLK			(6L)

#define FT_DISPLAY_KYOCERA_VSYNC0 		(0L)
#define FT_DISPLAY_KYOCERA_VSYNC1 		(2L)
#define FT_DISPLAY_KYOCERA_VOFFSET		(18L)
#define FT_DISPLAY_KYOCERA_VCYCLE 		(262L)
#define FT_DISPLAY_KYOCERA_HSYNC0 		(0L)
#define FT_DISPLAY_KYOCERA_HSYNC1 		(2L)
#define FT_DISPLAY_KYOCERA_HOFFSET 		(68L)
#define FT_DISPLAY_KYOCERA_HCYCLE 		(408L)
#define FT_DISPLAY_KYOCERA_HSIZE		(320L)
#define FT_DISPLAY_KYOCERA_VSIZE		(240L)
#define FT_DISPLAY_KYOCERA_PCLKPOL 		(0L)
#define FT_DISPLAY_KYOCERA_SWIZZLE 		(0L)
#define FT_DISPLAY_KYOCERA_PCLK			(8L)

#define FT_DISPLAY_VSYNC0 				FT_DISPLAY_VSYNC0_WQVGA
#define FT_DISPLAY_VSYNC1 				FT_DISPLAY_VSYNC1_WQVGA
#define FT_DISPLAY_VOFFSET				FT_DISPLAY_VOFFSET_WQVGA
#define FT_DISPLAY_VCYCLE 				FT_DISPLAY_VCYCLE_WQVGA
#define FT_DISPLAY_HSYNC0 				FT_DISPLAY_HSYNC0_WQVGA
#define FT_DISPLAY_HSYNC1 				FT_DISPLAY_HSYNC1_WQVGA
#define FT_DISPLAY_HOFFSET 				FT_DISPLAY_HOFFSET_WQVGA
#define FT_DISPLAY_HCYCLE 				FT_DISPLAY_HCYCLE_WQVGA
#define FT_DISPLAY_HSIZE				FT_DISPLAY_HSIZE_WQVGA
#define FT_DISPLAY_VSIZE				FT_DISPLAY_VSIZE_WQVGA
#define FT_DISPLAY_PCLKPOL 				FT_DISPLAY_PCLKPOL_WQVGA
#define FT_DISPLAY_SWIZZLE 				FT_DISPLAY_SWIZZLE_WQVGA
#define FT_DISPLAY_PCLK					FT_DISPLAY_PCLK_WQVGA

//Chip identifier macros
#define FT800_CHIPID			0x00010008UL

//Macros for general purpose
#define FT_DISABLE				0
#define FT_ENABLE				1
#define FT_FALSE				0
#define FT_TRUE					1

//Macros used for graphics commands
#define FT_NEVER                0
#define FT_LESS                 1
#define FT_LEQUAL               2
#define FT_GREATER              3
#define FT_GEQUAL               4
#define FT_EQUAL                5
#define FT_NOTEQUAL             6
#define FT_ALWAYS               7

//Bitmap format macros
#define FT_ARGB1555             0
#define FT_L1                   1
#define FT_L4                   2
#define FT_L8                   3
#define FT_RGB332               4
#define FT_ARGB2                5
#define FT_ARGB4                6
#define FT_RGB565               7
#define FT_PALETTED             8
#define FT_TEXT8X8              9
#define FT_TEXTVGA              10
#define FT_BARGRAPH             11

//Bitmap filter type macros
#define FT_NEAREST              0
#define FT_BILINEAR             1

//Bitmap wrap type macros
#define FT_BORDER               0
#define FT_REPEAT               1

//Stencil macros
#define FT_KEEP                 1
#define FT_REPLACE              2
#define FT_INCR                 3
#define FT_DECR                 4
#define FT_INVERT               5

//Graphics display list swap macros
#define FT_DLSWAP_DONE          0
#define FT_DLSWAP_LINE          1
#define FT_DLSWAP_FRAME         2

//Interrupt bits
#define FT_INT_SWAP             0x01
#define FT_INT_TOUCH            0x02
#define FT_INT_TAG              0x04
#define FT_INT_SOUND            0x08
#define FT_INT_PLAYBACK         0x10
#define FT_INT_CMDEMPTY         0x20
#define FT_INT_CMDFLAG          0x40
#define FT_INT_CONVCOMPLETE     0x80

//Touch mode macros
#define FT_TMODE_OFF        	0
#define FT_TMODE_ONESHOT    	1
#define FT_TMODE_FRAME      	2
#define FT_TMODE_CONTINUOUS 	3

//Alpha blending macros
#define FT_ZERO                 0
#define FT_ONE                  1
#define FT_SRC_ALPHA            2
#define FT_DST_ALPHA            3
#define FT_ONE_MINUS_SRC_ALPHA  4
#define FT_ONE_MINUS_DST_ALPHA  5

//Graphics primitives macros
#define FT_BITMAPS              1
#define FT_POINTS               2
#define FT_LINES                3
#define FT_LINE_STRIP           4
#define FT_EDGE_STRIP_R         5
#define FT_EDGE_STRIP_L         6
#define FT_EDGE_STRIP_A         7
#define FT_EDGE_STRIP_B         8
#define FT_RECTS                9

//Widget command macros
#define FT_OPT_MONO             1
#define FT_OPT_NODL             2
#define FT_OPT_FLAT             256
#define FT_OPT_CENTERX          512
#define FT_OPT_CENTERY          1024
#define FT_OPT_CENTER           (FT_OPT_CENTERX | FT_OPT_CENTERY)
#define FT_OPT_NOBACK           4096
#define FT_OPT_NOTICKS          8192
#define FT_OPT_NOHM             16384
#define FT_OPT_NOPOINTER        16384
#define FT_OPT_NOSECS           32768
#define FT_OPT_NOHANDS          49152
#define FT_OPT_RIGHTX           2048
#define FT_OPT_SIGNED           256

//Macros related to inbuilt font
#define FT_NUMCHAR_PERFONT 		(128L)		//number of font characters per bitmap handle
#define FT_FONT_TABLE_SIZE 		(148L)		//size of the font table - utilized for loopup by the graphics engine
#define FT_FONT_TABLE_POINTER	(0xFFFFCUL)	//pointer to the inbuilt font tables starting from bitmap handle 16

//Audio sample type macros
#define FT_LINEAR_SAMPLES       0	//8bit signed samples
#define FT_ULAW_SAMPLES         1	//8bit ulaw samples
#define FT_ADPCM_SAMPLES        2	//4bit ima adpcm samples

//Synthesized sound macros
#define FT_SILENCE              0x00
#define FT_SQUAREWAVE           0x01
#define FT_SINEWAVE             0x02
#define FT_SAWTOOTH             0x03
#define FT_TRIANGLE             0x04
#define FT_BEEPING              0x05
#define FT_ALARM                0x06
#define FT_WARBLE               0x07
#define FT_CAROUSEL             0x08
#define FT_PIPS(n)              (0x0F + (n))
#define FT_HARP                 0x40
#define FT_XYLOPHONE            0x41
#define FT_TUBA                 0x42
#define FT_GLOCKENSPIEL         0x43
#define FT_ORGAN                0x44
#define FT_TRUMPET              0x45
#define FT_PIANO                0x46
#define FT_CHIMES               0x47
#define FT_MUSICBOX             0x48
#define FT_BELL                 0x49
#define FT_CLICK                0x50
#define FT_SWITCH               0x51
#define FT_COWBELL              0x52
#define FT_NOTCH                0x53
#define FT_HIHAT                0x54
#define FT_KICKDRUM             0x55
#define FT_POP                  0x56
#define FT_CLACK                0x57
#define FT_CHACK                0x58
#define FT_MUTE                 0x60
#define FT_UNMUTE               0x61

//Synthesized sound frequencies, midi note macros
#define FT_MIDI_A0            	21
#define FT_MIDI_A_0           	22
#define FT_MIDI_B0            	23
#define FT_MIDI_C1            	24
#define FT_MIDI_C_1           	25
#define FT_MIDI_D1            	26
#define FT_MIDI_D_1           	27
#define FT_MIDI_E1            	28
#define FT_MIDI_F1            	29
#define FT_MIDI_F_1           	30
#define FT_MIDI_G1            	31
#define FT_MIDI_G_1           	32
#define FT_MIDI_A1            	33
#define FT_MIDI_A_1           	34
#define FT_MIDI_B1            	35
#define FT_MIDI_C2            	36
#define FT_MIDI_C_2           	37
#define FT_MIDI_D2            	38
#define FT_MIDI_D_2           	39
#define FT_MIDI_E2            	40
#define FT_MIDI_F2            	41
#define FT_MIDI_F_2           	42
#define FT_MIDI_G2            	43
#define FT_MIDI_G_2           	44
#define FT_MIDI_A2            	45
#define FT_MIDI_A_2           	46
#define FT_MIDI_B2            	47
#define FT_MIDI_C3            	48
#define FT_MIDI_C_3           	49
#define FT_MIDI_D3            	50
#define FT_MIDI_D_3           	51
#define FT_MIDI_E3            	52
#define FT_MIDI_F3            	53
#define FT_MIDI_F_3           	54
#define FT_MIDI_G3            	55
#define FT_MIDI_G_3           	56
#define FT_MIDI_A3            	57
#define FT_MIDI_A_3           	58
#define FT_MIDI_B3            	59
#define FT_MIDI_C4            	60
#define FT_MIDI_C_4           	61
#define FT_MIDI_D4            	62
#define FT_MIDI_D_4           	63
#define FT_MIDI_E4            	64
#define FT_MIDI_F4            	65
#define FT_MIDI_F_4           	66
#define FT_MIDI_G4            	67
#define FT_MIDI_G_4           	68
#define FT_MIDI_A4            	69
#define FT_MIDI_A_4           	70
#define FT_MIDI_B4            	71
#define FT_MIDI_C5            	72
#define FT_MIDI_C_5           	73
#define FT_MIDI_D5            	74
#define FT_MIDI_D_5           	75
#define FT_MIDI_E5            	76
#define FT_MIDI_F5            	77
#define FT_MIDI_F_5           	78
#define FT_MIDI_G5            	79
#define FT_MIDI_G_5           	80
#define FT_MIDI_A5            	81
#define FT_MIDI_A_5           	82
#define FT_MIDI_B5            	83
#define FT_MIDI_C6            	84
#define FT_MIDI_C_6           	85
#define FT_MIDI_D6            	86
#define FT_MIDI_D_6           	87
#define FT_MIDI_E6            	88
#define FT_MIDI_F6            	89
#define FT_MIDI_F_6           	90
#define FT_MIDI_G6            	91
#define FT_MIDI_G_6           	92
#define FT_MIDI_A6            	93
#define FT_MIDI_A_6           	94
#define FT_MIDI_B6            	95
#define FT_MIDI_C7            	96
#define FT_MIDI_C_7           	97
#define FT_MIDI_D7            	98
#define FT_MIDI_D_7           	99
#define FT_MIDI_E7            	100
#define FT_MIDI_F7            	101
#define FT_MIDI_F_7           	102
#define FT_MIDI_G7            	103
#define FT_MIDI_G_7           	104
#define FT_MIDI_A7            	105
#define FT_MIDI_A_7           	106
#define FT_MIDI_B7            	107
#define FT_MIDI_C8            	108

//GPIO bit macros
#define FT_GPIO0				0
#define FT_GPIO1				1																		//default gpio pin for audio shutdown, 1 - eanble, 0 - disable
#define FT_GPIO7				7																		//default gpio pin for display enable, 1 - enable, 0 - disable

/* Display rotation */
#define FT_DISPLAY_0			0																		//0 degrees rotation
#define FT_DISPLAY_180			1																		//180 degrees rotation

//Maximum display display resolution supported by graphics engine
#define FT_MAX_DISPLAYWIDTH		(512L)
#define FT_MAX_DISPLAYHEIGHT	(512L)

//Host command macros
#define FT_ACTIVE				0x00																	// Place FT800 in active state
#define FT_STANDBY				0x41																	// Place FT800 in Standby (clk running)
#define FT_SLEEP				0x42																	// Place FT800 in Sleep (clk off)
#define FT_PWRDOWN				0x50																	// Place FT800 in Power Down (core off)
#define FT_CLKEXT				0x44																	// Select external clock source
#define FT_CLKINT				0x48																	// Select internal clock source
#define FT_CLK48M				0x62																	// Select 48MHz PLL output
#define FT_CLK36M				0x61																	// Select 36MHz PLL output
#define FT_CORERST				0x68																	// Reset core - all registers default and processors reset

///Coprocessor reset related macros
#define FT_RESET_HOLD_COPROCESSOR		1
#define FT_RESET_RELEASE_COPROCESSOR	0

//Macros for sound play and stop
#define FT_SOUND_PLAY					1
#define FT_AUDIO_PLAY					1

//Macros for audio playback parameters
#define FT_AUDIO_SAMPLINGFREQ_MIN		8*1000L
#define FT_AUDIO_SAMPLINGFREQ_MAX		48*1000L

//coprocessor error macros
#define FT_COPRO_ERROR					0xfffUL

//Memory definitions
#define FT_RAM_G						0x000000UL
#define FT_ROM_CHIPID					0x0C0000UL
#define FT_ROM_FONT						0x0BB23CUL
#define FT_ROM_FONT_ADDR				0x0FFFFCUL
#define FT_RAM_DL						0x100000UL
#define FT_RAM_PAL						0x102000UL
#define FT_RAM_CMD						0x108000UL
#define FT_RAM_SCREENSHOT				0x1C2000UL

//Memory buffer sizes
#define FT_RAM_G_SIZE					256*1024L
#define FT_CMDFIFO_SIZE					4*1024L
#define FT_RAM_DL_SIZE					8*1024L
#define FT_RAM_PAL_SIZE					1*1024L

//Coprocessor related commands
#define CMD_APPEND           			0xFFFFFF1EUL
#define CMD_BGCOLOR          			0xFFFFFF09UL
#define CMD_BITMAP_TRANSFORM 			0xFFFFFF21UL
#define CMD_BUTTON           			0xFFFFFF0DUL
#define CMD_CALIBRATE        			0xFFFFFF15UL
#define CMD_CLOCK            			0xFFFFFF14UL
#define CMD_COLDSTART        			0xFFFFFF32UL
#define CMD_CRC              			0xFFFFFF03UL
#define CMD_DIAL             			0xFFFFFF2DUL
#define CMD_DLSTART          			0xFFFFFF00UL
#define CMD_EXECUTE          			0xFFFFFF07UL
#define CMD_FGCOLOR          			0xFFFFFF0AUL
#define CMD_GAUGE            			0xFFFFFF13UL
#define CMD_GETMATRIX        			0xFFFFFF33UL
#define CMD_GETPOINT         			0xFFFFFF08UL
#define CMD_GETPROPS         			0xFFFFFF25UL
#define CMD_GETPTR           			0xFFFFFF23UL
#define CMD_GRADCOLOR        			0xFFFFFF34UL
#define CMD_GRADIENT         			0xFFFFFF0BUL
#define CMD_HAMMERAUX        			0xFFFFFF04UL
#define CMD_IDCT             			0xFFFFFF06UL
#define CMD_INFLATE          			0xFFFFFF22UL
#define CMD_INTERRUPT        			0xFFFFFF02UL
#define CMD_KEYS             			0xFFFFFF0EUL
#define CMD_LOADIDENTITY     			0xFFFFFF26UL
#define CMD_LOADIMAGE        			0xFFFFFF24UL
#define CMD_LOGO             			0xFFFFFF31UL
#define CMD_MARCH            			0xFFFFFF05UL
#define CMD_MEMCPY           			0xFFFFFF1DUL
#define CMD_MEMCRC           			0xFFFFFF18UL
#define CMD_MEMSET           			0xFFFFFF1BUL
#define CMD_MEMWRITE         			0xFFFFFF1AUL
#define CMD_MEMZERO          			0xFFFFFF1CUL
#define CMD_NUMBER           			0xFFFFFF2EUL
#define CMD_PROGRESS         			0xFFFFFF0FUL
#define CMD_REGREAD          			0xFFFFFF19UL
#define CMD_ROTATE           			0xFFFFFF29UL
#define CMD_SCALE            			0xFFFFFF28UL
#define CMD_SCREENSAVER      			0xFFFFFF2FUL
#define CMD_SCROLLBAR        			0xFFFFFF11UL
#define CMD_SETFONT          			0xFFFFFF2BUL
#define CMD_SETMATRIX        			0xFFFFFF2AUL
#define CMD_SKETCH           			0xFFFFFF30UL
#define CMD_SLIDER           			0xFFFFFF10UL
#define CMD_SNAPSHOT         			0xFFFFFF1FUL
#define CMD_SPINNER          			0xFFFFFF16UL
#define CMD_STOP             			0xFFFFFF17UL
#define CMD_SWAP             			0xFFFFFF01UL
#define CMD_TEXT             			0xFFFFFF0CUL
#define CMD_TOGGLE           			0xFFFFFF12UL
#define CMD_TOUCH_TRANSFORM  			0xFFFFFF20UL
#define CMD_TRACK            			0xFFFFFF2CUL
#define CMD_TRANSLATE        			0xFFFFFF27UL

//Register definitions
#define REG_ID							0x102400UL
#define REG_FRAMES						0x102404UL
#define REG_CLOCK						0x102408UL
#define REG_FREQUENCY					0x10240CUL
#define REG_SCREENSHOT_EN				0x102410UL
#define REG_SCREENSHOT_Y				0x102414UL
#define REG_SCREENSHOT_START 			0x102418UL
#define REG_CPURESET 					0x10241CUL
#define REG_TAP_CRC 					0x102420UL
#define REG_TAP_MASK 					0x102424UL
#define REG_HCYCLE 						0x102428UL
#define REG_HOFFSET 					0x10242CUL
#define REG_HSIZE 						0x102430UL
#define REG_HSYNC0 						0x102434UL
#define REG_HSYNC1 						0x102438UL
#define REG_VCYCLE 						0x10243CUL
#define REG_VOFFSET 					0x102440UL
#define REG_VSIZE 						0x102444UL
#define REG_VSYNC0 						0x102448UL
#define REG_VSYNC1 						0x10244CUL
#define REG_DLSWAP 						0x102450UL
#define REG_ROTATE 						0x102454UL
#define REG_OUTBITS 					0x102458UL
#define REG_DITHER 						0x10245CUL
#define REG_SWIZZLE 					0x102460UL
#define REG_CSPREAD 					0x102464UL
#define REG_PCLK_POL 					0x102468UL
#define REG_PCLK 						0x10246CUL
#define REG_TAG_X 						0x102470UL
#define REG_TAG_Y 						0x102474UL
#define REG_TAG 						0x102478UL
#define REG_VOL_PB 						0x10247CUL
#define REG_VOL_SOUND 					0x102480UL
#define REG_SOUND 						0x102484UL
#define REG_PLAY 						0x102488UL
#define REG_GPIO_DIR 					0x10248CUL
#define REG_GPIO 						0x102490UL
#define REG_INT_FLAGS           		0x102498UL
#define REG_INT_EN              		0x10249CUL
#define REG_INT_MASK            		0x1024A0UL
#define REG_PLAYBACK_START      		0x1024A4UL
#define REG_PLAYBACK_LENGTH     		0x1024A8UL
#define REG_PLAYBACK_READPTR    		0x1024ACUL
#define REG_PLAYBACK_FREQ       		0x1024B0UL
#define REG_PLAYBACK_FORMAT     		0x1024B4UL
#define REG_PLAYBACK_LOOP       		0x1024B8UL
#define REG_PLAYBACK_PLAY       		0x1024BCUL
#define REG_PWM_HZ              		0x1024C0UL
#define REG_PWM_DUTY            		0x1024C4UL
#define REG_MACRO_0             		0x1024C8UL
#define REG_MACRO_1             		0x1024CCUL
#define REG_SCREENSHOT_BUSY				0x1024D8UL
#define REG_CMD_READ            		0x1024E4UL
#define REG_CMD_WRITE           		0x1024E8UL
#define REG_CMD_DL              		0x1024ECUL
#define REG_TOUCH_MODE          		0x1024F0UL
#define REG_TOUCH_ADC_MODE      		0x1024F4UL
#define REG_TOUCH_CHARGE        		0x1024F8UL
#define REG_TOUCH_SETTLE        		0x1024FCUL
#define REG_TOUCH_OVERSAMPLE    		0x102500UL
#define REG_TOUCH_RZTHRESH      		0x102504UL
#define REG_TOUCH_RAW_XY        		0x102508UL
#define REG_TOUCH_RZ            		0x10250CUL
#define REG_TOUCH_SCREEN_XY     		0x102510UL
#define REG_TOUCH_TAG_XY        		0x102514UL
#define REG_TOUCH_TAG           		0x102518UL
#define REG_TOUCH_TRANSFORM_A   		0x10251CUL
#define REG_TOUCH_TRANSFORM_B   		0x102520UL
#define REG_TOUCH_TRANSFORM_C   		0x102524UL
#define REG_TOUCH_TRANSFORM_D   		0x102528UL
#define REG_TOUCH_TRANSFORM_E   		0x10252CUL
#define REG_TOUCH_TRANSFORM_F   		0x102530UL
#define REG_SCREENSHOT_READ				0x102554UL
#define REG_TRIM						0x10256CUL
#define REG_TOUCH_DIRECT_XY 			0x102574UL
#define REG_TOUCH_DIRECT_Z1Z2 			0x102578UL
#define REG_TRACKER						0x109000UL

// Display list commands to be embedded in Graphics Processor
#define DL_ALPHA_FUNC       0x09000000UL 																// requires OR'd arguments
#define DL_BITMAP_HANDLE    0x05000000UL 																// requires OR'd arguments
#define DL_BITMAP_LAYOUT    0x07000000UL 																// requires OR'd arguments
#define DL_BITMAP_SIZE      0x08000000UL 																// requires OR'd arguments
#define DL_BITMAP_SOURCE    0x01000000UL 																// requires OR'd arguments
#define DL_BITMAP_TFORM_A   0x15000000UL 																// requires OR'd arguments
#define DL_BITMAP_TFORM_B   0x16000000UL 																// requires OR'd arguments
#define DL_BITMAP_TFORM_C   0x17000000UL 																// requires OR'd arguments
#define DL_BITMAP_TFORM_D   0x18000000UL 																// requires OR'd arguments
#define DL_BITMAP_TFORM_E   0x19000000UL 																// requires OR'd arguments
#define DL_BITMAP_TFORM_F   0x1A000000UL	 															// requires OR'd arguments
#define DL_BLEND_FUNC       0x0B000000UL 																// requires OR'd arguments
#define DL_BEGIN            0x1F000000UL 																// requires OR'd arguments
#define DL_CALL             0x1D000000UL 																// requires OR'd arguments
#define DL_CLEAR            0x26000000UL 																// requires OR'd arguments
#define DL_CELL             0x06000000UL 																// requires OR'd arguments
#define DL_CLEAR_RGB        0x02000000UL 																// requires OR'd arguments
#define DL_CLEAR_STENCIL    0x11000000UL 																// requires OR'd arguments
#define DL_CLEAR_TAG        0x12000000UL 																// requires OR'd arguments
#define DL_COLOR_A          0x0F000000UL 																// requires OR'd arguments
#define DL_COLOR_MASK       0x20000000UL 																// requires OR'd arguments
#define DL_COLOR_RGB        0x04000000UL	 															// requires OR'd arguments
#define DL_DISPLAY          0x00000000UL
#define DL_END              0x21000000UL
#define DL_JUMP             0x1E000000UL 																// requires OR'd arguments
#define DL_LINE_WIDTH       0x0E000000UL 																// requires OR'd arguments
#define DL_MACRO            0x25000000UL 																// requires OR'd arguments
#define DL_POINT_SIZE       0x0D000000UL 																// requires OR'd arguments
#define DL_RESTORE_CONTEXT  0x23000000UL
#define DL_RETURN           0x24000000UL
#define DL_SAVE_CONTEXT     0x22000000UL
#define DL_SCISSOR_SIZE     0x1C000000UL 																// requires OR'd arguments
#define DL_SCISSOR_XY       0x1B000000UL 																// requires OR'd arguments
#define DL_STENCIL_FUNC     0x0A000000UL 																// requires OR'd arguments
#define DL_STENCIL_MASK     0x13000000UL 																// requires OR'd arguments
#define DL_STENCIL_OP       0x0C000000UL 																// requires OR'd arguments
#define DL_TAG              0x03000000UL 																// requires OR'd arguments
#define DL_TAG_MASK         0x14000000UL 																// requires OR'd arguments
#define DL_VERTEX2F         0x40000000UL 																// requires OR'd arguments
#define DL_VERTEX2II        0x02000000UL 																// requires OR'd arguments

//Command and register value options
#define CLR_COL              0x4
#define CLR_STN              0x2
#define CLR_TAG              0x1
#define DECR                 4UL
#define DECR_WRAP            7UL
#define DLSWAP_DONE          0UL
#define DLSWAP_FRAME         2UL
#define DLSWAP_LINE          1UL
#define DST_ALPHA            3UL
#define EDGE_STRIP_A         7UL
#define EDGE_STRIP_B         8UL
#define EDGE_STRIP_L         6UL
#define EDGE_STRIP_R         5UL
#define EQUAL                5UL
#define GEQUAL               4UL
#define GREATER              3UL
#define INCR                 3UL
#define INCR_WRAP            6UL
#define INT_CMDEMPTY         32UL
#define INT_CMDFLAG          64UL
#define INT_CONVCOMPLETE     128UL
#define INT_PLAYBACK         16UL
#define INT_SOUND            8UL
#define INT_SWAP             1UL
#define INT_TAG              4UL
#define INT_TOUCH            2UL
#define INVERT               5UL
#define KEEP                 1UL
#define L1                   1UL
#define L4                   2UL
#define L8                   3UL
#define LEQUAL               2UL
#define LESS                 1UL
#define LINEAR_SAMPLES       0UL
#define LINES                3UL
#define LINE_STRIP           4UL
#define NEAREST              0UL
#define NEVER                0UL
#define NOTEQUAL             6UL
#define ONE                  1UL
#define ONE_MINUS_DST_ALPHA  5UL
#define ONE_MINUS_SRC_ALPHA  4UL
#define OPT_CENTER           1536UL
#define OPT_CENTERX          512UL
#define OPT_CENTERY          1024UL
#define OPT_FLAT             256UL
#define OPT_MONO             1UL
#define OPT_NOBACK           4096UL
#define OPT_NODL             2UL
#define OPT_NOHANDS          49152UL
#define OPT_NOHM             16384UL
#define OPT_NOPOINTER        16384UL
#define OPT_NOSECS           32768UL
#define OPT_NOTICKS          8192UL
#define OPT_RIGHTX           2048UL
#define OPT_SIGNED           256UL
#define PALETTED             8UL
#define PLAYCOLOR            0x00a0a080
#define FTPOINTS             2UL
#define RECTS                9UL
#define REPEAT               1UL
#define REPLACE              2UL
#define RGB332               4UL
#define RGB565               7UL
#define SRC_ALPHA            2UL
#define TEXT8X8              9UL
#define TEXTVGA              10UL
#define TOUCHMODE_CONTINUOUS 3UL
#define TOUCHMODE_FRAME      2UL
#define TOUCHMODE_OFF        0UL
#define TOUCHMODE_ONESHOT    1UL
#define ULAW_SAMPLES         1UL
#define ZERO                 0UL

#define pgm_read_byte_near(x)   (*(x))


//Define Colors with alpha
#define RED			0xFFFF0000UL		// Red
#define GREEN		0xFF00FF00UL		// Green
#define BLUE		0xFF0000FFUL		// Blue
#define WHITE		0xFFFFFFFFUL		// White
#define BLACK		0xFF000000UL		// Black
#define YELLOW		0xFFFFD200UL		// Yellow


#endif /* FT800_H_ */
