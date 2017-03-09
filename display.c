/*
 * display.c
 *
 *  Created on: Mar 23, 2016
 *      Author: srock
 */
#include "project.h"
#include <string.h>
#include <stdlib.h>
#include "custom_font.h"
#include "omniPump.h"

extern unsigned int hsize, vsize;
extern unsigned int volts_val, current_val, i_mot_val, leakage_val, phase_error_val, pot_val, sys_fail, agc_error_val, agc_pwm_val, pumpRPM, dispTimer, handpieceID;
extern float frequency;
extern states state;
//##################################################################################################################################################################
//						loadImages - load initial images into FT800 from DSP memory
//##################################################################################################################################################################
void loadImages()
{
	Cmd_Inflate(RAM_OMNIPUMP);
	WriteCmdfromflash(omniPump, sizeof(omniPump));
	Finish();

	DLStart();

	BitmapHandle(OMNIPUMP_HANDLE);
	BitmapSource(RAM_OMNIPUMP);
	BitmapLayout(FT_ARGB2,OMNIPUMP_LINESTRIDE,OMNIPUMP_HEIGHT);
	BitmapSize(FT_NEAREST,FT_BORDER,FT_BORDER,OMNIPUMP_WIDTH,OMNIPUMP_HEIGHT);

	DLEnd();
	Finish();
}


//##################################################################################################################################################################
//						loadFont - load custom font into FT800 from DSP memory
//##################################################################################################################################################################
void loadFont()
{
	Writefromflash(FT_RAM_G + RAM_FONT,	MetricBlock,sizeof(MetricBlock));
	Finish();
	Writefromflash(FT_RAM_G + RAM_FONT + sizeof(MetricBlock),FontBmpData,sizeof(FontBmpData));
	Finish();


	DLStart();
	Clear(1,1,1);
	BitmapHandle(7);
	BitmapSource(SOURCE_FONT); 																				//Source address from H file, note have to put option -d XXXX large enough so this is a positive number

	BitmapLayout(L1,6,89);
	BitmapSize(FT_NEAREST,FT_BORDER,FT_BORDER,48,89);

	Cmd_SetFont(7, FT_RAM_G + RAM_FONT);
	DLEnd();
	Finish();
}


//##################################################################################################################################################################
//						Display Time (FT800)
//##################################################################################################################################################################
void time(unsigned int timeLeft, unsigned char systemOn)
{
	char *custom_font[11] = {"\x01", "\x02", "\x03", "\x04", "\x05", "\x06", "\x07", "\x08", "\x09", "\x0A","\x0B"};	//Using a custom font requires this lookup array
	char runningTime[5]={0,0,0,0,0};
	unsigned int min, sec, sec1, sec2, min1, min2;
	min = timeLeft/60;
	min1 = min/10;
	min2 = min%10;
	sec = timeLeft%60;
	sec1 = sec/10;
	sec2 = sec % 10;

	//use lookup table display text, b/c utf8 encoded you can't just put the string
	strcat(runningTime,custom_font[min1]);																// Construct runningTime into the form MM:SS
	strcat(runningTime,custom_font[min2]);
	strcat(runningTime,custom_font[10]);
	strcat(runningTime,custom_font[sec1]);
	strcat(runningTime,custom_font[sec2]);
	DLStart();
	if(SHOW_ENG)																						// If GPIO1 is shorted show ENG screens
		dispEngScreens();
	Cmd_Text(0,60,20,0,"W1S02988");
	if(systemOn&&state!=DELAY)																						// If the system is outputting ultrasound display the following
	{
		if(timeLeft<120)																				// If less than 120 seconds display the following
		{
			ColorARGB(YELLOW);
			Cmd_Text((hsize/2), vsize/2-20, 7, OPT_CENTER, runningTime);
			ColorARGB(WHITE);
			Cmd_Text((hsize/2), 186, 24, OPT_CENTER, "Recommended Use Time Approaching");
		}
		else																							// If more than 120 seconds display the following
		{
			ColorARGB(WHITE);
			Cmd_Text((hsize/2), vsize/2-20, 7, OPT_CENTER, runningTime);
			ColorARGB(WHITE);
			Cmd_Text((hsize/2), 186, 30, OPT_CENTER, "Energy On");
		}
	}
	else if(state==DELAY)
	{
		ColorARGB(WHITE);
		Cmd_Text((hsize/2), vsize/2-20, 7, OPT_CENTER, runningTime);
		ColorARGB(WHITE);
		Cmd_Text((hsize/2), 186, 30, OPT_CENTER, "Remaining Delay");
	}
	else																								// If the system is off display the following
	{
		if(timeLeft<120)
		{
			ColorARGB(YELLOW);
			Cmd_Text((hsize/2), vsize/2-20, 7, OPT_CENTER, runningTime);
			ColorARGB(WHITE);
			Cmd_Text((hsize/2), 186, 30, OPT_CENTER, "System Ready");
		}
		else
		{
			ColorARGB(WHITE);
			Cmd_Text((hsize/2), vsize/2-20, 7, OPT_CENTER, runningTime);
			ColorARGB(WHITE);
			Cmd_Text((hsize/2), 186, 30, OPT_CENTER, "System Ready");
		}
	}
	DLEnd();
	Finish();
}


//################################################################################################################################################################################################
//						Display the ready to prime screen
//################################################################################################################################################################################################
void displayReadyToPrime(void)
{
	DLStart();
	Clear(1,1,1);
	Begin(FT_BITMAPS);
	Vertex2ii(hsize/2-OMNIPUMP_WIDTH/2,vsize/2-OMNIPUMP_HEIGHT/2,OMNIPUMP_HANDLE,0);
	End();
	Cmd_Text(hsize/2,50,24,OPT_CENTER,"Open Tubing Clamp and Load Tubing");
	Cmd_Text(hsize/2,216,24,OPT_CENTER,"Press Prime When Ready");
	DLEnd();
	Finish();
}


//################################################################################################################################################################################################
//						Display the priming screen
//################################################################################################################################################################################################
void displayPriming(void)
{
	DLStart();
	Cmd_Text(hsize/2,vsize/2,31,OPT_CENTER,"Priming");
	DLEnd();
	Finish();
}


//################################################################################################################################################################################################
//						Display the Connect Device screen when the hand piece is not connected
//################################################################################################################################################################################################
void displayConnectDevice(void)
{
	DLStart();
	if(SHOW_ENG)
		dispEngScreens();
	Cmd_Text(hsize/2,vsize/2,31,OPT_CENTER,"Connect Device");
	DLEnd();
	Finish();
}


//################################################################################################################################################################################################
//						This function will display relevant values on the screen for debugging purposes
//						This screen is enabled when GPIO1 is shorted to GND
//################################################################################################################################################################################################
void dispEngScreens(void)
{
	static unsigned int potVal=0,phaseCmd=0,pumpSpeed=0,volts=0,current=0,sysFail=0,phaseError=0,iMot=0,agcPWM=0,leakage=0,agcError=0,handID=0;
	static float freq=0;

	Cmd_Text(0,0,20,0,"C0 Value:"); 		Cmd_Text(80,0,20,0,"Phase CMD:"); 	Cmd_Text(175,0,20,0,"Pump PRM:");
	Cmd_Text(0,12,20,0,"Volts:"); 			Cmd_Text(80,12,20,0,"Current:"); 	Cmd_Text(175,12,20,0,"Sys Fail:");
	Cmd_Text(0,24,20,0,"Phase Err:"); 		Cmd_Text(80,24,20,0,"Freq:");		Cmd_Text(175,24,20,0,"I MOT:");
	Cmd_Text(0,36,20,0,"AGC PWM:"); 		Cmd_Text(80,36,20,0,"Leakage:"); 	Cmd_Text(175,36,20,0,"AGC Error:");
	Cmd_Text(0,48,20,0,"HandPieceID:");

	Cmd_Number(55,0,20,0,potVal); 			Cmd_Number(140,0,20,0,phaseCmd);	Cmd_Number(235,0,20,0,pumpSpeed);
	Cmd_Number(55,12,20,0,volts); 			Cmd_Number(140,12,20,0,current);	Cmd_Number(235,12,20,0,sysFail);
	Cmd_Number(55,24,20,0,phaseError);		Cmd_Number(140,24,20,0,freq); 		Cmd_Number(235,24,20,0,iMot);
	Cmd_Number(55,36,20,0,agcPWM); 			Cmd_Number(140,36,20,0,leakage);	Cmd_Number(235,36,20,0,agcError);
	Cmd_Number(55,48,20,0,handID);

	if(dispTimer>5000)																					// Update values every 0.5 seconds, anything quicker than that will make it unreadable
	{
		potVal = pot_val;
		phaseCmd = EPwm3Regs.CMPA.half.CMPA;
		pumpSpeed = pumpRPM;
		volts = volts_val;
		current = current_val;
		sysFail = sys_fail;
		phaseError = phase_error_val;
		iMot = i_mot_val;
		agcPWM = agc_pwm_val;
		leakage = leakage_val;
		agcError = agc_error_val;
		freq=frequency;
		handID = handpieceID;
		dispTimer=0;
	}

}

//################################################################################################################################################################################################
//						Show recoverable error screen when sys_fail is > 0
//################################################################################################################################################################################################
void displayRecoverableError(void)
{
	DLStart();																			// Update display
	if(SHOW_ENG)
		dispEngScreens();
	Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Recoverable Error");
	Cmd_Text(hsize/2,vsize/2+40, 25, OPT_CENTER,"Press Hand Piece Button or");
	Cmd_Text(hsize/2,vsize/2+80, 25, OPT_CENTER,"Footswitch to Reset");
	DLEnd();
	Finish();
}
