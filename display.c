/*
 * display.c
 *
 *  Created on: Mar 23, 2016
 *      Author: srock
 */
#include "project.h"
#include <string.h>
#include <stdlib.h>
#include "omniPump.h"
#include "hourGlass.h"
#include "timeBubble.h"

extern unsigned int hsize, vsize;
extern unsigned int volts_val, current_val, i_mot_val, leakage_val, phase_error_val, pot_val, sys_fail, agc_error_val, agc_pwm_val, pumpRPM, dispTimer, handpieceID, runTime, AGC_HIGH;
extern unsigned char energyOn;
extern float frequency;
extern const char software;
//##################################################################################################################################################################
//						loadImages - load initial images into FT800 from DSP memory
//##################################################################################################################################################################
void loadImages(unsigned char fault)
{
	unsigned char i=0, timesToRepeat=1;
	if(fault)
		timesToRepeat=2;																				// For some reason when recovering from display fault this image needs loaded twice

	for(i=0;i<timesToRepeat;i++)
	{
		Cmd_Inflate(RAM_OMNIPUMP);
		WriteCmdfromflash(omniPump, sizeof(omniPump));
		Finish();
	}

	Cmd_Inflate(RAM_HOURGLASS);
	WriteCmdfromflash(hourGlass, sizeof(hourGlass));
	Finish();

	Cmd_Inflate(RAM_TIMEBUBBLE);
	WriteCmdfromflash(timeBubble, sizeof(timeBubble));
	Finish();

	DLStart();

	BitmapHandle(OMNIPUMP_HANDLE);
	BitmapSource(RAM_OMNIPUMP);
	BitmapLayout(FT_ARGB2,OMNIPUMP_LINESTRIDE,OMNIPUMP_HEIGHT);
	BitmapSize(FT_NEAREST,FT_BORDER,FT_BORDER,OMNIPUMP_WIDTH,OMNIPUMP_HEIGHT);

	BitmapHandle(HOURGLASS_HANDLE);
	BitmapSource(RAM_HOURGLASS);
	BitmapLayout(FT_ARGB2,HOURGLASS_LINESTRIDE,HOURGLASS_HEIGHT);
	BitmapSize(FT_NEAREST,FT_BORDER,FT_BORDER,HOURGLASS_WIDTH,HOURGLASS_HEIGHT);

	BitmapHandle(TIMEBUBBLE_HANDLE);
	BitmapSource(RAM_TIMEBUBBLE);
	BitmapLayout(FT_ARGB2,TIMEBUBBLE_LINESTRIDE,TIMEBUBBLE_HEIGHT);
	BitmapSize(FT_NEAREST,FT_BORDER,FT_BORDER,TIMEBUBBLE_WIDTH,TIMEBUBBLE_HEIGHT);

	DLEnd();
	Finish();
}

//##################################################################################################################################################################
//						Display Time (FT800)
//##################################################################################################################################################################
void runningScreen(unsigned int timeLeft, unsigned int useTime, unsigned char runningCondition, unsigned char systemOn, unsigned long barColor, unsigned char textColor)
{
	unsigned long tColor;

	if(textColor==tWHITE)
		tColor=WHITE;

	else
		tColor=BLACK;

	displayTime(timeLeft,useTime,runningCondition,systemOn, barColor, textColor);
	if(systemOn)																						// If the system is outputting ultrasound display the following
	{
		if(timeLeft<=120)																				// If less than 120 seconds display the following
		{
			ColorARGB(WARNING_COLOR);
			Cmd_Text((hsize/2), 220, 24, OPT_CENTER, "USE TIME REMAINING");
		}
		else																							// If more than 120 seconds display the following
		{
			ColorARGB(tColor);
			Cmd_Text((hsize/2), 220, 30, OPT_CENTER, "ENERGY ON");
		}
	}
	else if(runningCondition!=ERROR&&runningCondition!=UNRECERROR)										// If the system is off display the following
	{
		if(timeLeft<=120)																				// If less than 120 seconds display the following
		{
			ColorARGB(WARNING_COLOR);
			Cmd_Text((hsize/2), 220, 30, OPT_CENTER, "SYSTEM READY");
		}
		else																							// If more than 120 seconds display the following
		{
			ColorARGB(tColor);
			Cmd_Text((hsize/2), 220, 30, OPT_CENTER, "SYSTEM READY");
		}
	}
	else if(runningCondition==ERROR)
		displayRecoverableError();
	else
		displayUnrecoverableError();
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
	unsigned char textPos;

	if(sys_fail>0&&sys_fail!=19)
		textPos=50;
	else
		textPos=160;

	Cmd_Text(10,0,20,0,"C0 Value:"); 		Cmd_Text(90,0,20,0,"Phase CMD:"); 	Cmd_Text(10,textPos,20,0,"Pump PRM:");
	Cmd_Text(10,12,20,0,"Volts:"); 			Cmd_Text(90,12,20,0,"Current:"); 	Cmd_Text(10,textPos+12,20,0,"Sys Fail:");
	Cmd_Text(10,24,20,0,"Phase Err:"); 		Cmd_Text(90,24,20,0,"Freq:");		Cmd_Text(10,textPos+24,20,0,"I MOT:");
	Cmd_Text(10,36,20,0,"AGC PWM:"); 		Cmd_Text(90,36,20,0,"Leakage:"); 	Cmd_Text(10,textPos+36,20,0,"AGC Error:");
																				Cmd_Text(10,textPos+48,20,0,"HandPieceID:");

	Cmd_Number(65,0,20,0,potVal); 			Cmd_Number(150,0,20,0,phaseCmd);	Cmd_Number(75,textPos,20,0,pumpSpeed);
	Cmd_Number(65,12,20,0,volts); 			Cmd_Number(150,12,20,0,current);	Cmd_Number(75,textPos+12,20,0,sysFail);
	Cmd_Number(65,24,20,0,phaseError);		Cmd_Number(150,24,20,0,freq); 		Cmd_Number(75,textPos+24,20,0,iMot);
	Cmd_Number(65,36,20,0,agcPWM); 			Cmd_Number(150,36,20,0,leakage);	Cmd_Number(75,textPos+36,20,0,agcError);
																				Cmd_Number(75,textPos+48,20,0,handID);
	Cmd_Text(470,240,20, OPT_RIGHT, &software);
	Cmd_Text(362,255,20, 0, "AGC:"); Cmd_Number(392,255,20, 0, AGC_HIGH);

	if(dispTimer>5000)																					// Update values every 0.5 seconds, anything quicker than that will make it unreadable
	{
		potVal = pot_val;
		phaseCmd = EPwm3Regs.CMPA.half.CMPA;
		pumpSpeed = pumpRPM;
		if(AGC_CMD_ePWM==AGC_HIGH&&!sys_fail)															// Only update the variables below when AGC is at AGC_HIGH
		{
			volts = volts_val;
			current = current_val;
			phaseError = phase_error_val;
			iMot = i_mot_val;
			agcPWM = agc_pwm_val;
			leakage = leakage_val;
			agcError = agc_error_val;
		}
		sysFail = sys_fail;
		if(!sys_fail)
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

	ColorARGB(WHITE);
	Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Recoverable Error");
	Cmd_Text(hsize/2,vsize/2+40, 25, OPT_CENTER,"Press Hand Piece Button or");
	Cmd_Text(hsize/2,vsize/2+80, 25, OPT_CENTER,"Footswitch to Reset");
}

void displayTime(unsigned int timeLeft, unsigned useTime, unsigned char runningCondition, unsigned char systemOn, unsigned long barColor, unsigned char textColor)
{
	char runningTime[10]={0,0,0,0,0};
	unsigned int sec1, sec2, min1, min2;
	char *standard_font[11] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",":"};	//Using a custom font requires this lookup array
	unsigned long tColor;
	min1 = (timeLeft/60)/10;
	min2 = (timeLeft/60)%10;
	sec1 = (timeLeft%60)/10;
	sec2 = (timeLeft%60) % 10;

	//use lookup table display text, b/c utf8 encoded you can't just put the string
	strcat(runningTime,standard_font[min1]);																// Construct runningTime into the form MM:SS
	strcat(runningTime,standard_font[min2]);
	strcat(runningTime,standard_font[10]);
	strcat(runningTime,standard_font[sec1]);
	strcat(runningTime,standard_font[sec2]);

	DLStart();																									// Update display
	if(textColor==tWHITE)
	{
		ClearColorRGB(0,0,0);
		Clear(1,1,1);
		tColor=WHITE;
	}
	else
	{
		ClearColorRGB(255,255,255);
		Clear(1,1,1);
		tColor=BLACK;
	}
	if(SHOW_ENG)
		dispEngScreens();

	if(timeLeft<=120)																						// If less than 120 seconds display the following
	{
		ColorARGB(WARNING_COLOR);
		Cmd_Text(440,28,29, OPT_RIGHT, runningTime);
		Begin(FT_BITMAPS);
		Vertex2ii(350,10,HOURGLASS_HANDLE,0);
		End();
	}
	else
	{
		ColorARGB(tColor);
		Cmd_Text(440,28,29, OPT_RIGHT, runningTime);
		Begin(FT_BITMAPS);
		Vertex2ii(350,10,HOURGLASS_HANDLE,0);
		End();
	}
	ColorARGB(tColor);
	Cmd_Text(195, 15, 28, 0, "Time Remaining");
	if(runningCondition==NORMAL)
	{
		updateBarGraph(useTime,timeLeft,barColor,textColor);
		if(timeLeft<=120)
			ColorARGB(WARNING_COLOR);
		else
			ColorARGB(tColor);
		Cmd_Text(390,156,25,0,"30:00");
	}

}

void displayUnrecoverableError(void)
{
	ColorARGB(WHITE);
	Cmd_Text(hsize/2,vsize/2-40, 25, OPT_CENTER,"Unrecoverable Error");
	Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Restart Console");
	Begin(LINES);
	Vertex2ii(30,160,0,0);
	Vertex2ii(450,160,0,0);
	Cmd_Text(hsize/2,vsize/2+45, 25, OPT_CENTER,"Note remaining time before");
	Cmd_Text(hsize/2,vsize/2+85, 25, OPT_CENTER,"restarting the Console");
}

void updateBarGraph(unsigned int maxTime, unsigned int remainingTime, unsigned long barColor, unsigned char textColor)
{
	unsigned int percentDone, barPosition, hPosition;
	char elapsedTime[10]={0,0,0,0,0};
	char *standard_font[11] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",":"};	//Using a custom font requires this lookup array
	unsigned int elapsedMin1, elapsedMin2, elapsedSec1, elapsedSec2;
	static unsigned char barStartPositionSet=0;
	static unsigned int warningBarStart=0;
	unsigned int barStartPosition=100, barEndPosition=380;
	unsigned long tColor;

	elapsedMin1 = ((maxTime-remainingTime)/60)/10;
	elapsedMin2 = ((maxTime-remainingTime)/60)%10;
	elapsedSec1 = ((maxTime-remainingTime)%60)/10;
	elapsedSec2 = ((maxTime-remainingTime)%60) % 10;

	strcat(elapsedTime,standard_font[elapsedMin1]);															// Construct elapsedTime into the form MM:SS
	strcat(elapsedTime,standard_font[elapsedMin2]);
	strcat(elapsedTime,standard_font[10]);
	strcat(elapsedTime,standard_font[elapsedSec1]);
	strcat(elapsedTime,standard_font[elapsedSec2]);

	if(textColor==tWHITE)
		tColor=WHITE;
	else
		tColor=BLACK;

	percentDone=mapRange(maxTime-remainingTime, 0, maxTime,0,100);
	barPosition=mapRange(percentDone, 0, 100, barStartPosition, barEndPosition);
	Begin(RECTS);
	ColorARGB(BAR_BACKGROUND);
	Vertex2ii(barStartPosition,160,0,0);
	Vertex2ii(barEndPosition,190,0,0);
	ColorARGB(barColor);
	Vertex2ii(barStartPosition,160,0,0);
	Vertex2ii(barPosition,190,0,0);
	if(remainingTime<=120)
	{
		if(!barStartPositionSet)
		{
			warningBarStart=barPosition;
			barStartPositionSet=1;
		}
		ColorARGB(WARNING_COLOR);
		Vertex2ii(warningBarStart,160,0,0);
		Vertex2ii(barPosition,190,0,0);
	}
	End();
	Begin(LINES);
	ColorARGB(WHITE);
	Vertex2ii(barPosition,160,0,0);
	Vertex2ii(barPosition,190,0,0);
	End();
	Begin(FT_BITMAPS);
	//v is always 48
	ColorARGB(BUBBLE_COLOR);
	hPosition=mapRange(percentDone,0,100,(barStartPosition-TIMEBUBBLE_WIDTH/2),(barEndPosition-TIMEBUBBLE_WIDTH/2));
	Vertex2ii(hPosition,48,TIMEBUBBLE_HANDLE,0);
	End();
	if(remainingTime<=120)
	{
		ColorARGB(WARNING_COLOR);
		Cmd_Text(TIMEBUBBLE_WIDTH/2+hPosition, 100, 25, OPT_CENTER, elapsedTime);
	}
	else
	{
		ColorARGB(tColor);
		Cmd_Text(TIMEBUBBLE_WIDTH/2+hPosition, 100, 25, OPT_CENTER, elapsedTime);
	}
}

long mapRange(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void displayClosePumpLid(void)
{
	DLStart();
	Cmd_Text(hsize/2,vsize/2-15,24,OPT_CENTER,"Pump Lid Open");
	Cmd_Text(hsize/2,vsize/2+15,24,OPT_CENTER,"Please Close to Continue");
	DLEnd();
	Finish();
}
