/*
 * display.h
 *
 *  Created on: Mar 23, 2016
 *      Author: srock
 */

#ifndef OMNIWAVE_DISPLAY_H_
#define OMNIWAVE_DISPLAY_H_

void loadImages(void);
void runningScreen(unsigned int timeLeft, unsigned int useTime, unsigned char runningCondiditon, unsigned char systemOn, unsigned long barColor, unsigned char textColor);
void displayReadyToPrime(void);
void displayPriming(void);
void displayConnectDevice(void);
void dispEngScreens(void);
void displayRecoverableError(void);
void displayTime(unsigned int timeLeft, unsigned int useTime, unsigned char runningCondidion, unsigned char systemOn, unsigned long barColor, unsigned char textColor);
void displayUnrecoverableError(void);
void updateBarGraph(unsigned int maxTime, unsigned int remainingTime, unsigned long barColor, unsigned char textColor);
long mapRange(long x, long in_min, long in_max, long out_min, long out_max);
void displayClosePumpLid(void);

#define TIMER_GREEN 		0x5000641E
#define BUBBLE_COLOR		0xFF383838
#define BAR_BACKGROUND		0xFFB3B3B3
#define BAR_COLOR			0xFFA55800
#define BAR_COLOR2			0xFF0F4FFF
#define WARNING_COLOR		0xFFDFE214



#endif /* OMNIWAVE_DISPLAY_H_ */
