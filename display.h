/*
 * display.h
 *
 *  Created on: Mar 23, 2016
 *      Author: srock
 */

#ifndef OMNIWAVE_DISPLAY_H_
#define OMNIWAVE_DISPLAY_H_

void loadImages(void);
void loadFont(void);
void displayOmniLogo(void);
void time(unsigned int timeLeft, unsigned char systemOn);
void displayReadyToPrime(void);
void displayPriming(void);
void displayConnectDevice(void);
void dispEngScreens(void);
void displayRecoverableError(void);

#define TIMER_GREEN 		0x5000641E



#endif /* OMNIWAVE_DISPLAY_H_ */
