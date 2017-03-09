/*
 * project.h
 *
 *  Created on: Jul 10, 2015
 *      Author: srock
 */

#ifndef OMNIWAVE_PROJECT_H_
#define OMNIWAVE_PROJECT_H_

#include "FT800.h"
#include "display.h"
#include "DSP2833x_EPwm_defines.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "DSP2833x_eCAP_defines.h"
#include "main_defines.h"

#define McBSPA_28335
void Gpio_select(void);

#ifndef DEBUG
// Do not remove code below used to put program on flash of DSP
// (must include 28335.cmd and exclude 28335_RAM_lnk.cmd)
void InitFlash(void);
#endif
void ADCconfig(void);
void Setup_ePWM(void);
void Setup_eCAP(void);
unsigned int spi_xmit(Uint16 a);
void spi_init(void);
void spi_fifo_init(void);
void updateWatchWordWhenFault(void);
void ultrasound_on_off(int a);
void DETECT_AGC_FAULT(void);
void AGC_SOFT_D_TO_A(void);
void ana_or_dig_freq(int a);
void ana_or_dig_AGC(int a);
unsigned int Cal_I_mot(void);
void ANA_RUNNING_AND_LOCKED(void);
unsigned int Sys_Watchdog(void);
interrupt void cpu_timer0_isr(void);
void Unrecoverable_Error(void);
void Recoverable_Error(void);
unsigned long getTickTime(void);
void blinkLight(void);
unsigned char HSWState(void);
unsigned char doneBeeping(unsigned char shouldBeep, unsigned char reset);
unsigned char alertBeep(unsigned char shouldBeep, unsigned char reset);
unsigned char HandSwitch(void);
void checkPump(void);

extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitAdc(void);


#endif /* OMNIWAVE_PROJECT_H_ */
