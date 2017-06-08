//###########################################################################
//
// FILE:	main.c
// 
// TITLE:	OmniWave Burn-In Software
// Copyright © Cybersonics Inc., 2016
//################################################################################################################################################################################################
//  Ver | mm dd yyyy | Who  | Description of changes
// =====|============|======|===============================================
//  01  | 03 09 2017 | S.R. | 1st Software release - Used W1S02988 - revision 02XA as a start
//							  30 minutes for ultrasound, and 15 minutes of rest
//  02 	| 05 26 2017 | S.R. | Updated based on version 02XK of W1S02915
//	03	| 06 08 2017 | S.R. | Updated based on version 03 of W1S02915
//################################################################################################################################################################################################
#include "math.h"
#include "project.h"
const char software[40] = "W1S02988 - Rev 03";

//################################################################################################################################################################################################
//						Global Variables
//################################################################################################################################################################################################
Uint16 rdata;																							// SPI receive data
unsigned int volts_val;																					// ADC Volts Value (0-4095 - min - max)
unsigned int overV,overI;																				// Variable used to make sure over voltage or over current occurred for more than specified amount of time
unsigned int current_val;																				// ADC Current Value (0-4095 - min - max)
unsigned int phase_val;																					// ADC Phase Value (0-4095 - min - max, 90° is midpoint 2048)
unsigned int i_mot_val;																					// ADC I Motional Value (0-4095 - min - max)
unsigned int phase_error_val;																			// ADC Phase Error Value (0-2048-4095 - 0 is at mid point 2048)
unsigned int vco_in_val;																				// ADC VCO IN Value (0-4095 - min - max)
unsigned int agc_error_val;																				// ADC AGC Error Value (0-2048-4095 - 0 is at mid point 2048)
unsigned int agc_pwm_val;																				// ADC AGC PWM Value (0-4095 - min - max)
unsigned int leakage_val;																				// Leakage Current
unsigned int handpieceID;																				// Handpiece ID
unsigned int sys_fail;																					// Used to force a pedal / switch release after a recoverable fault
unsigned int pll_on;																					// Tells system that PLL is engaged (analog control)
unsigned int lock;																						// Used to inform system that analog PLL control is locked
unsigned int ana_agc_on;																				// Tells system that analog AGC is engaged.
unsigned int sys_on;																					// From main loop to timer0 tells timer0 that user is commanding ultrasound
long unsigned int overload_ctr=0;																		// Determines that system is loaded beyond system power capability or feedback on AGC is faulty
unsigned int capture_ctr;																				// Used to detect initial capture - need N OK events
unsigned int restart_ctr;																				// Number of restarts before foot off pedal required
unsigned int new_ad_vals;																				// A semaphore to coordinate with routines that use A/D vals to make sure each iteration uses new vals
unsigned int sys_watchword;																				// Bits are set by each critical routine that must execute under system state for watchdog to determine proper function
unsigned int sys_watchdog_ctr;																			// Counts before above mentioned bits not being set indicate system malfunction
long unsigned int sys_safety_ctr;																		// If system re-start takes too long, issues a fault
unsigned int sample_window[SAMPLE_WINDOW_VALS];															// Window used in ANA_RUNNING_AND_LOCKED function
unsigned int sample_window_2[SAMPLE_WINDOW_VALS];														// Window used in AGC_SOFT_D_TO_A function
unsigned int ana_run_status;																			// Variable used by ANA_RUNNING_AND_LOCKED function to tell Timer0 status of PLL control
unsigned int pot_val;																					// Co function variable for adjusting I motional current
unsigned int co_on, co_done;																			// Semaphore used to indicate I motional adjust active so other routines do not interfere
unsigned int debounce_ctr;																				// Used for debounce
unsigned int xmt_test_val;
unsigned int no_restart_ctr, lost_lock;

unsigned int agc_delay_ctr, agc_fault_ctr;																// Used for when turning on AGC before locking
unsigned int agc_ramp_down_enabled, agc_ramp_down_delay;												// Used for AGC ramp down
unsigned int low_calibration_complete=0, high_calibration_complete=0;
unsigned int cal_freq_timer=0, freq_delay=0;
unsigned int START_PHASE;
unsigned int sys_stable, Stable_Ctr;
long unsigned int Sys_Monitor_Ctr;
unsigned int agc_duty_position;
unsigned int turnoff_delay=0;
float pf, time_delay, frequency, pf_adj, pf_moving_average, alpha=.001;
signed long phase, old_phase;
unsigned int mm=0,ss=0, timer=0, ms=0;
unsigned char mode=0;																					// Used for controlling different button modes
unsigned int agc_ctr=0, AGC_TIME=1, hardwareFaultCtr=0;
unsigned long tickTime=0, remainingTime=0, primeTimer=0,primeButtonTimer=0,restartedTime=0;
unsigned char energyOn=0;
unsigned int dispTimer=0;
unsigned int handpieceDisconDebounce=0, handpieceConDebounce=0;
unsigned int lockCtr=0,vcoInLowCtr=0;

//Pump variables
unsigned int pumpRPM;
unsigned char newPumpValue=0;
double pulseFreq;

//Default Transducer Settings
unsigned int AGC_HIGH=300;																				// Sets the upper portion of the pulsed AGC command
unsigned int AGC_LOW=10;
long HIGH_FREQ = 40500;
long LOW_FREQ = 37000;
long HIGH_FREQ, LOW_FREQ;
unsigned int 	MOD_FREQ=17;
unsigned int 	DEFAULT_AGC=200;
unsigned int 	duty=100;																				// 0=100% duty cycle
unsigned int 	STABLE_COUNTER = 10;
unsigned int 	START_CMD=10;
states state=READY_TO_PRIME, previousState=sNONE;
unsigned int runTime=1800, useTime=0;

//Display Global Variables
#ifndef RELEASE
unsigned char loadingImages=1;
#else
unsigned char loadingImages=0;
#endif
FT_GEStatus displayStatus=FT_GE_OK;
extern unsigned int hsize, vsize;

#ifndef DEBUG
// Do not remove code below used to put program on flash of DSP
// (must include 28335.cmd and exclude 28335_RAM_lnk.cmd)

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
#endif
//################################################################################################################################################################################################
//						main code									
//################################################################################################################################################################################################
void main(void)
{
	InitSysCtrl();	 																					// Basic Core Initialization
	EALLOW;
	SysCtrlRegs.WDCR = 0x0068;				 															// 0x00AF = Enable Watchdog 0x0068 = Disable Watchdog
	EDIS;

	DINT;																								// Disable all interrupts
#ifndef DEBUG
	// Do not remove code below used to put program on flash of DSP
	// (must include 28335.cmd and exclude 28335_RAM_lnk.cmd)
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, &RamfuncsLoadEnd - &RamfuncsLoadStart);				// Setup program to run in flash
	InitFlash();
#endif

	ana_run_status=SYS_OFF;																				// Set initial values
	sys_fail=INIT_NOT_DONE;
	sys_watchdog_ctr=0;
	sys_watchword=0;
	volts_val=0;
	agc_ramp_down_delay = 0;
	sys_on=0;
	sys_safety_ctr=0;
	co_done=0;
	co_on=0;
	pll_on=0;
	agc_fault_ctr = 0, no_restart_ctr = 0;
	lost_lock = 0;
	START_PHASE=START_PHASE_INIT;
	ultrasound_on_off(ULTRA_OFF);																		// Make sure ultra sound is off
	agc_duty_position = 10000/MOD_FREQ;

	Gpio_select();																						// Setup GPIO's
	Setup_ePWM();																						// Setup ePWM signals
	InitPieCtrl();																						// Basic setup of PIE table; from DSP2833x_PieCtrl.c
	InitPieVectTable();																					// Default ISR's in PIE
	InitAdc();																							// Initialize Analog to Digital converter
	ADCconfig();																						// Configure the ADC
	spi_init();																							// Initialize SPI interface
	spi_fifo_init();																					// Initialize SPI FIFO
	Setup_eCAP();																						// Setup the eCAP pins

	EALLOW;																								// Enable access to protected register
	PieVectTable.TINT0 = &cpu_timer0_isr;																// Re-map entry for Timer0 from ESTOP0 to real interrupt service
	EDIS;																								// Disable access to protected register
	InitCpuTimers();																					// Basic setup CPU Timer0, 1 and 2
	ConfigCpuTimer(&CpuTimer0, 150, 100);																// Initialize Timer0 to 100us (150 = speed of DSP, 100000 = 100ms, 100 = 100us)
	CpuTimer0Regs.TCR.bit.FREE = 0;
	CpuTimer0Regs.TCR.bit.SOFT = 0;
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;																	// Enable interrupt masks, Timer0 connected to group INT1, Bit7

	if(sys_watchword!=0x03FF)																			// All setups done?
	{
		ultrasound_on_off(ULTRA_OFF);
		sys_fail=WATCHDOG_ERROR;
		Unrecoverable_Error();
	}

	IER |= M_INT1;																						// Enable interrupt core line 1 (INT 1), modify register IER acc
	EINT;																								// Enable control interrupts
	ERTM;																								// Enable debug interrupts

	CpuTimer0Regs.TCR.bit.TSS = 0;																		// Start Timer0 Interrupt

	Init(FT_DISPLAY_0);
	Finish();
	loadImages(NULL);
#ifndef RELEASE
	loadingImages=0;
#endif

	unsigned char beepReset=0;
	unsigned long barColor=BAR_COLOR2;

	EnableDog();

#ifdef DEMO_UI
	unsigned char ended=0;
	DisableDog();
	while(1)
	{
		ServiceDog1();
		if(ended==0)
			barColor=BAR_COLOR;
		else
			barColor=BAR_COLOR2;
		if(ended<2)
			runningScreen(runTime,1800,NORMAL,energyOn,barColor,tWHITE);
		if(CpuTimer0.InterruptCount>1&&runTime>0&&ended<2)
		{
			if(runTime<120)
				runTime--;
			else
				runTime-=10;
			CpuTimer0.InterruptCount=0;
		}
		if(runTime==0)
		{
			ended++;
			runTime=1800;
		}
		if(ended==2)
		{
			runningScreen(900,1800,NORMAL,energyOn,BAR_COLOR,tWHITE);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended++;
			}
		}
		if(ended==3)
		{
			runningScreen(900,1800,NORMAL,energyOn,BAR_COLOR2,tWHITE);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended++;
			}
		}
		if(ended==4)
		{
			runningScreen(900,1800,ERROR,energyOn,barColor,tWHITE);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended++;
			}
		}
		if(ended==5)
		{
			runningScreen(60,1800,ERROR,energyOn,barColor,tWHITE);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended++;
			}
		}
		if(ended==6)
		{
			runningScreen(900,1800,UNRECERROR,energyOn,barColor,tWHITE);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended++;
			}
		}
		if(ended==7)
		{
			runningScreen(60,1800,UNRECERROR,energyOn,barColor,tWHITE);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended++;
			}
		}
		if(ended==8)
		{
			runningScreen(60,1800,NORMAL,energyOn,barColor,tBLACK);
			if(PRIME_PB)
			{
				DELAY_US(200000);
				ended=0;
			}
		}
	}
#endif
#ifdef DEBUG
	EALLOW;
	SysCtrlRegs.WDCR = 0x00AF;								// Reset and enable watch dog flag
	EDIS;
#endif
	//Check if system was reset by watchdog
	if(SysCtrlRegs.WDCR & 0x0080)
	{
		DLStart();
		ColorARGB(WHITE);
		Cmd_Text(hsize/2,vsize/2-40, 25, OPT_CENTER,"Unrecoverable Error");
		Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Restart Console");
		DLEnd();
		Finish();
		while(1)
		{
			ServiceDog1();
			ServiceDog2();
		}
	}

	while(1)																							// Main program (keep looping)
	{    
		ServiceDog1();
		if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)														// Detects missing clock and issues failure if sys goes into limp mode.
		{
			ultrasound_on_off(ULTRA_OFF);
			sys_fail=WATCHDOG_ERROR;
			Unrecoverable_Error();
		}
		sys_watchword|=0x00F0;																			// Watchdog identifier for main while loop
		displayStatus=Finish();
		if(displayStatus>FT_GE_FINISHED)
		{
			loadingImages=1;
			DisableDog();
			Reset();
			Init(FT_DISPLAY_0);
			displayStatus=Finish();
			if(displayStatus==FT_GE_OK)
			{
				loadImages(AFTER_FAULT);
				loadingImages=0;
			}
			EnableDog();
		}
		else
		{
			switch(state)																					// State machine for user interface
			{
				case READY_TO_PRIME:
					displayReadyToPrime();
					if(PRIME_PB&&!PUMP_LID_OPEN)
					{
						primeButtonTimer=getTickTime();
						displayPriming();
						state=PRIMING;
						primeTimer = getTickTime();
						PUMP_SPEED = PRIME_SPEED;
					}
					if(PRIME_PB&&PUMP_LID_OPEN)
					{
						displayClosePumpLid();
						previousState=state;
						state=CLOSE_PUMP;
					}
					break;
				case PRIMING:
					HP_LIGHT_OFF;
					if(getTickTime()-primeButtonTimer>1000&&PRIME_PB)										// If still holding button after 1 sec. activate momentary prime button
						state = MOMENTARY_PRIMING;
					else if(getTickTime()-primeButtonTimer>200&&!PRIME_PB)									// If not holding button after 200 ms activate latched auto timed prime
						state = LATCHED_PRIMING;
					if(PUMP_LID_OPEN)
					{
						displayClosePumpLid();
						remainingTime=(getTickTime()-primeTimer);
						previousState=READY_TO_PRIME;
						state=CLOSE_PUMP;
					}
					break;
				case MOMENTARY_PRIMING:
					HP_LIGHT_OFF;
					displayPriming();
					if(!PRIME_PB)
					{
						if(previousState==RUN)
							state=RUN,primeButtonTimer=getTickTime(),PUMP_SPEED = PUMP_OFF;					// If done priming and previous state is RUN return to state RUN
						else if(!sys_fail)
							state=DETECT_DEVICE,primeButtonTimer=getTickTime(),PUMP_SPEED = PUMP_OFF;		// If done priming and previous state is not RUN goto detect device
					}
					if(PUMP_LID_OPEN)
					{
						displayClosePumpLid();
						previousState=READY_TO_PRIME;
						state=CLOSE_PUMP;
					}
					break;
				case LATCHED_PRIMING:
					displayPriming();
					if(PRIME_PB)
						state=DETECT_DEVICE,primeButtonTimer=getTickTime(), PUMP_SPEED = PUMP_OFF;			// If ending latched priming early, go to detect device
					else if(getTickTime()-primeTimer>PRIME_TIME-remainingTime)								// Run latched prime for desired time, if error occurs remainingTime will have a value and adjust accordingly
					{
						state=DETECT_DEVICE;
						PUMP_SPEED = PUMP_OFF;
					}
					if(PUMP_LID_OPEN)
					{
						displayClosePumpLid();
						remainingTime=(getTickTime()-primeTimer);
						previousState=state;
						state=CLOSE_PUMP;
					}
					break;
				case DETECT_DEVICE:
					PUMP_SPEED = PUMP_OFF;																	// Make sure pump is off
					if(!sys_fail)
						mode=0;																				// Only reset button state if no errors
					if(STANDARD_DEVICE)																		// Load settings
					{
						HIGH_FREQ = 41500;
						LOW_FREQ = 39500;
						AGC_HIGH=1900;
						AGC_LOW=10;
						MOD_FREQ = 10;
						duty=85;
						START_PHASE=2000;
						if(sys_fail!=WATCHDOG_ERROR&&sys_fail!=VOLTS_WHILE_OFF)
							sys_fail=INIT_NOT_DONE;
						low_calibration_complete=0;
						high_calibration_complete=0;
						runTime=1800;
						useTime=runTime;
						state=RUN;
					}
					else
					{
						displayConnectDevice();
					}
					if(PRIME_PB&&getTickTime()-primeButtonTimer>200)										// Allow priming device when no device is connected yet, 200ms debounce from the previous state
					{
						previousState=state;
						PUMP_SPEED=PRIME_SPEED;
						state=MOMENTARY_PRIMING;
					}
					break;
				case RUN:
					//*********************************************************Transducer Connected**********************************************************************************
					if(handPieceStatus())																				// Only allow system to run it transducer is connected and debounce count is less than 1000.
					{
						//***********************************************************Button Pressed***********************************************************************************
						if(HSWState()==ON&&!sys_fail)																	// Test if user is commanding US
						{
							PUMP_SPEED = RUN_SPEED;
							HP_LIGHT_ON;
							if(debounce_ctr>10000)
							{
								if((!co_done)&&(ana_run_status!=SYS_RESTART)&&!sys_fail)							// Run Co Adjust, after beeping is done
								{
									energyOn=1;
									co_on=1;																		// Do not do anything other than Co
									if(Cal_I_mot())																	// Co turns on US and nobody else should touch it
									{
										sys_on=1;																	// Tell Timer0 to run US normally
										co_on=0;
									}
									else
									{
										ultrasound_on_off(ULTRA_OFF);
										sys_fail=C0_ERROR;
										Recoverable_Error();														// If Co did not succeed - do not run.
									}
								}
								if(runTime<=120&&energyOn)
								{
									if(runTime==120&&!beepReset)
										doneBeeping(OFF,RESET, NULL, NULL), beepReset=1;
									doneBeeping(ON,DONT_RESET,USE_TIME,2);
								}
							}
						}
						//***********************************************************End Button Pressed******************************************************************************

						//******************************************************************* No Button Pressed**********************************************************************
						else
						{
							if(sys_fail==NONE)
								doneBeeping(OFF,RESET,NULL,NULL);											// Reset the beeper so next start of ultrasound, it will cause a beep 3 times
							PUMP_SPEED = PUMP_OFF;
							if(!sys_fail)
								blinkLight();
							debounce_ctr=0;
							ultrasound_on_off(ULTRA_OFF);
							sys_on=0;
							co_on=0;
							energyOn=0;
							ana_run_status=SYS_OFF;
							restart_ctr=0;
							lost_lock=0;
							Sys_Monitor_Ctr=0;
							no_restart_ctr=0;
							lockCtr=0;
							if (pot_val != START_POT_VAL)													// Make sure pot_val is back to beginning
							{
								pot_val = START_POT_VAL;
								xmt_test_val=spi_xmit(0x1802);
								if(xmt_test_val!=XMT_OK)
									sys_fail=DIG_POT_ERROR,Recoverable_Error();
								xmt_test_val=spi_xmit(pot_val);
								if(xmt_test_val!=XMT_OK)
									sys_fail=DIG_POT_ERROR,Recoverable_Error();
							}
						}
					}
					else if(!sys_fail)																		// If no device return to detect device state
					{
						mode=0;
						BEEPER_OFF;
						ultrasound_on_off(ULTRA_OFF);
						state=DETECT_DEVICE;
					}
					if(PUMP_LID_OPEN)
						sys_fail=PUMP_OPEN;
					if(sys_fail==NONE)																		// As long as there is no failure display remaining time and energy on
						runningScreen(runTime,useTime,NORMAL,energyOn,barColor,tWHITE);
					else if(sys_fail<=OVER_I&&!handPieceStatus()&&sys_fail!=PUMP_ERROR)
					{
						ultrasound_on_off(ULTRA_OFF);
						BEEPER_OFF;
						state=DETECT_DEVICE;
					}
					else if(sys_fail!=NONE&&sys_fail<=OVER_I)												// Play alert beep if there is a recoverable error
					{
						PUMP_SPEED=PUMP_OFF;
						HP_LIGHT_OFF;
						if(alertBeep(ON,DONT_RESET))														// Wait for alertBeep to finish before switching states
						{
							mode=4;
							if((!(previousState==LATCHED_PRIMING||previousState==MOMENTARY_PRIMING))||useTime)
								previousState=state;
							else
								previousState=READY_TO_PRIME;
							state=RECOVERABLE_ERROR;
						}
						runningScreen(runTime,useTime,ERROR,energyOn,barColor,tWHITE);
					}
					else if(sys_fail==INIT_NOT_DONE)
					{
						DLStart();
						Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Device Initializing");
						DLEnd();
						Finish();
					}
					else if(sys_fail==WATCHDOG_ERROR||sys_fail==VOLTS_WHILE_OFF)
					{
						HP_LIGHT_OFF;
						PUMP_SPEED=PUMP_OFF;
						if(alertBeep(ON,DONT_RESET)){state=UNRECOVERABLE_ERROR;}
						runningScreen(runTime,useTime,UNRECERROR,energyOn,barColor,tWHITE);
					}
					else if(sys_fail==PUMP_OPEN)
					{
						mode=0;
						displayClosePumpLid();
						previousState=state;
						state=CLOSE_PUMP;
					}
					//******************************************************************* End No Button Pressed**********************************************************************
					if(PRIME_PB&&getTickTime()-primeButtonTimer>200&&!energyOn&&!sys_fail)					// Momentary priming
					{
						previousState=state;
						state=MOMENTARY_PRIMING;
						PUMP_SPEED = PRIME_SPEED;
					}
					break;
				case DELAY:
					runningScreen(runTime,useTime,NORMAL,energyOn,barColor,tWHITE);
					break;
				case USE_FOOTSWITCH:																	// Tells user to use footswitch if connected and trying to use handswitch
					PUMP_SPEED = PUMP_OFF;
					DLStart();
					Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Press footswitch to activate");
					DLEnd();
					Finish();
					HandSwitch();																		// Resets handswitch state
					if((NO_DEVICE))
					{
						state=DETECT_DEVICE;
					}
					if(!FSW_DETECT)
						state=previousState;
					if(FSW)
						state=RUN;
					break;
				case PROCEDURE_DONE_ALERT:																// State when procedure is done and the alert sound still needs to occur
					PUMP_SPEED = PUMP_OFF;
					ultrasound_on_off(ULTRA_OFF);
					if(alertBeep(ON,DONT_RESET)){state=PROCEDURE_DONE;}
					DLStart();
					Cmd_Text(hsize/2,vsize/2, 25, OPT_CENTER,"Device Use");
					Cmd_Text(hsize/2,vsize/2+40, 25, OPT_CENTER,"Time Expired");
					DLEnd();
					Finish();
					break;
				case PROCEDURE_DONE:																	// Procedure done state, cannot leave this state
					PUMP_SPEED = PUMP_OFF;
					ultrasound_on_off(ULTRA_OFF);
					runTime=900;
					state=DELAY;
					break;
				case RECOVERABLE_ERROR:																	// State to handle recoverable errors and reset them
					PUMP_SPEED = PUMP_OFF;
					ultrasound_on_off(ULTRA_OFF);
					runningScreen(runTime,useTime,ERROR,energyOn,barColor,tWHITE);
					if(HSWState()==RESET_ERROR)
					{
						co_done=0;
						DSP_RESET_PA_L;																	// Make sure SR latch is reset and ready to operate, incase hardware fault occurred
						DELAY_US(1000);
						DSP_RESET_PA_H;
						DELAY_US(1000);
						DSP_RESET_PA_L;
						DELAY_US(1000);
						alertBeep(OFF,RESET);
						doneBeeping(OFF,RESET,NULL,NULL);
						if(sys_fail==PUMP_ERROR&&previousState==MOMENTARY_PRIMING)
							state=READY_TO_PRIME;
						else if(previousState==LATCHED_PRIMING&&remainingTime<PRIME_TIME)
							state=READY_TO_PRIME;
						else
							state=previousState;
						sys_fail=NONE;
					}
					break;
				case UNRECOVERABLE_ERROR:																// If unrecoverable error, stay at this state infinitely
					PUMP_SPEED = PUMP_OFF;
					ultrasound_on_off(ULTRA_OFF);
					runningScreen(runTime,useTime,UNRECERROR,energyOn,barColor,tWHITE);
					break;
				case CLOSE_PUMP:
					PUMP_SPEED = PUMP_OFF;
					displayClosePumpLid();
					if(!PUMP_LID_OPEN)
					{
						if(previousState==LATCHED_PRIMING&&remainingTime<PRIME_TIME)
							state=READY_TO_PRIME;
						else
							state=previousState;
					}
					break;
				default:
					break;
			}
		}
	}
} 

//################################################################################################################################################################################################
//						End of main code
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Timer 0 (100us)
//################################################################################################################################################################################################
void cpu_timer0_isr()
{
	#define VOLTS 		AdcMirror.ADCRESULT0
	#define CURRENT 	AdcMirror.ADCRESULT1
	#define I_MOT		AdcMirror.ADCRESULT2
	#define PHASE		AdcMirror.ADCRESULT3
	#define PHASE_ERROR	AdcMirror.ADCRESULT4
	#define VCO_IN		AdcMirror.ADCRESULT5
	#define AGC_ERROR	AdcMirror.ADCRESULT6
	#define AGC_PWM		AdcMirror.ADCRESULT7
	#define LEAKAGE_OUT AdcMirror.ADCRESULT8
#ifdef NEW_BOARD
	#define HANDPIECE_ID	AdcMirror.ADCRESULT11
#else
#define HANDPIECE_ID	AdcMirror.ADCRESULT10
#endif

	static unsigned long int master_restart_ctr;
	static unsigned int volts_while_off_ctr;
	static unsigned int msTimer=0;
	unsigned int sys_integrity;

	if(handpieceDisconDebounce<=DEBOUNCE_TIME)
		handpieceDisconDebounce++;
	if(handpieceConDebounce<=DEBOUNCE_TIME)
		handpieceConDebounce++;
	if(SHOW_ENG)
		dispTimer++;

	ServiceDog2();
	CpuTimer0.InterruptCount++;																			// Increment global interrupt counter
	if(freq_delay<=FREQ_DELAY_TIME)
		freq_delay++;
	cal_freq_timer++;
	msTimer++;
	if(msTimer>9)
		tickTime++,msTimer=0;
	agc_ctr++;																							// Counter for increasing to final AGC command during turn on

	if((energyOn&&!sys_fail)||state==DELAY)																// Timer stuff
	{
		timer++;
		if(timer>9)
		{
			timer=0;
			ms++;
		}
		if(ms>999)
		{
			ms=0;
			ss++;
			if(runTime>0)
				runTime--;
			if(runTime<=0)
			{
				if(state==DELAY)
				{
					runTime=1800;
					state=RUN;
				}
				else
				{
					energyOn=0;
					sys_on=0;
					co_done=0;
					state=PROCEDURE_DONE;
				}
			}
		}
	}

	//******************************************************Debounce for functions in main while(1) loop*****************************************************************
	if(debounce_ctr<10002)																				// Co Adjust Debounce to only allow Co Adj once
		debounce_ctr++;
	//******************************************************End Debounce for functions in main while(1) loop*************************************************************

	//**************************************************************************AGC Modulation***************************************************************************
	if (sys_stable&&lock)
	{
		if (agc_duty_position>((10000/MOD_FREQ)*0.01*(100-duty)))
		{
			if (AGC_CMD_ePWM<AGC_HIGH)
				AGC_CMD_ePWM=AGC_HIGH;
		}
		else
		{
			if (AGC_CMD_ePWM>AGC_LOW)
				AGC_CMD_ePWM=AGC_LOW;
		}
		agc_duty_position--;
		if (agc_duty_position==0)
			agc_duty_position = 10000/MOD_FREQ;
	}
	//************************************************************************End AGC Modulation*************************************************************************

	//*********************************************************Hardware Safety Circuit Fault Detection*******************************************************************
	if ((TWENTY_AMP_OFF || FIVE_AMP_OFF ||OVERVOLTS_OFF || AGC_AMP_OFF) && sys_fail!=INIT_NOT_DONE)
	{
		if(hardwareFaultCtr>100)																		// Make sure hardware fault is real (10ms)
		{
			ultrasound_on_off(ULTRA_OFF);
			sys_fail=HARDWARE_FAULT_OCCURRED;
		}
		else
			hardwareFaultCtr++;
	}
	else
		hardwareFaultCtr=0;
	//*********************************************************End Hardware Safety Circuit Fault Detection*******************************************************************


	//*************************************************************Recoverable Error Check*******************************************************************************
	// If system is in recoverable error continue lighting the correct lights
	if(sys_fail)
		updateWatchWordWhenFault();
	//*************************************************************End Recoverable Error Check***************************************************************************

	//***********************************************************************ADC Update**********************************************************************************
	if(AdcRegs.ADCST.bit.INT_SEQ1)																		// If an ADC conversion is taking place do the following
	{
		sys_watchword|=0x000F;																			// A/D vals must be updated
		if((VOLTS>3800))																				// Check to make sure voltage and current are within a safe range
		{
			overV++;
			if(overV>10)
			{
				ultrasound_on_off(ULTRA_OFF);
				sys_fail=OVER_V;
				Recoverable_Error();
			}

		}
		else
			overV=0;
		if((CURRENT>3800))																				// Check to make sure voltage and current are within a safe range
		{
			overI++;
			if(overI>10)
			{
				ultrasound_on_off(ULTRA_OFF);
				sys_fail=OVER_I;
				Recoverable_Error();
			}

		}
		else
			overI=0;
		phase_val=PHASE;																				// Store current ADC values in global variables
		i_mot_val=I_MOT;
		phase_error_val=PHASE_ERROR;
		vco_in_val=VCO_IN;
		agc_error_val=AGC_ERROR;
		agc_pwm_val=AGC_PWM;
		volts_val=VOLTS;
		current_val=CURRENT;
		leakage_val=LEAKAGE_OUT;
		handpieceID = HANDPIECE_ID;
		new_ad_vals=1;																					// Tell other routines of new A/D vals
	}
	//*********************************************************************End ADC Update********************************************************************************

	//*********************************************************************Capture Update********************************************************************************
	if(ECap1Regs.ECFLG.bit.CEVT1&&ECap2Regs.ECFLG.bit.CEVT1&&ECap5Regs.ECFLG.bit.CEVT2)											// If capture event 1 occurs on ecap1 do the following and event 4 on ecap5
	{
		old_phase=ECap2Regs.CAP1 - ECap1Regs.CAP1;														// Set value for old_phase (current falling edge-voltage rising edge)
		if (old_phase>-900 && old_phase<900)															// Filter out erroneous values, values should not be any greater than 90° apart
			phase = old_phase;																			// acos(PF=0=90°)*180/PI/(360*freq)*150e6, (i.e. for 42khz 90°=892 but should add some cushion)
		ECap1Regs.ECCLR.bit.CEVT1=1;																	// Get ready for next capture
		ECap1Regs.ECCTL2.bit.REARM=1;
		ECap2Regs.ECCLR.bit.CEVT1=1;
		ECap2Regs.ECCTL2.bit.REARM=1;
		ECap5Regs.ECCLR.bit.CEVT2=1;
		ECap5Regs.ECCTL2.bit.REARM=1;
		frequency = 150e6/ECap5Regs.CAP2;																// Calculate frequency (150e6 is speed of processor)
																										// Read cap value. (Note eCAP5 is setup as difference capture so the measurement is directly the period)
		time_delay = pow((150e6/phase),-1);																// Calculate how much time occurs between the two signals being compared to get the phase angle (theta = 360°*f*time_delay)
		pf = cos((360*frequency*time_delay)*PI/180);													// Calculate power factor
		pf_moving_average = (alpha * pf) + (1-alpha) * pf_moving_average;								// Calculate moving average
		pf_adj = 1.0866*pf_moving_average-.0922;
	}

	//*********************************************************************End Capture Update****************************************************************************

	//***********************************************************************START US MODULE*****************************************************************************
	if(sys_on && co_done && !sys_fail)																	// Sys on and doing Co should not go in here until Co done
	{
		if((!pll_on)&&(!sys_fail)&&no_restart_ctr<=NO_US_START&&restart_ctr<MAX_RESTARTS)				// Start up US
		{
			no_restart_ctr++;																			// Start a counter so if pll isn't started correctly it will time out and issue a fault
			ana_or_dig_freq(ANALOG);																	// Turn on PLL switches, note at this point phase command should be 0 so vco_in_val starts at the lower end
			if(vco_in_val<200)																			// Ensure vco_in_val is low before proceeding
				vcoInLowCtr++;
			else
				vcoInLowCtr=0;
			if(vcoInLowCtr>STARTUP_DELAY)																// Ensure vco_in_val is low for STARTUP_DELAY amount of time
			{
				PHASE_CMD_ePWM=START_PHASE;																// Once we are sure vco_in_val is low then set the phase command that is required
				vcoInLowCtr=0;																			// Reset counter for next lock
				pll_on=1;																				// Indicate to system that pll is on
			}
			sys_watchword|=0x0F00;																		// Update sys watchword
		}
		else if(no_restart_ctr>NO_US_START||restart_ctr>=MAX_RESTARTS)									// If it takes too long for vco_in_val to come down issue a fault
		{
			ultrasound_on_off(ULTRA_OFF);
			sys_fail=CANT_LOCK;
			Recoverable_Error();
		}
		//***********************************************************************END START US MODULE*********************************************************************

		//***********************************************************************MONITOR US MODULE***********************************************************************
		if(restart_ctr<MAX_RESTARTS)																	// Continue to run this as long as system has not restarted too many times
		{
			ANA_RUNNING_AND_LOCKED();																	// Call capture / lock detect routine
			if(ana_run_status==SYS_ANA_RUN_OK)															// If sys locked and OK
			{
				if((!ana_agc_on)&&new_ad_vals&&lock)
					AGC_SOFT_D_TO_A();																	// Do AGC soft start and turn sys over to closed loop AGC
				else if(ana_agc_on&&(PHASE_CMD_ePWM<(START_PHASE+150)))									// After system has locked increase phase command
					PHASE_CMD_ePWM++;
				else if((Stable_Ctr<STABLE_COUNTER)&&ana_agc_on&&(PHASE_CMD_ePWM>=(START_PHASE+150)))
					Stable_Ctr++;
				else if(Stable_Ctr>=STABLE_COUNTER)
				{
					sys_stable=1;
					Sys_Monitor_Ctr=0;
				}
				else
					Sys_Monitor_Ctr++;
				if(Sys_Monitor_Ctr>50000)																// If phase command is too low, the system will sit at the low end of the window
				{
					ultrasound_on_off(ULTRA_OFF);
					sys_fail=PHASE_CMD_LOW;
					Recoverable_Error();
				}
			}
			else if(ana_run_status==SYS_RESTART)														// If sys in re-start, and monitor time out
			{
				master_restart_ctr++;
				sys_watchword|=0x0F00;
				if(master_restart_ctr>200000)
				{
					ultrasound_on_off(ULTRA_OFF);
					sys_fail=MASTER_RESTART_ERROR;
					Recoverable_Error();
				}
			}
			if(ana_run_status!=SYS_RESTART)
			{
				DETECT_AGC_FAULT();
				if(ana_run_status==OVERLOAD_FAULT)														// If sys is overloaded where the feedbacks are maxed out and commands are not met
				{																						// User needs to back off on pressure
					ultrasound_on_off(ULTRA_OFF);
					sys_fail=OVERLOAD_ERROR;
					Recoverable_Error();
				}
				else if(ana_run_status==AGC_FAULT)														// If sys errors are high and there is small feedback, there may be a fault in feedback.
				{
					ultrasound_on_off(ULTRA_OFF);
					sys_fail=AGC_ERROR2;
					Recoverable_Error();
				}
			}
			sys_watchword|=0x0800;
		}
		//***********************************************************************END MONITOR US MODULE*******************************************************************
	}
		//***********************************************************************MONITOR UNEXPECTED RUNNING MODULE*******************************************************
	else if((!co_on)&&(ana_run_status!=SYS_RESTART))													// Should happen only with sys off.
	{
		ana_run_status=SYS_OFF;
		ultrasound_on_off(ULTRA_OFF);
		co_done=0;
		sys_safety_ctr=0;
		master_restart_ctr=0;
		if(volts_val>200)																				// Sys should have no volts if not on
		{
			if(volts_while_off_ctr<10000)
				volts_while_off_ctr++;
			ultrasound_on_off(ULTRA_OFF);
			if(volts_while_off_ctr>=10000)
			{
				ultrasound_on_off(ULTRA_OFF);
				sys_fail=VOLTS_WHILE_OFF;
				Unrecoverable_Error();																	// Turn off and let watchdog reset if sys running when not intended.
			}
		}
		else
			volts_while_off_ctr=0;
		//***********************************************************************END MONITOR UNEXPECTED RUNNING MODULE***************************************************

		//***********************************************************************FREQUENCY WINDOW ADJ MODULE*************************************************************
		if(ECap5Regs.ECFLG.bit.CEVT2)
		{
			frequency = 150e6/ECap5Regs.CAP2;																	// Calculate frequency F=150e6/T
																												// Read cap value. (Note eCAP5 is setup as difference capture so the measurement is directly the period)
			if(cal_freq_timer<10000 && freq_delay>FREQ_DELAY_TIME)												// Adjust low freq for ~1 sec and allow 1ms between adjusting
			{
				if(DIG_FREQ_ePWM != LOW_VCO)																	// Make sure DIG_FREQ_ePWM is at the low end before adjusting "R2" of PLL
					DIG_FREQ_ePWM = LOW_VCO;
				else if(frequency>LOW_FREQ*1.01 && DIG_FREQ_ePWM==LOW_VCO && R2_ePWM>LOW_PWM)					// Coarse Adjust, within 1%
					R2_ePWM-=10;
				else if(frequency<LOW_FREQ*0.99 && DIG_FREQ_ePWM==LOW_VCO && R2_ePWM<HIGH_PWM)					// Coarse Adjust, within 1%
					R2_ePWM+=10;
				else if(frequency>LOW_FREQ*1.002 && DIG_FREQ_ePWM==LOW_VCO && R2_ePWM>LOW_PWM)					// Fine Adjust, within 0.2%
				{
					R2_ePWM--;
					low_calibration_complete = 1;																// Indication that calibration complete
				}
				else if(frequency<LOW_FREQ*0.998 && DIG_FREQ_ePWM==LOW_VCO && R2_ePWM<HIGH_PWM)					// Fine Adjust, within 0.2%
				{
					R2_ePWM++;
					low_calibration_complete = 1;																// Indication that calibration complete
				}
				else if(R2_ePWM>=HIGH_PWM || R2_ePWM<=LOW_PWM)
					low_calibration_complete = 0;
				else
					low_calibration_complete=1;
				freq_delay=0;																					// Reset 1ms delay

			}
			else if(cal_freq_timer>10000 && freq_delay>FREQ_DELAY_TIME && low_calibration_complete)				// Adjust "R1" high end of window for approx. 1 second
			{
				if(DIG_FREQ_ePWM != HIGH_VCO)																	// Make sure DIG_FREQ_ePWM is at the high end before adjusting "R1" of PLL
					DIG_FREQ_ePWM = HIGH_VCO;
				else if(frequency>HIGH_FREQ*1.01 && DIG_FREQ_ePWM==HIGH_VCO && R1_ePWM>LOW_PWM)
					R1_ePWM-=10;
				else if(frequency<HIGH_FREQ*0.99 && DIG_FREQ_ePWM==HIGH_VCO && R1_ePWM<HIGH_PWM)
					R1_ePWM+=10;
				else if(frequency>HIGH_FREQ*1.002 && DIG_FREQ_ePWM==HIGH_VCO && R1_ePWM>LOW_PWM)
				{
					R1_ePWM--;
					high_calibration_complete = 1;
				}
				else if(frequency<HIGH_FREQ*0.998 && DIG_FREQ_ePWM==HIGH_VCO && R1_ePWM<HIGH_PWM)
				{
					R1_ePWM++;
					high_calibration_complete = 1;
				}
				else if(R1_ePWM>=HIGH_PWM || R1_ePWM<=LOW_PWM)
					high_calibration_complete = 0;
				else
					high_calibration_complete=1;
				freq_delay=0;
			}
			else if (cal_freq_timer>20000)
				cal_freq_timer = 0;
			if(low_calibration_complete && high_calibration_complete && sys_fail>=INIT_NOT_DONE && runTime>0)	// Resets sys_fail when freq window is calibrated and when pump door gets open during running
				sys_fail=NONE;																					// Notify system frequency window is calibrated
			else if((!low_calibration_complete||!high_calibration_complete)&&(sys_fail!=WATCHDOG_ERROR&&sys_fail!=VOLTS_WHILE_OFF))
				sys_fail=INIT_NOT_DONE;

			ECap5Regs.ECCLR.bit.CEVT2 = 1;																// Re-enable the CAP2 interrupt
			ECap5Regs.ECCTL2.bit.REARM=1;																// Re-arm the eCAP5
		}
		sys_watchword|=0x0F00;																			// All done check
	}
	//***********************************************************************END FREQUENCY WINDOW ADJ MODULE*************************************************************

	else																								// Should happen during re-start.
	{
		if(!co_on)
			ANA_RUNNING_AND_LOCKED();																	// Keep on calling routine that is controlling the re-start.
		sys_safety_ctr++;
		sys_watchword|=0x0F00;																			// Update sys_watchword
		if(sys_safety_ctr>1e6)																			// If restart takes too long issue fault
		{
			ultrasound_on_off(ULTRA_OFF);
			sys_fail=SYS_SAFETY_ERROR;
			Recoverable_Error();
		}
	}

	//*****************************************************************************WATCHDOG STATUS***********************************************************************
	if(!loadingImages)																					// Don't check while loading images
		sys_integrity=Sys_Watchdog();																	// Check status of watchdog
	else
		sys_integrity=SYS_DOG_OK;
	if(sys_integrity==SYS_DOG_OUT)
	{
		ultrasound_on_off(ULTRA_OFF);
		sys_fail=WATCHDOG_ERROR;
		Unrecoverable_Error();
	}
	//***************************************************************************END WATCHDOG STATUS*********************************************************************
	sys_watchword|=0xF000;																				// Update sys_watchword
	if(!co_on)
		new_ad_vals=0;

	if(ECap4Regs.ECFLG.bit.CEVT2)
	{
		pulseFreq=150e6/(ECap4Regs.CAP2-ECap4Regs.CAP1);												// This gives freq of the pulses -> rev/s, encoder spec = 4 pulses per revolution
		pumpRPM=(pulseFreq*60)/4;																		// 60s in 1 minute, divied by 4 pulses per revolution
		newPumpValue=1;
		ECap4Regs.ECCLR.bit.CEVT2=1;																	// Clear the 2nd capture event flag
		ECap4Regs.ECCTL2.bit.REARM=1;																	// Re-arm the eCAP4
	}
	else if(PUMP_SPEED==PUMP_OFF)
		pumpRPM=0;

	checkPump();

	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;																	// Reset ADC Seq 1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;																	// Clears the SEQ1 interrupt flag bit, INT_SEQ1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;																	// Start an ADC conversion
}
//################################################################################################################################################################################################
//						End Timer 0 (100us)
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						SPI Transmit
//################################################################################################################################################################################################
unsigned int spi_xmit(Uint16 a)
{
	unsigned int spi_test_ctr;
	spi_test_ctr=0;
	SpiaRegs.SPITXBUF=a;																				// Send data to SPI device
	while((SpiaRegs.SPIFFRX.bit.RXFFST!=1)&&(spi_test_ctr<10000))										// Wait until data is received
		spi_test_ctr++;
	if(spi_test_ctr<10000)
	{
		rdata = SpiaRegs.SPIRXBUF;																		// Set rdata to Receive buffer (SDO)
		return XMT_OK;
	}
	else
		return XMT_FAIL;
}
//################################################################################################################################################################################################
//						End SPI Transmit
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Select analog or digital frequency (controls U28 and U11)
//################################################################################################################################################################################################
void ana_or_dig_freq(int a)
{
	if (a == ANALOG)
	{
		ANA_FREQ_ON;
		DIG_FREQ_OFF;
	}
	else
	{
		DIG_FREQ_ON;
		ANA_FREQ_OFF;
	}
}
//################################################################################################################################################################################################
//						Select analog or digital frequency
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Select analog or digital AGC (controls U26 and U12)
//################################################################################################################################################################################################
void ana_or_dig_AGC(int a)
{
	if (a == ANALOG)
	{
		ANA_AGC_ON;
		DIG_AGC_OFF;
	}
	else
	{
		DIG_AGC_ON;
		ANA_AGC_OFF;
	}
}
//################################################################################################################################################################################################
//						End Select analog or digital AGC
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						AGC SOFT D TO A Sub Routine
//################################################################################################################################################################################################
void AGC_SOFT_D_TO_A(void)
{
	unsigned int k;
	unsigned int agc_error_avg;
	static unsigned int dig_agc_ctr;
	for(k=SAMPLE_WINDOW_VALS-1; k>0; k--)																// Initialized to 0 in ultrasound control sub routine
		sample_window_2[k]=sample_window_2[k-1];														// Move last reading down the buffer window (FIFO)
	sample_window_2[0]=agc_error_val;																	// Set current value of agc_error_val to position 0 of sample_window_2
	agc_error_avg=0;																					// Reset agc_error_avg
	for(k=0; k<SAMPLE_WINDOW_VALS; k++)																	// Add up all agc_error_val's
		agc_error_avg+=sample_window_2[k];
	agc_error_avg/=10;																					// Calculate average
	if((lock)||(restart_ctr>=MAX_RESTARTS))																// Wait for lock
	{
		if((DIG_AGC_ePWM<2500)&&(agc_error_avg<2048))													// With a limit on to how high to go, raise digital PWM and try to minimize error for minimum transient.
		{
			if(dig_agc_ctr<1)																			// Slight delay that seems to work well. Increments PWM only every second time thru routine. Allows averaging window to move.
				dig_agc_ctr++;
			else
			{
				dig_agc_ctr=0;																			// Initiate another delay
				DIG_AGC_ePWM++;																			// Increment PWM
			}
		}
		else
		{
			ana_or_dig_AGC(ANALOG);																		// Turn on analog control. Note: Hardware architecture requires BOTH these be turned on for analog control
			if(AGC_CMD_ePWM<AGC_HIGH&&agc_ctr>AGC_TIME)
				AGC_CMD_ePWM++, agc_ctr=0;
			else if (AGC_CMD_ePWM==AGC_HIGH)
				ana_agc_on=1;																			// Set ana_agc_on flag
		}
	}
}
//################################################################################################################################################################################################
//						End AGC SOFT D TO A Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						ANA RUNNING AND lockED Sub Routine
//################################################################################################################################################################################################
void ANA_RUNNING_AND_LOCKED(void)
{
	unsigned int j;
	unsigned int phase_error_avg;
	sys_watchword|=0x0300;																				// Update sys_watchword
	if(new_ad_vals)																						// Perform the following algorithms if the A/D has new values
	{
		for(j=SAMPLE_WINDOW_VALS-1; j>0; j--)															// Sliding window of phase error (FIFO)
			sample_window[j]=sample_window[j-1];
		sample_window[0]=phase_error_val;
		phase_error_avg=0;
		for(j=0; j<10; j++)
			phase_error_avg+=sample_window[j];
		phase_error_avg/=10;																			// Average phase error
		if((!lost_lock)&&(pll_on)&&(restart_ctr<MAX_RESTARTS))											// Monitor lock status
		{
			if((phase_error_avg<2500)&&(phase_error_avg>1800)&&(vco_in_val<3500))						// Phase error within limits and VCO not too far?
			{
				if(capture_ctr<201)																		// True for > 100 times = lock
					capture_ctr++;																		// increment if sys is locked
				if(sys_stable&&lock&&lockCtr<=LOCK_TIME)
					lockCtr++;
				ana_run_status=SYS_ANA_RUN_OK;															// Tell timer0 things OK
			}
			if((capture_ctr>200)&&(!lock))																// Sys locked if phase error is within limits 200 times
			{
				lockCtr=0;
				lock=1;																					// Tell system it is locked
			}
			if(vco_in_val>=3500)																		// Initiate a restart if vco_in_val has flown up to top of window
			{
				ultrasound_on_off(ULTRA_OFF);															// Turn ultrasound off
				restart_ctr++;																			// Increment number of times restart has occurred
				lost_lock=1;																			// Indicate to system that it has lost lock
				pll_on=0;																				// Reset indicator for PLL on
				lock=0;																					// Make sure lock indictor is reset
				no_restart_ctr=0;																		// Restart PLL timer
				restartedTime=getTickTime();															// Grab current tick Time so there is a delay when restarting
				ana_run_status=SYS_RESTART;																// Tell timer0 re-starting
				if(lockCtr>LOCK_TIME)																	// If lockCtr is greater than LOCK_TIME don't allow restarts, only restart at start up
				{
					ultrasound_on_off(ULTRA_OFF);
					sys_fail=CANT_LOCK;
					Recoverable_Error();
				}
				if(START_PHASE>MIN_PHASE+40)															// Decrease phase command if restarting to have better chance of starting
					START_PHASE-=40;
				if(START_PHASE<MIN_PHASE)
					START_PHASE=MIN_PHASE;
				sys_on=0;
			}
		}
		else if(lost_lock&&restart_ctr<MAX_RESTARTS)													// Restart
		{
			if(getTickTime()-restartedTime>RESTART_DELAY)												// Allow restart if delay is up
			{
				sys_on=1;
				lost_lock=0;
				ultrasound_on_off(ULTRA_ON);															// Restart ultrasound
			}
		}
		else if(restart_ctr>=MAX_RESTARTS)																// If attemped to start MAX_RESTARTS then issue a can't lock failure
		{
			ultrasound_on_off(ULTRA_OFF);
			sys_fail=CANT_LOCK;
			Recoverable_Error();
		}
	}
}
//################################################################################################################################################################################################
//						End ANA RUNNING AND lockED Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						DETECT AGC FAULT Sub Routine
//################################################################################################################################################################################################
void DETECT_AGC_FAULT(void)
{
	sys_watchword|=0x0400;
	if(lock&&sys_stable)
	{
		if ((agc_pwm_val>2200)&&((i_mot_val<100)||(volts_val<200)||(current_val<200)))					// AGC has ramped up and is not coming back, (no control of AGC)
			agc_fault_ctr++;
		else
			agc_fault_ctr=0;
		if(((agc_error_val<2048*.9)||(agc_error_val>2048*1.1))&&(agc_pwm_val>2200))						// This is a normal overload
			overload_ctr++;
		else
			overload_ctr=0;

		if(agc_fault_ctr>5000)
			ana_run_status=AGC_FAULT;
		else if(overload_ctr>10000)
			ana_run_status=OVERLOAD_FAULT;
		else
			ana_run_status=SYS_ANA_RUN_OK;
	}
	else
		ana_run_status=SYS_ANA_RUN_OK;
}
//################################################################################################################################################################################################
//						End DETECT AGC FAULT Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Watch Dog Sub Routine
//################################################################################################################################################################################################
unsigned int Sys_Watchdog(void)
{
	unsigned int bone_ctr;
	#define WATCHDOG_MAX	3000
	if(sys_watchword==0xFFFF)																			// sys_watchword must = 0xFFFF or something in the code isn't running properly
	{
		sys_watchword=0;
		WD_BONE_ON;																						// GIVE_DOG_BONE;
		bone_ctr=20;
		while(bone_ctr>0)
			bone_ctr--;
		WD_BONE_OFF;
		sys_watchdog_ctr=0;
		return SYS_DOG_OK;
	}
	else
	{
		sys_watchdog_ctr++;
		if(sys_watchdog_ctr>WATCHDOG_MAX)																// Issue fault if sys_watchword is not 0xFFFF after certain amount of time
		{																								// This will cause the DSP to reset
			ultrasound_on_off(ULTRA_OFF);
			return SYS_DOG_OUT;
		}
		else
			return SYS_DOG_OK;
	}
}
//################################################################################################################################################################################################
//						End Watch Dog Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Co Cal Routine
//################################################################################################################################################################################################
unsigned int Cal_I_mot(void)
{
	#define Co_WINDOW_VALS	10
	#define I_MOT_CTR_MAX	1000000
	unsigned int pot_change;
	unsigned int i_mot_valid_ctr;
	long unsigned int i_mot_ctr;
	DIG_FREQ_ePWM=LOW_VCO;																				// Start @ bottom of window
	DELAY_US(10000);																					// Delay to allow frequency to settle
	ultrasound_on_off(ULTRA_ON);																		// Turn on US
	//set algorithm to determine specified voltage
	DIG_AGC_ePWM=1700;																					// Set Voltage to a reasonable level to allow for accurate C0 calculation
	DELAY_US(10000);
	i_mot_valid_ctr=0;
	pot_val=START_POT_VAL;																				// Set the pot to the extreme value to pass the entire signal
	xmt_test_val=spi_xmit(0x1802);																		// Send initial command to POT to let it know the next command is to set the pot value
	if(xmt_test_val!=XMT_OK)
		sys_fail=DIG_POT_ERROR,Recoverable_Error();
	xmt_test_val=spi_xmit(pot_val);
	if(xmt_test_val!=XMT_OK)
		sys_fail=DIG_POT_ERROR,Recoverable_Error();
	i_mot_ctr=0;
	i_mot_valid_ctr=0;
	while((i_mot_valid_ctr<100)&&(pot_val<0x07ff)&&(i_mot_ctr<I_MOT_CTR_MAX))							// Limit travel of pot / Limit time in loop / Exit loop after 2 instances of 0 I motional
	{
		ServiceDog1();																					// Service dog in this while loop as the main loop does not run and if it takes too long to calculate C0 a watchdog error could occur
		sys_watchword=0xFFF0;
		if(new_ad_vals)
		{
			DINT;																						// Disable interrupt
			if(pot_val<0x07f0)																			// Limit pot movement
			{
				if(i_mot_val>50)																		// Consider 50 A/D counts in the noise, goal is to zero out I_MOT
				{
					pot_val+=1;
					pot_change=1;																		// Write to pot only if there is change
					i_mot_valid_ctr=0;
				}
				else
				{
					pot_change=0;
					i_mot_valid_ctr++;
				}
				if(pot_change)																			// Change pot_val
				{
					xmt_test_val=spi_xmit(0x1802);
					if(xmt_test_val!=XMT_OK)
						sys_fail=DIG_POT_ERROR,Recoverable_Error();
					xmt_test_val=spi_xmit(pot_val);
					if(xmt_test_val!=XMT_OK)
						sys_fail=DIG_POT_ERROR,Recoverable_Error();
				}
			}
			new_ad_vals=0;																				// Need new A/D vales before next adjustment
			EINT;																						// Enable interrupt to get new A/D values
		}
		i_mot_ctr++;
	}
	DIG_FREQ_ePWM=LOW_VCO;																				// Set digital frequency low so when the PLL is activated it can try and capture
	DIG_AGC_ePWM=AGC_START_VAL;
	if((pot_val>=0x07fe)||(i_mot_ctr>=I_MOT_CTR_MAX))													// If sys could not zero out I motional, i_mot_ctr would have counted out
		return I_MOT_NG;
	else																								// System succussfully zeroed out I_MOT
	{
		co_done=1;
		return I_MOT_OK;
	}
}
//################################################################################################################################################################################################
//						End Co Cal Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Ultrasound Control Sub Routine
//################################################################################################################################################################################################
void ultrasound_on_off(int a)
{
	unsigned int j;
	if (a == ULTRA_OFF)
	{
		AGC_CMD_ePWM=DEFAULT_AGC;
		DIG_AGC_ePWM=AGC_START_VAL;
		ana_or_dig_AGC(DIGITAL);																		// Switch AGC to digital
		while (turnoff_delay<10000&&agc_ramp_down_enabled)
			sys_watchword |= 0x00F0, turnoff_delay++,DIG_FREQ_ePWM=LOW_VCO;								// Delay to allow AGC to come down before frequency is moved
		ana_or_dig_freq(DIGITAL);																		// Switch frequency to digital
		agc_ramp_down_enabled = 0;
		turnoff_delay = 0;
		SYS_ON_2_OFF;
		ULTRASOUND_OFF;																					// Turn off ultra sound and restore flags and variables to initial state
		agc_delay_ctr=0;
		PHASE_CMD_ePWM=0;																				// Set phase command to 0 when idle/when starting to ensure vco_in_val is low when activating pll switch
		pll_on=0;
		lock=0;
		for(j=0; j<10; j++)
			sample_window[j]=0;
		for(j=0; j<10; j++)
			sample_window_2[j]=0;
		ana_agc_on=0;
		capture_ctr=0;
		sys_stable=0;
		Stable_Ctr=0;
		agc_fault_ctr=0;
		overload_ctr=0;
	}
	else if (a == ULTRA_ON)
	{
		SYS_ON_2_ON;																					// Turn on U30 dual relay
		DELAY_US(15000);																				// Delay in case U30 dual relay doesn't switch at same speed
		ULTRASOUND_ON;																					// Turn on ultra sound
		agc_ramp_down_enabled = 1;
	}
}
//################################################################################################################################################################################################
//						End Ultrasound Control Sub Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Unrecoverable error Sub Routine
//################################################################################################################################################################################################
void Unrecoverable_Error(void)
{
	unsigned int bone_ctr;
	ultrasound_on_off(ULTRA_OFF); if(state!=UNRECOVERABLE_ERROR) {state=RUN;}
	sys_watchword|=0xFFF0;
	WD_BONE_ON;																						// GIVE_DOG_BONE;
	bone_ctr=20;
	while(bone_ctr>0)
		bone_ctr--;
	WD_BONE_OFF;

}
//################################################################################################################################################################################################
//						End Unrecoverable error Sub Routine
//################################################################################################################################################################################################

//################################################################################################################################################################################################
//						Recoverable error Sub Routine
//################################################################################################################################################################################################
void Recoverable_Error(void)
{
	ultrasound_on_off(ULTRA_OFF);
}
//################################################################################################################################################################################################
//						End Recoverable error Sub Routine
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Updates the watchword when there is an error
//################################################################################################################################################################################################
void updateWatchWordWhenFault(void)
{
	sys_watchword |= 0x0F00;
}


//################################################################################################################################################################################################
//						Returns the current CpuTimer0Interrupt counter value
//################################################################################################################################################################################################
unsigned long getTickTime(void)
{
	return tickTime;																					// Used for delays
}


//################################################################################################################################################################################################
//						Function used for blinking the hand piece LED
//################################################################################################################################################################################################
void blinkLight(void)
{
	static unsigned long hpTimer;
	static unsigned int blinkState=0;
	switch(blinkState)
	{
		case 0:
			hpTimer=getTickTime();
			blinkState=1;
			break;
		case 1:
			if(getTickTime()<hpTimer+500)
				HP_LIGHT_OFF;
			else if(getTickTime()<hpTimer+1000)
				HP_LIGHT_ON;
			else
				blinkState=0;
			break;
		default:
			break;
	}
}


//################################################################################################################################################################################################
//						Fuction to allow momentary action on the hand piece by pressing and holding button or latching action by double clicking button
//################################################################################################################################################################################################
unsigned char HSWState(void)
{
	unsigned char returnValue=OFF, buttonPressed=0;

	if(FSW_DETECT)
	{
		buttonPressed = FSW;
		if(HandSwitch()&&!energyOn)																		// If user is trying to use hand piece button when foot switch is plugged switch states to notify user
		{
			previousState=state;
			state=USE_FOOTSWITCH;
		}
	}
	else
		buttonPressed = HandSwitch();

	switch( mode )
	{
		case 0:																							// Detect 1st button down
			if (buttonPressed)
				mode = 1;																				// First button press detected move to the next detection state (1st release)
			break;
		case 1:																							// Detect 1st button release
			returnValue = ON;
			if(!buttonPressed)
				mode = 2;
			break;
		case 2:																							// Detect button down after system turned on
			returnValue = ON;																				// Keep system on until another press
			if(buttonPressed)
			{
				returnValue = OFF;
				mode = 3;
			}
			break;
		case 3:																							// Detect button release after turning off
			if(!buttonPressed)
			{
				mode=0;
			}
			break;
		case 4:																							// Error
			returnValue=OFF;
			if(buttonPressed)
				mode=5;
			break;
		case 5:
			if(!buttonPressed)
			{
				returnValue=RESET_ERROR;
				mode=0;
			}
			break;
	}
	return returnValue;
}


//################################################################################################################################################################################################
//						Function to beep beeper three times before starting ultrasound
//################################################################################################################################################################################################
unsigned char doneBeeping(unsigned char shouldBeep, unsigned char reset, unsigned char alert, unsigned char beeps)
{
	static unsigned long beepTimer;
	static unsigned int beepState=0, beepCount=0, beepTime;
	if(reset)
		beepCount=0, beepState=0;
	if(alert==STARTUP)
		beepTime=500;
	else
		beepTime=250;
	if(shouldBeep&&beepCount<beeps)
	{
		switch(beepState)
		{
			case 0:
				beepTimer=getTickTime();
				beepState=1;
				break;
			case 1:
				if(getTickTime()<beepTimer+beepTime)
					BEEPER_ON;
				else if(getTickTime()<beepTimer+beepTime*2)
					BEEPER_OFF;
				else
				{
					beepState=0;
					beepCount++;
				}
				break;
			default:
				break;
		}
	}
	else
		BEEPER_OFF;
	if(beepCount>=beeps)
		return 1;
	else
		return 0;
}


//################################################################################################################################################################################################
//						Function to beep beeper for alerts (i.e. failure modes)
//################################################################################################################################################################################################

unsigned char alertBeep(unsigned char shouldBeep, unsigned char reset)
{
	static unsigned long alertTimer, pauseTimer;
	static unsigned int alertState=0, alertCount=0;
	if(reset)
		alertCount=0, alertState=0;
	if(shouldBeep)
	{
		switch(alertState)
		{
			case 0:
				alertTimer=getTickTime();
				alertState=1;
				break;
			case 1:
				if(getTickTime()<alertTimer+125)
					BEEPER_ON;
				else if(getTickTime()<alertTimer+250)
					BEEPER_OFF;
				else
				{
					alertState=0;
					alertCount++;
					if(alertCount==3)
						alertState=2;
				}
				break;
			case 2:
				pauseTimer=getTickTime();
				alertState=3;
				break;
			case 3:
				if(getTickTime()-pauseTimer>500)
					alertState=0;
			default:
				break;
		}
	}
	if(alertCount>=6)
		return 1;
	else
		return 0;
}


//################################################################################################################################################################################################
//						Function to debounce hand switch
//################################################################################################################################################################################################
unsigned char HandSwitch(void)
{
   static unsigned char debounce;
   static unsigned long last_msec;
   static unsigned char hsw_state = 0;

   // If asking again too soon, just give same answer as before (only checks every 10ms)
   if ( last_msec == getTickTime() )
	   return hsw_state;
   last_msec = getTickTime();

   if (HSW == hsw_state)
	   debounce = 0; 					// Foot switch in same state? Reset timer
   else									// New state
   {
      debounce++;                      	// Count number of times new state seen
      if ( debounce > 3 )  {          	// still give old answer until debounce satisfied
      hsw_state = !hsw_state;         	// Okay debounce satisfied, flip state
      debounce = 0;
      }
   }
   return hsw_state;
}


//################################################################################################################################################################################################
//						Function to check pump speed when it is running
//################################################################################################################################################################################################
void checkPump(void)
{
	static unsigned char errorCtr=0,pumpTimer=0,pumpSpeed=0,noNewPumpValCtr=0;
	if(PUMP_SPEED!=PUMP_OFF)																			// Check to see if pump is supposed to be running
	{
		if(CpuTimer0.InterruptCount>2000)																// Monitor pump every 200ms
		{
			CpuTimer0.InterruptCount=0;																	// Reset 10ms timer
			if(pumpTimer<6)																				// Timer used for setting the current pumpSpeed used for comparing to the "real time speed"
				pumpTimer++;
			if(pumpTimer>5&&pumpRPM==0)																	// Check if pump is not running when it is supposed to be
			{
				errorCtr++;
			}
			if(pumpTimer>5&&pumpRPM&&!pumpSpeed)														// Set pumpSpeed after 1 second of running
				pumpSpeed=pumpRPM;
			if(newPumpValue)
			{
				if(pumpSpeed&&(pumpRPM<pumpSpeed*.8||pumpRPM>pumpSpeed*1.2)&&pumpRPM!=65535)			// Allow 20% variablity pump's encoder isn't very accurate, RPM of 65535 is errouneous don't allow error
				{
					errorCtr++;
					noNewPumpValCtr=0;
					newPumpValue=0;
				}
				else
				{
					errorCtr=0;
					noNewPumpValCtr=0;
					newPumpValue=0;
				}
			}
			else if(!newPumpValue)																		// Check to make sure pump pulses are setting triggering the ECap4 unit
				noNewPumpValCtr++;																		// At the maximum RPM (~135) the pump speed can only be updated at about every 111ms, because of the encoder, 4 pulses/rev
			if(errorCtr>5||noNewPumpValCtr>10)															// Error occurred for >~1 second, or can't capture pump pulses on ECap4 >~2s
			{
				errorCtr=0;
				noNewPumpValCtr=0;
				sys_fail=PUMP_ERROR;																	// Update sys_fail
				previousState=state;
				if(previousState==LATCHED_PRIMING&&(getTickTime()-primeTimer)<PRIME_TIME)
					remainingTime=getTickTime()-primeTimer;
				else if (previousState==MOMENTARY_PRIMING&&(getTickTime()-primeTimer)<PRIME_TIME)
					remainingTime=0;																	// Reset remainingTime so it goes back to ready to prime state
				else
					remainingTime=PRIME_TIME;
				state=RUN;																				// Update state
			}
		}
	}
	else
		pumpTimer=0,pumpSpeed=0,errorCtr=0,noNewPumpValCtr=0;
}



//##################################################################################################################################################################
//						HandPieceStatus Debounce
//						Returns a 0 if handpiece is disconnected
//						Returns a 1 if handpiece is connected
//##################################################################################################################################################################
unsigned char handPieceStatus(void)
{
	static unsigned char handpieceState=0;
	if(NO_DEVICE)
	{
		if(handpieceDisconDebounce>DEBOUNCE_TIME)
		{
			handpieceConDebounce=0;
			handpieceState=0;
		}
	}
	else
	{
		if(handpieceConDebounce>DEBOUNCE_TIME)
		{
			handpieceDisconDebounce=0;
			handpieceState=1;
		}
	}
	return handpieceState;
}
//##################################################################################################################################################################
//						End HandPieceStatus Debounce
//##################################################################################################################################################################
