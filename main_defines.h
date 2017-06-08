/*
 * main_defines.h
 *
 *  Created on: Oct 13, 2014
 *      Author: srock
 */

#ifndef OMNIWAVE_MAIN_DEFINES_H_
#define OMNIWAVE_MAIN_DEFINES_H_

//#define DEMO_UI
//#define NO_TIME_LIMIT
//#define NO_BEEPER
//#define NO_PUMP_DOOR
#define NEW_BOARD

typedef enum states
{
	sNONE,
	READY_TO_PRIME,
	PRIMING,
	MOMENTARY_PRIMING,
	LATCHED_PRIMING,
	DETECT_DEVICE,
	USE_FOOTSWITCH,
	RUN,
	DELAY,
	PROCEDURE_DONE_ALERT,
	PROCEDURE_DONE,
	RECOVERABLE_ERROR,
	UNRECOVERABLE_ERROR,
	CLOSE_PUMP
} states;

#define APP_VERSION     3       //<! Define version for use in strings
#define NONE			0
#define HSW				handpieceID<250
#define PRIME_PB		!GpioDataRegs.GPBDAT.bit.GPIO40
#define FSW				(!GpioDataRegs.GPADAT.bit.GPIO27)
#define FSW_DETECT		(!GpioDataRegs.GPCDAT.bit.GPIO81)
#ifdef NO_PUMP_DOOR
#define PUMP_LID_OPEN	0
#else
#define PUMP_LID_OPEN	GpioDataRegs.GPCDAT.bit.GPIO71
#endif
#define DONT_RESET		0
#define RESET			1
#define OFF				0
#define ON				1
#define SHOW_ENG		!GpioDataRegs.GPADAT.bit.GPIO1

#define HP_LIGHT_ON GpioDataRegs.GPCCLEAR.bit.GPIO70 =1
#define HP_LIGHT_OFF GpioDataRegs.GPCSET.bit.GPIO70 =1

#define PUMP_ERROR				3
#define HARDWARE_FAULT_OCCURRED	4
#define HARDWARE_FAULT_CLEARED	5
#define C0_ERROR				6
#define DIG_POT_ERROR			7
#define CANT_LOCK				8
#define PHASE_CMD_LOW			9
#define MASTER_RESTART_ERROR	10
#define OVERLOAD_ERROR			11
#define AGC_ERROR2				12
#define SYS_SAFETY_ERROR		13
#define OVER_V					14
#define OVER_I					15
#define WATCHDOG_ERROR			16
#define VOLTS_WHILE_OFF			17
#define INIT_NOT_DONE			18
#define PUMP_OPEN				19

#define NO_DEVICE		handpieceID>4000
#define STANDARD_DEVICE handpieceID>2000&&handpieceID<2300


#define TWENTY_AMP_OFF	GpioDataRegs.GPCDAT.bit.GPIO75
#define FIVE_AMP_OFF	GpioDataRegs.GPCDAT.bit.GPIO76
#define OVERVOLTS_OFF	GpioDataRegs.GPCDAT.bit.GPIO77
#define AGC_AMP_OFF		GpioDataRegs.GPCDAT.bit.GPIO78

//Output Defines
#define WD_BONE_ON		GpioDataRegs.GPASET.bit.GPIO12 		= 1										// Toggle this to reset hardware watchdog (every 3.4ms)
#define WD_BONE_OFF		GpioDataRegs.GPACLEAR.bit.GPIO12 	= 1

#define ULTRASOUND_OFF	GpioDataRegs.GPCSET.bit.GPIO68 		= 1										// Turns the system off
#define ULTRASOUND_ON	GpioDataRegs.GPCCLEAR.bit.GPIO68 	= 1										// Turns the system on
#define ULTRA_ON		1
#define ULTRA_OFF		0

#define DIG_FREQ_OFF	GpioDataRegs.GPBSET.bit.GPIO36 		= 1										// Turn off Digital freq and allow use of analog freq (4046 PLL chip)
#define DIG_FREQ_ON		GpioDataRegs.GPBCLEAR.bit.GPIO36 	= 1										// Turn on Digital AGC

#define ANA_FREQ_ON		GpioDataRegs.GPBSET.bit.GPIO37 		= 1										// Turn on Analog freq (4046 PLL chip)
#define ANA_FREQ_OFF	GpioDataRegs.GPBCLEAR.bit.GPIO37 	= 1										// Turn off analog freq and allow use of digital freq

#define DIG_AGC_OFF		GpioDataRegs.GPASET.bit.GPIO13 		= 1										// Turn off Digital AGC and allow use of analog AGC
#define DIG_AGC_ON		GpioDataRegs.GPACLEAR.bit.GPIO13 	= 1										// Turn on Digital AGC

#define ANA_AGC_ON		GpioDataRegs.GPBSET.bit.GPIO35 		= 1										// Turn on Analog AGC
#define ANA_AGC_OFF		GpioDataRegs.GPBCLEAR.bit.GPIO35 	= 1										// Turn off analog AGC and allow use of digital AGC

#define DSP_RESET_PA_H	GpioDataRegs.GPBSET.bit.GPIO52 		= 1										// Reset SR Latch on Power Board (Bridge and AGC Errors)
#define DSP_RESET_PA_L	GpioDataRegs.GPBCLEAR.bit.GPIO52	= 1										// Use to clear reset on SR Latch on Power Board

#define SYS_ON_2_ON	GpioDataRegs.GPBSET.bit.GPIO59			= 1										// Use to turn on/open LBB127 on power board
#define SYS_ON_2_OFF	GpioDataRegs.GPBCLEAR.bit.GPIO59	= 1										// Use to turn off/close LBB127 on power board for proper shutdown of system

#ifdef NEW_BOARD
#define BEEPER_OFF	GpioDataRegs.GPCSET.bit.GPIO69			= 1
#ifndef NO_BEEPER
#define BEEPER_ON	GpioDataRegs.GPCCLEAR.bit.GPIO69		= 1
#else
#define BEEPER_ON	GpioDataRegs.GPCSET.bit.GPIO69			= 1
#endif
#else
#ifdef NO_BEEPER
#define BEEPER_ON	GpioDataRegs.GPCCLEAR.bit.GPIO69		= 1
#else
#define BEEPER_ON	GpioDataRegs.GPCSET.bit.GPIO69			= 1
#endif
#define BEEPER_OFF	GpioDataRegs.GPCCLEAR.bit.GPIO69		= 1
#endif

//ePWM Defines
#define R1_ePWM 		EPwm1Regs.CMPA.half.CMPA
#define R2_ePWM 		EPwm2Regs.CMPA.half.CMPA
#define AGC_Min_ePWM 	EPwm2Regs.CMPB
#define PHASE_CMD_ePWM 	EPwm3Regs.CMPA.half.CMPA
#define DIG_FREQ_ePWM 	EPwm4Regs.CMPA.half.CMPA													// Sets VCO_IN level which corresponds to the upper and lower window range of 4046
#define PUMP_SPEED		EPwm4Regs.CMPB																// Sets pump speed
#define AGC_CMD_ePWM 	EPwm5Regs.CMPA.half.CMPA
#define DIG_AGC_ePWM 	EPwm6Regs.CMPA.half.CMPA

//Constant Variable Defines
#define ANALOG 	0
#define DIGITAL 1
#define PLL_CTR_MAX			100000
#define AGC_MIN_VAL 		675
#define AGC_START_VAL 		200
#define AGC_COMMAND_VAL 	50
#define START_PHASE_INIT 	1500
#define MIN_PHASE			1000
#define SYS_ANA_RUN_OK  	0
#define SYS_RESTART 		1
#define AGC_FAULT 			3
#define OVERLOAD_FAULT		4
#define SYS_OFF				10
#define SAFETY_CTR_MAX		5000
#define SYS_DOG_OUT 		0
#define SYS_DOG_OK	 		1
#define XMT_OK 				1
#define XMT_FAIL 			0
#define HIGH_VCO 			2790																		// Set Point for setting VCO to ~5V
#define LOW_VCO  			10																			// Set Point for setting VCO to ~0V
#define HIGH_PWM			2800
#define LOW_PWM				40
#define SAMPLE_WINDOW_VALS	10
#define LARGE_AGC			500
#define R1_INIT_VAL			1500
#define R2_INIT_VAL 		1500
#define I_MOT_OK			1
#define I_MOT_NG			0
#define START_POT_VAL		0x0401
#define HSW_DEBOUNCE_CTR	1000
#define MAX_RESTARTS		4
#define WINDOW_ADJ_PERIOD	500
#define PI 3.1415
#define PRIME_SPEED			3000
#define RUN_SPEED			1600
#define PUMP_OFF			0
#define PRIME_TIME			50000
#define FREQ_DELAY_TIME		10
#define STARTUP				1
#define USE_TIME			2
#define NULL				0
#define AFTER_FAULT			1
#define ERROR				1
#define NORMAL				2
#define UNRECERROR			3
#define DEBOUNCE_TIME		2000
#define LOCK_TIME			10000
#define STARTUP_DELAY		100
#define NO_US_START			10000
#define RESTART_DELAY		100
#define RESET_ERROR			2

#endif /* OMNIWAVE_MAIN_DEFINES_H_ */
