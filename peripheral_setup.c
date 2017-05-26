/*
 * peripheral_setup.c
 *
 *  Created on: Mar 23, 2016
 *      Author: srock
 */
#include "project.h"
extern unsigned int START_PHASE;
extern unsigned int sys_watchword;
//################################################################################################################################################################################################
//						GPIO Setup
//################################################################################################################################################################################################

void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = 0;																		// GPIO0  ... GPIO15 = General Purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;																// Set GPIO 0  as EPWM1A (PWM for R1 of 4046 Chip)
	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1;																// Set GPIO 2  as EPWM2A (PWM for R2 of 4046 Chip)
	GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 1;																// Set GPIO 3  as EPWM2B (PWM for AGC_MIN_LEVEL_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1;																// Set GPIO 4  as EPWM3A (PWM for PHASE_COMND_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 1;																// Set GPIO 6  as EPWM4A (PWM for DIG_FREQ_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO7	= 1;																// Set GPIO 7  as EPWM4B (PWM for pump)
	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 1;																// Set GPIO 8  as EPWM5A (PWM for AGC_COMMAND_PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;																// Set GPIO 10 as EPWM6A (PWM for DIG_AGC_PWM)
	GpioCtrlRegs.GPAPUD.bit.GPIO11	= 0;																// Enable pull up on GPIO11
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 3;																// Set GPIO 11 as eCAP4  (used for measuring pump speed)

	GpioCtrlRegs.GPAMUX2.all = 0;																		// GPIO16 ... GPIO31 = General Purpose I/O
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;																// GPIO24 as eCAP1 for detecting voltage freq/phase
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;																// GPIO25 as eCAP2 for detecting current freq/phase
	GpioCtrlRegs.GPBMUX1.all = 0;																		// GPIO32 ... GPIO47 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;																		// GPIO48 ... GPIO63 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 1;																// GPIO48 as eCAP5 for detecting VCO frequency
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;																// Set GPIO54 as SPISIMOA
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;																// Set GPIO55 as SPISOMIA
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;																// Set GPIO56 as SPICLKA
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1;																// Set GPIO57 as SPISTEA
	GpioCtrlRegs.GPCMUX1.all = 0;																		// GPIO64 ... GPIO79 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;																		// GPIO80 ... GPIO87 = General Purpose I/O

	GpioCtrlRegs.GPADIR.all = 0xFFFFFFFF;																// GPIO0  ... GPIO31 as outputs by default because unused GPIO's are unconnected

	GpioCtrlRegs.GPADIR.bit.GPIO1  = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO7  = 1;//debug
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;																	// Set GPIO 12 as output (WD_BONE)
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;																	// Set GPIO 13 as output (ANA_OR_DIG_AGC)
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;																	// HP_LED
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;																	// DSP SYS_ON_2
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;																	// Footswitch

	GpioCtrlRegs.GPBDIR.all = 0xFFFFFFFF;																// GPIO32 ... GPIO63 as outputs by default because unused GPIO's are unconnected

	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;																	// Set GPIO 35 as output (ANA_OR_DIG_AGC2)
	GpioCtrlRegs.GPBDIR.bit.GPIO36 = 1;																	// Set GPIO 35 as output (ANA_OR_DIG_FREQ)
	GpioCtrlRegs.GPBDIR.bit.GPIO37 = 1;																	// Set GPIO 35 as output (ANA_OR_DIG_FREQ2)
	GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0;																	// Set GPIO 40 as input (PRIME PB)

	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;																	// Set GPIO 52 as output (DSP_RESET_PA)
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;																	// Set GPIO 58 as output F_SEL_1
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;																	// Set GPIO 59 as output 20KHZ_ON

	GpioCtrlRegs.GPCDIR.all = 0xFFFFFFFF;																// GPIO64 ... GPIO87 as outputs by default because unused GPIO's are unconnected
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;																	// Set GPIO 65 as output V_GAIN
	GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;																	// Set GPIO 66 as output I_GAIN
	GpioCtrlRegs.GPCDIR.bit.GPIO68 = 1;																	// Set GPIO 68 as output (sys_on_3.3)
	GpioCtrlRegs.GPCDIR.bit.GPIO69 = 1;																	// Set GPIO 68 as output for speaker
	GpioCtrlRegs.GPCDIR.bit.GPIO70 = 1;																	// Set GPIO 70 as output for hand piece LED
	GpioCtrlRegs.GPCDIR.bit.GPIO71 = 0;																	// Set GPIO 71 as input for door lid
	GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;																	// Footswitch detect

	ULTRASOUND_OFF;																						// PREVENT SYS FROM TURNING ON @ INITIALIZATION
	SYS_ON_2_OFF;

	sys_watchword|=0x0001;																				// Update sys_watchword
	EDIS;


	DSP_RESET_PA_L;																						// Make sure SR latch is reset and ready to operate
	DELAY_US(1000);
	DSP_RESET_PA_H;
	DELAY_US(1000);
	DSP_RESET_PA_L;
	DELAY_US(1000);
	HP_LIGHT_OFF;																						// Ensure LED is off to start
}
//################################################################################################################################################################################################
//						End of GPIO Setup
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Setup ePWM
//################################################################################################################################################################################################

void Setup_ePWM(void)
{

	//ePWM1A Setup - Sets PWM for R1 on 4046 Chip
	//ePWM2A Setup - Sets PWM for R2 on 4046 Chip
	//ePWM2B Setup - Sets PWM for AGC Min Level
	//ePWM3A Setup - Sets PWM for Phase Command
	//ePWM4A Setup - Sets PWM for Digital Frequency
	//ePWM4B Setup - Sets PWM for Pump Control
	//ePWM5A Setup - Sets PWM for AGC Command
	//ePWM6A Setup - Sets PWM for Digital AGC
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default
	EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;																// CLKDIV = /1    ->default

	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;															// HSPCLKDIV = /1
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;															// Count up mode
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

	EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)
	EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm3Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm4Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm5Regs.TBCTL.bit.FREE_SOFT = 2;
	EPwm6Regs.TBCTL.bit.FREE_SOFT = 2;

	// Set Up AQCTLA registers
	// When CTR = CMPA on Up Count - Clear
	// When CTR = CMPA on Up Count - Set
	// When CTR = 0 - Set
	// Modifying these can alter the direction of the duty cycle
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;

	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;

	EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;

	EPwm1Regs.TBPRD = 3000;																				// 50 kHz - PWM Signal  = TBPRD = fsysclockout/(fpwm*CLKDIV*HSPCLKDIV)
	EPwm2Regs.TBPRD = 3000;																				// TBPRD = 150e6/(50e3*1*1), CLKDIV and HSPCLKDIV are the divisor numbers both 1.
	EPwm3Regs.TBPRD = 3000;																				// CTRMODE has a scaling factor of 0.5 if up/down mode is used
	EPwm4Regs.TBPRD = 3000;
	EPwm5Regs.TBPRD = 3000;
	EPwm6Regs.TBPRD = 3000;

	// Set initial values for ePWM modules
	R1_ePWM 		= R1_INIT_VAL;
	R2_ePWM 		= R2_INIT_VAL;
	AGC_Min_ePWM	= AGC_MIN_VAL;
	PHASE_CMD_ePWM 	= START_PHASE;
	DIG_FREQ_ePWM 	= LOW_VCO;
	AGC_CMD_ePWM 	= AGC_COMMAND_VAL;
	DIG_AGC_ePWM 	= AGC_START_VAL;
	PUMP_SPEED		= 0;																				// Set pump to off initially
	sys_watchword|=0x0002;
}

//################################################################################################################################################################################################
//						End Setup ePWM1A
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						Setup eCAP
//################################################################################################################################################################################################
void Setup_eCAP(void)	//EM_CODE
{
	ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CAP2POL = EC_RISING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CAP4POL = EC_RISING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;															// Do not reset counter on Capture Event 1 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;															// Do not reset counter on Capture Event 2 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;															// Do not reset counter on Capture Event 3 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;															// Do not reset counter on Capture Event 4 (absolute time stamp)
	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap1Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_ONESHT;														// Operate in one shot mode
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0x00;																// Pass through
	ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_ENABLE;															// Ensable sync in signal
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)
	ECap1Regs.ECCTL2.bit.SWSYNC = 1;																	// Synchronize down stream ECaps to ECap1



	ECap2Regs.ECCTL1.bit.CAP1POL = EC_FALLING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CAP2POL = EC_FALLING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CAP3POL = EC_FALLING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CAP4POL = EC_FALLING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap2Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;															// Do not reset counter on Capture Event 1 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;															// Do not reset counter on Capture Event 2 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;															// Do not reset counter on Capture Event 3 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;															// Do not reset counter on Capture Event 4 (absolute time stamp)
	ECap2Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap2Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap2Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap2Regs.ECCTL2.bit.CONT_ONESHT = EC_ONESHT;														// Operate in one shot mode
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0x00;
	ECap2Regs.ECCTL2.bit.SYNCI_EN = EC_ENABLE;															// Enable sync in signal
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)
	ECap2Regs.CTRPHS = 0;


	ECap4Regs.ECCTL1.bit.CAP1POL = EC_RISING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap4Regs.ECCTL1.bit.CAP2POL = EC_RISING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap4Regs.ECCTL1.bit.CAP3POL = EC_RISING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap4Regs.ECCTL1.bit.CAP4POL = EC_RISING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap4Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;															// Do not reset counter on Capture Event 1 (absolute time stamp)
	ECap4Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;															// Do not reset counter on Capture Event 2 (absolute time stamp)
	ECap4Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;															// Do not reset counter on Capture Event 3 (absolute time stamp)
	ECap4Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;															// Do not reset counter on Capture Event 4 (absolute time stamp)
	/*ECap4Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap4Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap4Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap4Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event*/
	ECap4Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap4Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap4Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap4Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;														// Capture Mode
	ECap4Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;													// Operate in continuous mode
	ECap4Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;														// Disable sync out signal
	ECap4Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;															// Disable sync in signal
	ECap4Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)
	//ECap4Regs.ECEINT.bit.CEVT2 = 1;																		// 2 Events = interrupt

	ECap5Regs.ECCTL1.bit.CAP1POL = EC_RISING;															// Capture Event 1 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CAP2POL = EC_RISING;															// Capture Event 2 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CAP3POL = EC_RISING;															// Capture Event 3 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CAP4POL = EC_RISING;															// Capture Event 4 triggered on a rising edge (RE)
	ECap5Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap5Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap5Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap5Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;														// TSCTR is reset to 0 after every event
	ECap5Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;															// Enable Loading of CAP1-4 registers on a capture event
	ECap5Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap5Regs.ECCTL1.bit.FREE_SOFT = 2;																	// Free Run (continue running when breakpoint is hit)

	ECap5Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;														// Capture Mode
	ECap5Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;													// Operate in continuous mode
	ECap5Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;														// Disable sync out signal
	ECap5Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;															// Disable sync in signal
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;															// Counter stop control (free running)

	sys_watchword|=0x0100;
}

//################################################################################################################################################################################################
//						End of Setup eCAP
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						ADC Config
//################################################################################################################################################################################################
void ADCconfig()
{
	AdcRegs.ADCTRL1.all = 0;
	AdcRegs.ADCTRL1.bit.ACQ_PS = 7;																		// Sampling sw close time = (2 / HSPCLK) * 8 - ends up = .1us
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; 																	// 0 = Dual Sequencer Mode, 1 = cascaded sequencer
	AdcRegs.ADCTRL1.bit.CPS = 0;																		// divide by 1
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0; 																	// Single Run Mode
	AdcRegs.ADCTRL2.all = 0;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0;																// Disable SEQ1 interrupt
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 6;																	// ADC clock: FCLK = HSPCLK / (2 * ADCCLKPS)
																										// HSPCLK = 150MHz (see DSP2833x_SysCtrl.c)
																										// FCLK = 12.5 MHz
	AdcRegs.ADCTRL3.bit.SMODE_SEL = 1;																	// Simultaneous Sampling Mode
	AdcRegs.ADCMAXCONV.all = 0x0005;																	// 4 conversions from Sequencer 1 (Number of conversions minus 1)
																										// Really 10 b/c ADC is in Simultaneous Sampling mode
																										// When in Dual Sequencer Mode ADCMAXCONV.all is bits 0-2 are for SEQ1
																										// bits 4-6 are for SEQ2
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;																// RESULT0 = ADCINA0 = VOLTS, 			RESULT1 = ADCINB0 = CURRENT
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;																// RESULT2 = ADCINA1 = I_MOT, 			RESULT3 = ADCINB1 = PHASE
	AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 7;																// RESULT4 = ADCINA7 = PHASE_ERROR, 	RESULT5 = ADCINB7 = VCO_IN
	AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 6;																// RESULT6 = ADCINA6 = AGC_ERROR, 		RESULT7 = ADCINB6 = AGC_PWM
	AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 2;																// RESULT8 = ADCINA2 = LEAKAGE_OUT,		RESULT9 = ADCINB2 = Nothing
	AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 3;																// RESULT10 = ADCINA3 = HANDPIECE_ID	RESULT11 = ADCINB3 = Nothing
																										// This is because ADC is in Simultaneous Sampling Mode
	sys_watchword|=0x0020;																				// Update sys_watchword
}
//################################################################################################################################################################################################
//						End ADC Config
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						SPI Init
//################################################################################################################################################################################################
void spi_init()
{
	SpiaRegs.SPICCR.all =0x000F;	             														// Reset on, rising edge, 16-bit char bits
	SpiaRegs.SPICTL.all =0x0006;    		    														// Enable master mode, normal phase,
                                                 	 	 	 	 	 	 	 	 	 	 	 	 	 	// Enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR =0x007F;
    SpiaRegs.SPICCR.all =0x008F;		         														// Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;                														// Set so breakpoints don't disturb xmission
    sys_watchword|=0x0200;																				// Update sys_watchword
}
//################################################################################################################################################################################################
//						End SPI Init
//################################################################################################################################################################################################


//################################################################################################################################################################################################
//						SPI FIFO Init
//################################################################################################################################################################################################
void spi_fifo_init()
{
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x2044;
    SpiaRegs.SPIFFCT.all=0x0;
    sys_watchword|=0x0080;																				// Update sys_watchword
}
//################################################################################################################################################################################################
//						End SPI FIFO Init
//################################################################################################################################################################################################



