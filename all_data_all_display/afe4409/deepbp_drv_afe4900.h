/*
 * DEEPBP_DRV_AFE4900.H
 *
 * Provides AFE4900 API
 *
 * Copyright (C) 2018 Intelligent Sensing Limited - https://ints.hk/ 
 * 
 * Revision 1.0 by Ningqi Luo
 * 
 * ALL RIGHTS RESERVED 
*/

#ifndef	_DEEPBP_DRV_AFE4900_H_
#define	_DEEPBP_DRV_AFE4900_H_

/*
 * Global Constants
 * Global Control
 * Control Register
 * Timer Register
 * LED Driver and Control Register
 * Global Function
*/


/*------------------------------------------------------------------------------------------------------------+
| Global Constants                                                                                            |
+------------------------------------------------------------------------------------------------------------*/

#define	AFE4900_I2C_ADDR1_5A	0x5A	/*Address of the first AFE4900*/
#define	AFE4900_I2C_ADDR2_5B	0x5B	/*Address of the second AFE4900*/

#define AFE4900_EXTERNAL_CLOCK 	0
#define AFE4900_INTERNAL_CLOCK 	1  /*internal 128-kHz oscillator*/

#define FILTER_BW_2_5kHz 0 
#define FILTER_BW_5kHz 	 1
#define FILTER_BW_10kHz  2

#define PROG_OUT1_DISABLE 0
#define PROG_OUT1_ENABLE  1

#define ECG_IAN_GAIN_2 	1
#define ECG_IAN_GAIN_3 	0
#define ECG_IAN_GAIN_4 	2
#define ECG_IAN_GAIN_5 	3
#define ECG_IAN_GAIN_6 	4
#define ECG_IAN_GAIN_9 	5
#define ECG_IAN_GAIN_12 6

#define IFS_OFFDAC_15_75uA 		0
#define IFS_OFFDAC_31_5uA  		3
#define IFS_OFFDAC_63uA    		5
#define IFS_OFFDAC_126uA   		7
#define POL_OFFDAC_POSITIVE 	0
#define POL_OFFDAC_NEGATIVIE 	1

#define ILED_FS_50mA 		0
#define ILED_FS_100mA 	1

#define TIA_GAIN_500kohm 	0
#define TIA_GAIN_250kohm 	1
#define TIA_GAIN_100kohm 	2
#define TIA_GAIN_50kohm 	3
#define TIA_GAIN_25kohm 	4
#define TIA_GAIN_10kohm 	5
#define TIA_GAIN_1Mohm 		6
#define TIA_GAIN_2Mohm 		7
#define TIA_GAIN_1_5Mohm 	8

#define TIA_CF_5pF 			0
#define TIA_CF_2_5pF 		1
#define TIA_CF_10pF 		2
#define TIA_CF_7_5pF 		3
#define TIA_CF_20pF 		4	
#define TIA_CF_17_5pF 	5
#define TIA_CF_25pF 		6
#define TIA_CF_22_5pF 	7


/*------------------------------------------------------------------------------------------------------------+
| Global Control                                                                                              |
+------------------------------------------------------------------------------------------------------------*/

#define afe4900_fifo_enable 0 		/*0 disable, 1 enable*/
#define afe4900_clock AFE4900_INTERNAL_CLOCK  	/*AFE4900_EXTERNAL_CLOCK or AFE4900_INTERNAL_CLOCK*/
#define afe4900_pd_disconnect 0 	/*0 disable, normal mode. 1 enable, disconnect INP and INM of PD*/
#define afe4900_filter_bandwidth FILTER_BW_2_5kHz  	/*FILTER_BW_2_5kHz, FILTER_BW_5kHz, FILTER_BW_10kHz*/
#define afe4900_prog_out1 PROG_OUT1_DISABLE 		/*PROG_OUT1_ENABLE or PROG_OUT1_DISABLE*/
#define afe4900_ecg_ina_gain ECG_IAN_GAIN_3 		/*ECG_IAN_GAIN_2,3,4,5,6,9,12*/


/*Offset current, direction, fullscale, amplitude*/
/*The relative accuracy of the current control 
 *from the I_OFFDAC*_LSB_EXT bit is worse than the accuracy of the other bits. 
 *Additionally, the absolute accuracy of the 
 *offset DAC current setting becomes worse at higher settings of IFS_OFFDAC.*/
/*The offset cancellation DAC is not trimmed at production; 
 *therefore, the value of the full-scale current can vary across units by ±20%.
 *POL_OFFDAC* = 0, positive current; POL_OFFDAC* = 1, negative current.*/
#define afe4900_offset_current_fullscale 	 IFS_OFFDAC_15_75uA /*IFS_OFFDAC_15_75uA, 31_5uA, 63uA, 126uA*/
#define afe4900_offset_current_polarity_led2 POL_OFFDAC_NEGATIVIE  /*POSITIVE or NAGATIVE*/ 
#define afe4900_offset_current_polarity_led3 POL_OFFDAC_NEGATIVIE
#define afe4900_offset_current_polarity_led1 POL_OFFDAC_NEGATIVIE
#define afe4900_offset_current_polarity_led4 POL_OFFDAC_NEGATIVIE
#define afe4900_offset_current_led2 0 /*range: 0~127, prefer even number and lower offset (<15uA)*/
#define afe4900_offset_current_led3 0
#define afe4900_offset_current_led1 0
#define afe4900_offset_current_led4 0


/*Driven current, fullscale, amplitude*/
#define afe4900_driven_current_fullscale ILED_FS_100mA  /*ILED_FS_50mA or ILED_FS_100mA*/
#define afe4900_driven_current_led2 30 /*2Blue*/ /*range: 0~255. 0.392mA step for 100mA FS.*/
#define afe4900_driven_current_led3 10 /*3IR*/
#define afe4900_driven_current_led1 20 /*1Green*/
#define afe4900_driven_current_led4 30 /*4Yellow*/


/*TIA GAIN and CF*/
#define afe4900_enable_sep_gain  	0 	/*0 disable, 1 enable*/
#define afe4900_enable_sep_gain4 	0 	/*0 disable, 1 enable*/

#define afe4900_tia_gain_sep 			TIA_GAIN_50kohm  
#define afe4900_tia_cf_sep 				TIA_CF_5pF

#define afe4900_tia_gain_sep2 		TIA_GAIN_50kohm
#define afe4900_tia_cf_sep2 			TIA_CF_5pF

#define afe4900_tia_gain 					TIA_GAIN_50kohm /*_10kohm, 25k, 50k, 100k, 250k, 500k, 1M, 1_5M, 2M*/
#define afe4900_tia_cf 						TIA_CF_5pF  /*2_5pF, 5pF, 7_5pF, 10pF, 17_5pF, 20pF, 22_5pF, 25pF*/

#define afe4900_tia_gain_sep3 		TIA_GAIN_50kohm
#define afe4900_tia_cf_sep3 			TIA_CF_5pF


/*------------------------------------------------------------------------------------------------------------+
| Control Register                                                                                            |
+------------------------------------------------------------------------------------------------------------*/

/* # # # Control Register Address  # # # # # # # # # # # # # # # # # # # # # */
#define CONTROL0_ADDR	0x00	/*CONTROL0*/
#define CONTROL1_ADDR	0x1E	/*CONTROL1*/
#define CONTROL2_ADDR	0x23	/*CONTROL2*/
#define CONTROL3_ADDR	0x31	/*CONTROL3*/
#define CONTROL4_ADDR	0x4B	/*CONTROL4*/
#define CONTROL5_ADDR	0x50	/*CONTROL5*/

/* # # # Constants of CONTROL0  0x00  # # # # # # # # # # # # # # # # # # # */

/*0 = FIFO disabled; 1 = FIFO enabled*/
#define FIFO_EN 		afe4900_fifo_enable 
#define FIFO_EN_POS 	6

/*Enables ultra-low power (ULP) mode; 
 *must write 1. 
 *Program this bit before programming registers with address 23h or higher.*/
#define ENABLE_ULP		1 	/*must write 1, program before 23h*/
#define ENABLE_ULP_POS	5

/*0 = Read or write only one register at a time.
 *1 = Read or write continuously in both I2C and SPI modes*/	
#define RW_COUNT		0
#define RW_COUNT_POS	4

/*Self-clearing reset bit. For a software reset, write 1.*/	
#define SW_RESET		0
#define SW_RESET_POS	3

/*This bit is used to suspend the count and keep the counter in a reset state.*/	
#define TM_COUNT_RST		0
#define TM_COUNT_RST_POS	1

/*Register readout enable for write registers in SPI mode 
 *(not needed in I2C mode or for reading ADC output registers or FIFO in SPI mode).
 *0 = Register write mode in SPI interface mode
 *1 = Enables the readout of write registers in SPI interface mode
 *Not required for readout of read-only registers with these addresses: 
 *2Ah, 2Bh, 2Ch, 2Dh, 2Eh, 2Fh, and FFh.*/	
#define SPI_REG_READ		0
#define SPI_REG_READ_POS	0	


/* # # # Constents of CONTROL1  0x1E  # # # # # # # # # # # # # # # # # # # */

/*This bit enables the timing engine that can be programmed to generate all 
 *clock phases for the synchronized transmit drive, receive sampling, and data conversion.
 *0 = Timer module disabled
 *1 = Enables timer module*/
#define TIMEREN 		1 
#define TIMEREN_POS 	8

/*These bits determine the number of ADC averages.
 *By programming a higher ADC conversion time, the ADC can be set to do 
 *multiple conversions and average these multiple conversions to achieve lower noise. 
 *This programmability is set with the NUMAV bit control. 
 *The number of samples that are averaged is 
 *represented by the decimal equivalent of NUMAV + 1. 
 *For example, NUMAV = 0 represents no averaging, 
 *NUMAV = 2 represents averaging of three samples, 
 *and NUMAV = 15 represents averaging of 16 samples.
 *When operating in PTT mode (ENABLE_PTT = 1), NUMAV must be set to a value of 1 
 *or higher if chopping mode is enabled (ENABLE_ECG_CHOP = 1).*/
#define NUMAV 		1
#define NUMAV_POS 	0


/* # # # Constants of CONTROL2  0x23  # # # # # # # # # # # # # # # # # # # */

/*Always write 1.
 *This bit is one of the bits that powers down the LED current bias 
 *when the device is in deep sleep phase.*/
#define CONTROL_DYN_TX_0 		1 	/*Always write 1*/
#define CONTROL_DYN_TX_0_POS	20
	
/*Powers down the bias for the offset cancellation DAC and ADC when the device is in deep sleep phase. 
 *Always write 1.*/
#define CONTROL_DYN_BIAS 		1 	/*Always write 1*/
#define CONTROL_DYN_BIAS_POS	14

/*0 = External clock mode (default). 
 *In this mode, the CLK pin functions as an input pin where the external clock can be input. 
 *1 = Enables oscillator mode. 
 *In this mode, the 128-kHz internal oscillator is enabled and used by the timing engine.*/
#define OSC_ENABLE 			afe4900_clock
#define OSC_ENABLE_POS		9

/*Always write 1.
 *Powers down the TIA when the device comes out of active mode.*/
#define CONTROL_DYN_TIA 		1 	/*Always write 1*/
#define CONTROL_DYN_TIA_POS		4

/*Always write 1.
 *Powers down the ADC when the device comes out of active mode.*/
#define CONTROL_DYN_ADC 		1 	/*Always write 1*/
#define CONTROL_DYN_ADC_POS		3

/*0 = Normal mode
1 = RX portion of the AFE is powered down*/
#define PDNRX 		0
#define PDNRX_POS	1

/*0 = Normal mode
1 = Entire AFE is powered down*/
#define PDNAFE 		0
#define PDNAFE_POS	0


/* # # # Constants of CONTROL3  0x31  # # # # # # # # # # # # # # # # # # # */

/*Bit D1 that controls the bandwidth setting of the noise-reduction filter.*/
#define FILTER_BW_0 		(afe4900_filter_bandwidth & 1)
#define FILTER_BW_0_POS 	9  	
#define FILTER_BW_1 		((afe4900_filter_bandwidth & 2)>>1)
#define FILTER_BW_1_POS 	23

/*This bit disconnects the PD signals (INP, INM) from the TIA inputs. 
 *When enabled, the current input to the TIA is determined completely by 
 *the offset cancellation DAC current (I_OFFDAC).
 *In this mode, the AFE no longer sets the bias for the PD.*/
#define PD_DISCONNECT 		afe4900_pd_disconnect
#define PD_DISCONNECT_POS 	10

/*INP, INN are shorted to VCM whenever the TIA is in power-down.*/
#define ENABLE_INPUT_SHORT 		1
#define ENABLE_INPUT_SHORT_POS 	5


/* # # # Constants of CONTROL4  0x4B  # # # # # # # # # # # # # # # # # # # */

/*This bit enables PROG_OUT1 to become an output pin where interrupts can be made to come out on.*/
#define EN_PROG_OUT1 		afe4900_prog_out1
#define EN_PROG_OUT1_POS 	8

/*Powers down the VCM buffer used to set the dc bias at the input pins and inside the ADC 
 *when the device is in deep sleep phase. Always write to 1.*/
#define CONTROL_DYN_VCM 		1 /*Always write to 1*/
#define CONTROL_DYN_VCM_POS 	3

/*When operating with the internal LDOs enabled, 
 *this bit puts the digital LDO in a low power state when the device is in deep sleep phase.*/
#define CONTROL_DYN_DLDO 		0
#define CONTROL_DYN_DLDO_POS 	2

/*When operating with the internal LDOs enabled, 
 *this bit powers down the analog LDO (and tri-states the output) when the device is in deep sleep phase. 
 *If this bit is set to 1, then short the output of the analog LDO in deep sleep phase 
 *to the output of the digital LDO using the SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP register control.*/
#define CONTROL_DYN_ALDO 		0
#define CONTROL_DYN_ALDO_POS 	1

/*Powers down the band-gap and reference circuits when the device is in deep sleep phase. 
 *Always write to 1.*/
#define CONTROL_DYN_BG 			1 	/*Always write to 1*/
#define CONTROL_DYN_BG_POS 		0


/* # # # Constants of CONTROL5  0x50  # # # # # # # # # # # # # # # # # # # */

/*Control for shorting the ECG input pins to RLD_OUT 
 *00 = INP_ECG, INM_ECG not shorted to RLD_OUT 
 *11 = INP_ECG, INM_ECG internally shorted to RLD_OUT*/
#define SHORT_ECG_TO_RLD 		0
#define SHORT_ECG_TO_RLD_POS 	22

/*Sets the gain of the INA in the ECG signal chain*/
#define GAIN_ECG 		afe4900_ecg_ina_gain 
#define GAIN_ECG_POS 	18

/*Shorts the output of the analog LDO to the output of the digital LDO when in deep sleep mode.*/
#define SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP 		0 
#define SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP_POS 	5

/*Short PD2 input pins to VCM (internal common-mode voltage) 
 *when PD2 is disconnected from the active TIA in dual PD mode. 
 *Similarly shorts PD2 and PD3 input pins to VCM when in triple PD mode.*/
#define ENABLE_PD2_SHORT 		1
#define ENABLE_PD2_SHORT_POS 	4

/*Second bit that powers down LED current bias when the device is in deep sleep mode. 
 *Always write 1.*/
#define CONTROL_DYN_TX_1 		1 /*Always write 1.*/
#define CONTROL_DYN_TX_1_POS 	3


/* # # # Control Register Value    # # # # # # # # # # # # # # # # # # # # # */
#define CONTROL0_VAL	(0x000000ul 									|	\
						 FIFO_EN << FIFO_EN_POS 						| 	\
						 ENABLE_ULP << ENABLE_ULP_POS 					|	\
						 RW_COUNT << RW_COUNT_POS 						| 	\
						 SW_RESET << SW_RESET_POS 						|	\
						 TM_COUNT_RST << TM_COUNT_RST_POS 				|	\
						 SPI_REG_READ << SPI_REG_READ_POS)

#define CONTROL1_VAL	(0x000000ul 									|	\
						 TIMEREN << TIMEREN_POS 						|	\
						 NUMAV << NUMAV_POS)

#define CONTROL2_VAL	(0x000000ul 									|	\
						 CONTROL_DYN_TX_0 << CONTROL_DYN_TX_0_POS 		| 	\
						 ILED_FS << ILED_FS_POS 						|	\
						 ENSEPGAIN4 << ENSEPGAIN4_POS 					|	\
						 CONTROL_DYN_BIAS << CONTROL_DYN_BIAS_POS 		|	\
						 OSC_ENABLE << OSC_ENABLE_POS 					|	\
						 CONTROL_DYN_TIA << CONTROL_DYN_TIA_POS 		|	\
						 CONTROL_DYN_ADC << CONTROL_DYN_ADC_POS 		|	\
						 PDNRX << PDNRX_POS 							|	\
						 PDNAFE << PDNAFE_POS)

#define CONTROL3_VAL	(0x000000ul 									|	\
						 FILTER_BW_1 << FILTER_BW_1_POS 				| 	\
						 PD_DISCONNECT << PD_DISCONNECT_POS 			|	\
						 ENABLE_INPUT_SHORT << ENABLE_INPUT_SHORT_POS)

#define CONTROL4_VAL	(0x000000ul 									|	\
						 EN_PROG_OUT1 << EN_PROG_OUT1_POS 				| 	\
						 CONTROL_DYN_VCM << CONTROL_DYN_VCM_POS 		|	\
						 CONTROL_DYN_DLDO << CONTROL_DYN_DLDO_POS		|	\
						 CONTROL_DYN_ALDO << CONTROL_DYN_ALDO_POS 		|	\
						 CONTROL_DYN_BG << CONTROL_DYN_BG_POS)

#define CONTROL5_VAL	(0x000000ul 									|	\
						 SHORT_ECG_TO_RLD << SHORT_ECG_TO_RLD_POS 		| 	\
						 GAIN_ECG << GAIN_ECG_POS 						|	\
						 SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP << SHORT_ALDO_TO_DLDO_IN_DEEP_SLEEP_POS	|	\
						 ENABLE_PD2_SHORT << ENABLE_PD2_SHORT_POS 		|	\
						 CONTROL_DYN_TX_1 << CONTROL_DYN_TX_1_POS)


/*------------------------------------------------------------------------------------------------------------+
| Timer Register                                                                                              |
+------------------------------------------------------------------------------------------------------------*/

#define LED2STC_ADDR				0x01	/*LED2STC*/
#define LED2ENDC_ADDR				0x02	/*LED2ENDC*/
#define LED1LEDSTC_ADDR				0x03	/*LED1LEDSTC*/
#define LED1LEDENDC_ADDR			0x04	/*LED1LEDENDC*/
#define LED3STC_ADDR				0x05	/*ALED2STC*/
#define LED3ENDC_ADDR				0x06	/*ALED2ENDC*/
#define LED1STC_ADDR				0x07	/*LED1STC*/
#define LED1ENDC_ADDR				0x08	/*LED1ENDC*/
#define LED2LEDSTC_ADDR				0x09	/*LED2LEDSTC*/
#define LED2LEDENDC_ADDR			0x0A	/*LED2LEDENDC*/
#define LED4STC_ADDR				0x0B	/*ALED1STC*/
#define LED4ENDC_ADDR				0x0C	/*ALED1ENDC*/
#define LED2CONVST_ADDR				0x0D	/*LED2CONVST*/
#define LED2CONVEND_ADDR			0x0E	/*LED2CONVEND*/
#define LED3CONVST_ADDR				0x0F	/*ALED2CONVST*/
#define LED3CONVEND_ADDR			0x10	/*ALED2CONVEND*/
#define LED1CONVST_ADDR				0x11	/*LED1CONVST*/
#define LED1CONVEND_ADDR			0x12	/*LED1CONVEND*/
#define LED4CONVST_ADDR				0x13	/*ALED1CONVST*/
#define LED4CONVEND_ADDR			0x14	/*ALED1CONVEND*/

#define PRPCOUNT_ADDR				0x1D	/*PRPCOUNT*/

#define LED3LEDSTC_ADDR				0x36	/*LED3LEDSTC*/
#define LED3LEDENDC_ADDR			0x37	/*LED3LEDENDC*/
#define LED4LEDSTC_ADDR				0x43	/*LED4LEDSTC*/
#define LED4LEDENDC_ADDR			0x44	/*LED4LEDENDC*/

#define DATA_RDY_STC_ADDR			0x52	/*DATA_RDY_STC*/
#define DATA_RDY_ENDC_ADDR			0x53	/*DATA_RDY_ENDC*/

#define DYN_TIA_STC_ADDR			0x64	/*DYN_TIA_STC*/
#define DYN_TIA_ENDC_ADDR			0x65	/*DYN_TIA_ENDC*/

#define DYN_ADC_STC_ADDR			0x66	/*DYN_ADC_STC*/
#define DYN_ADC_ENDC_ADDR			0x67	/*DYN_ADC_ENDC*/

#define DYN_CLOCK_STC_ADDR			0x68	/*DYN_CLOCK_STC*/
#define DYN_CLOCK_ENDC_ADDR			0x69	/*DYN_CLOCK_ENDC*/

#define DEEP_SLEEP_STC_ADDR			0x6A	/*DEEP_SLEEP_STC*/
#define DEEP_SLEEP_ENDC_ADDR		0x6B	/*DEEP_SLEEP_ENDC*/



/*------------------------------------------------------------------------------------------------------------+
| LED Driver Register                                                                                         |
+------------------------------------------------------------------------------------------------------------*/

/* # # # LED Driver Register Address # # # # # # # # # # # # # # # # # # # # */
#define TIAGAIN_2_3_ADDR	0x1F	/*TIAGAIN_2_3*/
#define TIAGAIN_ADDR		0x20	/*TIAGAIN*/
#define TIA_AMB_GAIN_ADDR	0x21	/*TIA_AMB_GAIN*/
#define LEDCNTRL1_ADDR		0x22	/*LEDCNTRL1*/
#define LEDCNTRL2_ADDR		0x24	/*LEDCNTRL2*/
#define OFFDAC_ADDR			0x3A	/*OFFDAC*/
#define I_OFFDAC_ADDR		0x3E	/*I_OFFDAC*/

/*The relative accuracy of the current control 
 *from the I_OFFDAC*_LSB_EXT bit is worse than the accuracy of the other bits. 
 *Additionally, the absolute accuracy of the 
 *offset DAC current setting becomes worse at higher settings of IFS_OFFDAC.*/
/*The offset cancellation DAC is not trimmed at production; 
 *therefore, the value of the full-scale current can vary across units by ±20%.
 *POL_OFFDAC* = 0, positive current; POL_OFFDAC* = 1, negative current.*/
#define IFS_OFFDAC 					afe4900_offset_current_fullscale
#define IFS_OFFDAC_POS 				12

#define EARLY_OFFSET_DAC 			1
#define EARLY_OFFSET_DAC_POS 		20	

#define POL_OFFDAC_LED2 			afe4900_offset_current_polarity_led2
#define POL_OFFDAC_LED2_POS 		19
#define I_OFFDAC_LED2_MSB 			((afe4900_offset_current_led2 & 64) >> 6) /*ob1000000=64*/
#define I_OFFDAC_LED2_MSB_POS 		7
#define I_OFFDAC_LED2_MID 			((afe4900_offset_current_led2 & 60) >> 2) /*0b0111100=60*/
#define I_OFFDAC_LED2_MID_POS 		15
#define I_OFFDAC_LED2_LSB 			((afe4900_offset_current_led2 & 2) >> 1) /*0b0000010=2*/
#define I_OFFDAC_LED2_LSB_POS 		6
#define I_OFFDAC_LED2_LSB_EXT 		( afe4900_offset_current_led2 & 1 ) /*0b0000001=1*/
#define I_OFFDAC_LED2_LSB_EXT_POS 	11

#define POL_OFFDAC_LED3 			afe4900_offset_current_polarity_led3
#define POL_OFFDAC_LED3_POS 		4
#define I_OFFDAC_LED3_MSB 			((afe4900_offset_current_led3 & 64) >> 6)
#define I_OFFDAC_LED3_MSB_POS 		1
#define I_OFFDAC_LED3_MID 			((afe4900_offset_current_led3 & 60) >> 2)
#define I_OFFDAC_LED3_MID_POS 		0
#define I_OFFDAC_LED3_LSB 			((afe4900_offset_current_led3 & 2) >> 1)
#define I_OFFDAC_LED3_LSB_POS  		0
#define I_OFFDAC_LED3_LSB_EXT 		( afe4900_offset_current_led3 & 1 )
#define I_OFFDAC_LED3_LSB_EXT_POS  	8

#define POL_OFFDAC_LED1 			afe4900_offset_current_polarity_led1
#define POL_OFFDAC_LED1_POS 		9
#define I_OFFDAC_LED1_MSB    		((afe4900_offset_current_led1 & 64) >> 6)
#define I_OFFDAC_LED1_MSB_POS 		3
#define I_OFFDAC_LED1_MID 			((afe4900_offset_current_led1 & 60) >> 2)
#define I_OFFDAC_LED1_MID_POS 		5
#define I_OFFDAC_LED1_LSB 			((afe4900_offset_current_led1 & 2) >> 1)
#define I_OFFDAC_LED1_LSB_POS 		2
#define I_OFFDAC_LED1_LSB_EXT 		( afe4900_offset_current_led1 & 1 )
#define I_OFFDAC_LED1_LSB_EXT_POS 	9

#define POL_OFFDAC_LED4 			afe4900_offset_current_polarity_led4
#define POL_OFFDAC_LED4_POS 		14
#define I_OFFDAC_LED4_MSB 			((afe4900_offset_current_led4 & 64) >> 6)
#define I_OFFDAC_LED4_MSB_POS 		5
#define I_OFFDAC_LED4_MID 			((afe4900_offset_current_led4 & 60) >> 2)
#define I_OFFDAC_LED4_MID_POS 		10
#define I_OFFDAC_LED4_LSB 			((afe4900_offset_current_led4 & 2) >> 1)
#define I_OFFDAC_LED4_LSB_POS 		4
#define I_OFFDAC_LED4_LSB_EXT 		( afe4900_offset_current_led4 & 1 )
#define I_OFFDAC_LED4_LSB_EXT_POS 	10


/*Programs the full-scale current range of the LED driver.*/
#define ILED_FS 		afe4900_driven_current_fullscale 
#define ILED_FS_POS		17

#define ILED2_MSB 		((afe4900_driven_current_led2 & 252) >> 2) /*0b11111100=252*/
#define ILED2_MSB_POS 	6
#define ILED2_LSB 		( afe4900_driven_current_led2 & 3) /*0b00000011=3*/
#define ILED2_LSB_POS 	20

#define ILED3_MSB 		((afe4900_driven_current_led3 & 252) >> 2)
#define ILED3_MSB_POS 	12
#define ILED3_LSB 		( afe4900_driven_current_led3 & 3)
#define ILED3_LSB_POS 	22

#define ILED1_MSB 		((afe4900_driven_current_led1 & 252) >> 2)
#define ILED1_MSB_POS 	0
#define ILED1_LSB 		( afe4900_driven_current_led1 & 3)
#define ILED1_LSB_POS 	18

#define ILED4_MSB 		((afe4900_driven_current_led4 & 252) >> 2)
#define ILED4_MSB_POS 	11
#define ILED4_LSB 		( afe4900_driven_current_led4 & 3)
#define ILED4_LSB_POS 	9


#define ENSEPGAIN 				afe4900_enable_sep_gain
#define ENSEPGAIN_POS 			15
/*Mode to enable independently programmable RF and CF values in each of the four phases.*/
#define ENSEPGAIN4 				afe4900_enable_sep_gain4
#define ENSEPGAIN4_POS			15

#define TIA_GAIN_MSB  			((afe4900_tia_gain & 8)>>3) /*0b1000=8*/
#define TIA_GAIN_MSB_POS 		6
#define TIA_GAIN_LSB 			(afe4900_tia_gain & 7) /*0b0111=7*/
#define TIA_GAIN_LSB_POS 		0
#define TIA_CF 					afe4900_tia_cf
#define TIA_CF_POS 				3

#define TIA_GAIN_SEP_MSB 		((afe4900_tia_gain_sep & 8)>>3)
#define TIA_GAIN_SEP_MSB_POS 	6
#define TIA_GAIN_SEP_LSB 		(afe4900_tia_gain_sep & 8)
#define TIA_GAIN_SEP_LSB_POS 	0
#define TIA_CF_SEP 				afe4900_tia_cf_sep
#define TIA_CF_SEP_POS 			3

#define TIA_GAIN_SEP2_MSB 		((afe4900_tia_gain_sep2 & 8)>>3)
#define TIA_GAIN_SEP2_MSB_POS 	6
#define TIA_GAIN_SEP2_LSB 		(afe4900_tia_gain_sep2 & 7)
#define TIA_GAIN_SEP2_LSB_POS 	0
#define TIA_CF_SEP2 			afe4900_tia_cf_sep2
#define TIA_CF_SEP2_POS 		3

#define TIA_GAIN_SEP3_MSB 		((afe4900_tia_gain_sep3 & 8)>>3)
#define TIA_GAIN_SEP3_MSB_POS	14
#define TIA_GAIN_SEP3_LSB  		(afe4900_tia_gain_sep3 & 7)
#define TIA_GAIN_SEP3_LSB_POS 	8
#define TIA_CF_SEP3 			afe4900_tia_cf_sep3
#define TIA_CF_SEP3_POS			11


/* # # # LED Driver Register Value   # # # # # # # # # # # # # # # # # # # # */
#define TIAGAIN_2_3_VAL 	(0x000000ul 									|	\
							 TIA_GAIN_SEP3_MSB << TIA_GAIN_SEP3_MSB_POS 	|	\
							 TIA_CF_SEP3 << TIA_CF_SEP3_POS 				|	\
							 TIA_GAIN_SEP3_LSB << TIA_GAIN_SEP3_LSB_POS 	|	\
							 TIA_GAIN_SEP2_MSB << TIA_GAIN_SEP2_MSB_POS 	|	\
							 TIA_CF_SEP2 << TIA_CF_SEP2_POS 				|	\
							 TIA_GAIN_SEP2_LSB << TIA_GAIN_SEP2_LSB_POS)

#define TIAGAIN_VAL 		(0x000000ul 								|	\
							 ENSEPGAIN << ENSEPGAIN_POS 				|	\
							 TIA_GAIN_SEP_MSB << TIA_GAIN_SEP_MSB_POS 	|	\
							 TIA_CF_SEP << TIA_CF_SEP_POS 				|	\
							 TIA_GAIN_SEP_LSB << TIA_GAIN_SEP_LSB_POS)

#define TIA_AMB_GAIN_VAL	(0x000000ul 						|	\
							 IFS_OFFDAC << IFS_OFFDAC_POS 		|	\
							 FILTER_BW_0 << FILTER_BW_0_POS 	|	\
							 TIA_GAIN_MSB << TIA_GAIN_MSB_POS 	|	\
							 TIA_CF << TIA_CF_POS 				|	\
							 TIA_GAIN_LSB << TIA_GAIN_LSB_POS)

#define LEDCNTRL1_VAL 		(0x000000ul 					|	\
							 ILED3_LSB << ILED3_LSB_POS 	|	\
							 ILED2_LSB << ILED2_LSB_POS 	|	\
							 ILED1_LSB << ILED1_LSB_POS 	|	\
							 ILED3_MSB << ILED3_MSB_POS 	|	\
							 ILED2_MSB << ILED2_MSB_POS 	|	\
							 ILED1_MSB << ILED1_MSB_POS)

#define LEDCNTRL2_VAL 		(0x000000ul 					|	\
							 ILED4_MSB << ILED4_MSB_POS 	|	\
							 ILED4_LSB << ILED4_LSB_POS)

#define OFFDAC_VAL 			(0x000000ul 								|	\
							 EARLY_OFFSET_DAC << EARLY_OFFSET_DAC_POS 	|	\
							 POL_OFFDAC_LED2 << POL_OFFDAC_LED2_POS 	|	\
							 I_OFFDAC_LED2_MID << I_OFFDAC_LED2_MID_POS |	\
							 POL_OFFDAC_LED4 << POL_OFFDAC_LED4_POS 	|	\
							 I_OFFDAC_LED4_MID << I_OFFDAC_LED4_MID_POS |	\
							 POL_OFFDAC_LED1 << POL_OFFDAC_LED1_POS 	|	\
							 I_OFFDAC_LED1_MID << I_OFFDAC_LED1_MID_POS |	\
							 POL_OFFDAC_LED3 << POL_OFFDAC_LED3_POS 	|	\
							 I_OFFDAC_LED3_MID << I_OFFDAC_LED3_MID_POS)	

#define I_OFFDAC_VAL 		(0x000000ul 										|	\
							 I_OFFDAC_LED2_LSB_EXT << I_OFFDAC_LED2_LSB_EXT_POS |	\
							 I_OFFDAC_LED4_LSB_EXT << I_OFFDAC_LED4_LSB_EXT_POS |	\
							 I_OFFDAC_LED1_LSB_EXT << I_OFFDAC_LED1_LSB_EXT_POS |	\
							 I_OFFDAC_LED3_LSB_EXT << I_OFFDAC_LED3_LSB_EXT_POS |	\
							 I_OFFDAC_LED2_MSB << I_OFFDAC_LED2_MSB_POS 		|	\
							 I_OFFDAC_LED2_LSB << I_OFFDAC_LED2_LSB_POS 		|	\
							 I_OFFDAC_LED4_MSB << I_OFFDAC_LED4_MSB_POS 		|	\
							 I_OFFDAC_LED4_LSB << I_OFFDAC_LED4_LSB_POS 		|	\
							 I_OFFDAC_LED1_MSB << I_OFFDAC_LED1_MSB_POS 		|	\
							 I_OFFDAC_LED1_LSB << I_OFFDAC_LED1_LSB_POS 		|	\
							 I_OFFDAC_LED3_MSB << I_OFFDAC_LED3_MSB_POS 		|	\
							 I_OFFDAC_LED3_LSB << I_OFFDAC_LED3_LSB_POS)




/*------------------------------------------------------------------------------------------------------------+
| Other Register                                                                                              |
+------------------------------------------------------------------------------------------------------------*/


#define TOGGLE_ADDR					0x28	/*TOGGLE*/
#define CLKDIV1_ADDR				0x29	/*CLKDIV1*/
#define LED2VAL_ADDR				0x2A	/*LED2VAL*/
#define ALED2VAL_ADDR				0x2B	/*ALED2VAL*/
#define LED1VAL_ADDR				0x2C	/*LED1VAL*/
#define ALED1VAL_ADDR				0x2D	/*ALED1VAL*/
#define LED2_ALED2VAL_ADDR			0x2E	/*LED2-ALED2VAL*/
#define LED1_ALED1VAL_ADDR			0x2F	/*LED1-ALED1VAL*/

#define PROG_INT2_STC_ADDR			0x34	/*PROG_INT2_STC*/
#define PROG_INT2_ENDC_ADDR			0x35	/*PROG_INT2_ENDC*/

#define CLKDIV2_ADDR				0x39	/*CLKDIV2*/




#define THRDETLOW_ADDR				0x3B	/*THRDETLOW*/
#define THRDETHIGH_ADDR				0x3C	/*THRDETHIGH*/
#define THRDET_ADDR					0x3D	/*THRDET*/

#define AVG_LED2_ALED2VAL_ADDR		0x3F	 /*AVG_LED2_ALED2VAL*/
#define AVG_LED1_ALED1VAL_ADDR		0x40	/*AVG_LED1_ALED1VAL*/
#define FIFO_ADDR					0x42	/*FIFO*/

#define TG_PD1STC_ADDR				0x45	/*TG_PD1STC*/
#define TG_PD1ENDC_ADDR				0x46	/*TG_PD1ENDC*/
#define TG_PD2STC_ADDR				0x47	/*TG_PD2STC*/
#define TG_PD2ENDC_ADDR				0x48	/*TG_PD2ENDC*/
#define TG_PD3STC_ADDR				0x49	/*TG_PD3STC*/
#define TG_PD3ENDC_ADDR				0x4A	/*TG_PD3ENDC*/

#define DUAL_PD_ADDR				0x4E	/*DUAL_PD*/

#define FIFO_OFFSET_ADDR			0x51	/*FIFO_OFFSET*/

#define MASK_PPG_ADDR				0x54	/*MASK_PPG*/
#define PROG_INT1_STC_ADDR			0x57	/*PROG_INT1_STC*/
#define PROG_INT1_ENDC_ADDR			0x58	/*PROG_INT1_ENDC*/
#define ECG_CHOP_ADDR				0x61	/*ECG_CHOP*/
#define ECG_RLD_ADDR				0x62	/*ECG_RLD*/
#define RCOMP_ADDR					0x63	/*RCOMP*/

#define PD_SHORT_ADDR				0x6C	/*PD_SHORT*/
#define REG_POINTER_ADDR			0x6D	/*REG_POINTER*/
#define LED_DRIVER_CONTROL_ADDR		0x72	/*LED_DRIVER_CONTROL*/
#define THR_DETECT_LOGIC_ADDR		0x73	/*THR_DETECT_LOGIC*/


/************************************************************************************************************/
/* Global Functions																							*/
/************************************************************************************************************/


#endif	/*end define deepbp_drv_afe4900.h*/
