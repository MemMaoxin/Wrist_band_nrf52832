/*
 * AFE4404.c
 *
 * Provides AFE4404 API
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 * ALL RIGHTS RESERVED  
 *
*/

#include "AFE4404.h"
#include "afe4404_slice.h"
#include "nrf_delay.h"
#include "slice_drv_twi.h"
#include "slice_cfg.h"


/**@brief write data to device 
 * @param addr:  i2c device address
 * @param data:  data to write to i2c device, data[0] must be the register of the device
 * @param len :  data len
 */
static void I2C_write(uint8_t addr, unsigned char *data, uint8_t len)
{
    I2C_TransferSeq_TypeDef    seq;
    uint8_t i2c_read_data[1];

    seq.addr  = addr;
    seq.flags = I2C_FLAG_WRITE;
    /* Select register and data to write */

    seq.buf[0].data = data;
    seq.buf[0].len  = len;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = 0;

    slice_drv_twi_transfer(&seq);
    
}


/**@brief read data from device 
 * @param reg :  write cmd
 * @param addr:  i2c device address
 * @param data:  data to write to i2c device, data[0] must be the register of the device
 * @param len :  data len
*/
static void I2C_read(uint8_t addr, uint8_t reg, unsigned char *data, uint8_t len)
{
    I2C_TransferSeq_TypeDef    seq;
    uint8_t i2c_write_data[3];

    seq.addr  = addr;
    seq.flags = I2C_FLAG_READ;
    /* Select register and data to write */

    i2c_write_data[0]=reg;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 1;
    seq.buf[1].data = data;
    seq.buf[1].len  = len;

    slice_drv_twi_transfer(&seq);

    return;
}


/**@brief init AFE4404 Reg 
 */
static void AFE4404_Reg_Init(void)
{
    AFE4404_Disable_Read();
    //Time duration = (end count -  start count + 1) / Frq.
    AFE4404_Reg_Write(0x31, 5);    //CLKDIV_EXTMODE  DIVISION RATIO = 1
    AFE4404_Reg_Write(0x39, 5);     //CLKDIV_PRF	 DIVISION RATIO = 4 - > 1MHZ clcok
	AFE4404_Reg_Write(0x1D, 31250-1); //AFE_PRPCOUNT  32HZ 
	
	//set time engine for each test stage, 1us per count.
	AFE4404_Reg_Write(0x09, 0);    //TE1 -    LED2 start  
    AFE4404_Reg_Write(0x0a, 399);   //  TE2 - AFE_LED2LEDENDC    LED2 end
	AFE4404_Reg_Write(0x01, 80);     // TE3 - AFE_LED2STC  Sample LED2 start 
    AFE4404_Reg_Write(0x02, 399);    // TE4 - AFE_LED2ENDC  Sample LED2 end
	AFE4404_Reg_Write(0x15, 401);   //  TE5 - AFE_ADCRSTSTCT0       ADC reset phase 0 start
    AFE4404_Reg_Write(0x16, 407);   //  TE6 - AFE_ADCRSTENDCT0       ADC reset phase 0 end
	AFE4404_Reg_Write(0x0d, 408);   //  TE7 - AFE_LED2CONVST      LED2 convert phase start
    AFE4404_Reg_Write(0x0E, 1467);  //	TE8 - AFE_LED2CONVEND     LED2 convert phase end
	
    AFE4404_Reg_Write(0x36, 400);   //  TE9 - AFE_LED3LEDSTC    LED3 start. If LED3 is not used, set these register bits to '0'.	
    AFE4404_Reg_Write(0x37, 799);   //	TE10 - AFE_LED3LEDENDC
    AFE4404_Reg_Write(0x05, 480);    // TE11 - AFE_ALED3STC     Sample ambient 2 (or sample LED3) start
    AFE4404_Reg_Write(0x06, 799);    // TE12 - AFE_ALED3ENDC     Sample ambient 2 (or sample LED3) end
	AFE4404_Reg_Write(0x17, 1469);  //  TE13 - AFE_ADCRSTSTCT1        ADC reset phase 1 start
    AFE4404_Reg_Write(0x18, 1475);  //  TE14 - AFE_ADCRSTENDCT1      ADC reset phase 1 end
	AFE4404_Reg_Write(0x0F, 1476);  //  TE15 - AFE_ALED2CONVST      Ambient 2 (or LED3) convert phase start
    AFE4404_Reg_Write(0x10, 2535);  //  TE16 - AFE_ALED2CONVEND    Ambient 2 (or LED3) convert phase end


	AFE4404_Reg_Write(0x03, 800);    // TE17 - AFE_LED1LEDSTC   LED1 start
    AFE4404_Reg_Write(0x04, 1199);   // TE18 - AFE_LED1LEDENDC    LED1 end
	AFE4404_Reg_Write(0x07, 880);    // TE19 - AFE_LED1STC        Sample LED1 start
    AFE4404_Reg_Write(0x08, 1199);   // TE20 - AFE_LED1ENDC       Sample LED1 end
	AFE4404_Reg_Write(0x19, 2537);  //  TE21 - AFE_ADCRSTSTCT2        ADC reset phase 2 start
    AFE4404_Reg_Write(0x1A, 2543);  //  TE22 - AFE_ADCRSTENDCT2       ADC reset phase 2 end
	AFE4404_Reg_Write(0x11, 2544);  //  TE23 - AFE_LED1CONVST      LED1 convert phase start
    AFE4404_Reg_Write(0x12, 3603);  //  TE24 - AFE_LED1CONVEND     LED1 convert phase end
  
    AFE4404_Reg_Write(0x0b, 1279);  //  TE25 - AFE_ALED1STC         Sample ambient 1 start
    AFE4404_Reg_Write(0x0c, 1598);  //  TE26 - AFE_ALED1ENDC        Sample ambient 1 end
	AFE4404_Reg_Write(0x1B, 3605);  //  TE27 - AFE_ADCRSTSTCT3        ADC reset phase 3 start
    AFE4404_Reg_Write(0x1C, 3611);  //  TE28 - AFE_ADCRSTENDCT3       ADC reset phase 3 end  
	AFE4404_Reg_Write(0x13, 3612);  //  TE29 - AFE_ALED1CONVST      Ambient 1 convert phase start
    AFE4404_Reg_Write(0x14, 4671);  //  TE30 - AFE_ALED1CONVEND      Ambient 1 convert phase end

    AFE4404_Reg_Write(0x32, 5471);  //  TE31 - AFE_DPD1STC  PDN_CYCLE start
    AFE4404_Reg_Write(0x33, 39199); //TE32 - AFE_DPD1ENDC  PDN_CYCLE end  -25hz
    //AFE4404_Reg_Write(0x33,( 31200-500)); //TE32 - AFE_DPD1ENDC  PDN_CYCLE end  -32hz
	
	/*Register 1E
		BIT8 TIMEREN R/W 0h 0 = Timer module disabled
												1 = Enables timer module. This bit enables the timing engine that can
														be programmed to generate all clock phases for the synchronized
														transmit drive, receive sampling, and data conversion.

		BIT3-0 NUMAV R/W 0h These bits determine the number of ADC averages. By programming a
												higher ADC conversion time, the ADC can be set to do multiple
												conversions and average these multiple conversions to achieve lower
												noise. This programmability is set with the NUMAV bit control. The
												number of samples that are averaged is represented by the decimal
												equivalent of NUMAV + 1. For example, NUMAV = 0 represents no
												averaging, NUMAV = 2 represents averaging of three samples, and
												NUMAV = 15 represents averaging of 16 samples.	
	*/
    AFE4404_Reg_Write(0x1E, BIT_8 + NUMAV_3);     
	
	/*REGISTER 20
		BIT15 ENSEPGAIN R/W 0h 0 = Single TIA gain for all phases
														1 = Enables two separate sets of TIA gains
		BIT5-3 TIA_CF_SEP R/W 0h   When ENSEPGAIN = 1, TIA_CF_SEP is the control for the Cf2 setting.
		BIT2-0 TIA_GAIN_SEP R/W 0h When ENSEPGAIN = 1, TIA_GAIN_SEP is the control for the Rf2
															 setting.	
	*/
    AFE4404_Reg_Write(0x20, BIT_15+TIA_GAIN_SEP_100K+TIA_CF_SEP_22_5P);      
	
	/*REGISTER 21
		BIT8 PROG_TG_EN W 0h This bit replaces the ADC_RDY output with a fullyprogrammable
													signal from the timing engine. The start and end
													points of this signal are set using the PROG_TG_STC and
													PROG_TG_ENDC controls.
		BIT5-3 TIA_CF R/W 0h When ENSEPGAIN = 0, these bits control the Cf setting (both
													Cf1 and Cf2); 
													0-5 pF/1-2.5 pF/2-10 pF/3-7.5 pF/4-20 pF/5-17.5 pF/6-25 pF/7-22.5 pF													
													When ENSEPGAIN = 1, these bits control the Cf1 setting.
		BIT2-0 TIA_GAIN R/W 0h When ENSEPGAIN = 0, these bits control the Rf setting (both
													Rf1 and Rf2); 0-500K/1=250K/2-100K/3 50-k/4-25K/5-10k/6-1M/7-2M
													When ENSEPGAIN = 1, these bits control the Rf1 setting.	
	*/
    AFE4404_Reg_Write(0x21, BIT_15+TIA_GAIN_SEP_100K+TIA_CF_SEP_22_5P);
		
	/*REGISTER 22  set led current
		BIT17-12 LED3 current control 0~63, 0~50mA  each step 1.6mA for lcd current range 100mA
		BIT11-6  LED2 current control
		BIT5-0   LED1 current control.	
	*/
    AFE4404_Reg_Write(0x22, 1);// (1<<6) + (1)); 1
  
  //AFE4404_Reg_Write(0x31, 5); // CLKDIV_EXTMODE = 5 (4MHz) 
	/* REGISTER 0X23
		BIT20 DYNAMIC1 R/W 0h 0 = Transmitter is not powered down
													1 = Transmitter is powered down in dynamic power-down mode
		BIT17 ILED_2X R/W 0h  0 = LED current range is 0 mA to 50 mA
													1 = LED current range is 0 mA to 100 mA
		BIT14 DYNAMIC2 R/W 0h 0 = ADC is not powered down
													1 = ADC is powered down in dynamic power-down mode
		BIT9 OSC_ENABLE R/W 0h 0 = External clock mode (default). In this mode, the CLK pin
															 functions as an input pin where the external clock can be input.
													 1 = Enables oscillator mode. In this mode, the 4-MHz internal
															 oscillator is enabled.
		BIT4 DYNAMIC3 R/W 0h  0 = TIA is not powered down
													1 = TIA is powered down in dynamic power-down mode
		BIT3 DYNAMIC4 R/W 0h  0 = Rest of ADC is not powered down
													1 = Rest of ADC is powered down in dynamic power-down mode
		BIT1 PDNRX R/W 0h     0 = Normal mode
													1 = RX portion of the AFE is powered down
		BIT0 PDNAFE R/W 0h    0 = Normal mode
													1 = Entire AFE is powered down	
	*/
	
	#ifndef INTERNAL_CLOCK
	AFE4404_Reg_Write(0x23, BIT_20+BIT_17+BIT_14+BIT_4+BIT_0); //DYN1, LEDCurr, DYN2, OSC, DYN3, DYN4 //0x000200); - 0x200 Osc mode //AFE_CONTROL2
	#else
	AFE4404_Reg_Write(0x23, BIT_20+BIT_17+BIT_14+BIT_9+BIT_4+BIT_0); //DYN1, LEDCurr, DYN2, OSC, DYN3, DYN4 //0x000200); - 0x200 Osc mode //AFE_CONTROL2
	#endif
	/* REGISTER 34h
	BIT15-0 PROG_TG_STC		These bits define the start time for the programmable timing
												engine signal that can replace ADC_RDY.
	*/
	/* REGISTER 35h
	BIT15-0 PROG_TG_ENDC 	 These bits define the end time for the programmable timing
												engine signal that can replace ADC_RDY
	*/	
	//AFE4404_Reg_Write(0x35, 0x0000ff);      //PROG_TG_ENDC  
	
	/*REGIGSTER 3Ah
			Offset cancellation DAC polarity - NEED TO STUDY 
	*/
  AFE4404_Reg_Write(0x3A, 0x000000);      //AFE_DAC_SETTING_REG   
	
	/* REGISTER 3dh
	BIT5 DEC_EN 			0 = Decimation mode disabled
									1 = Decimation mode enabled
	BIT3-1 DEC_FACTOR Decimation factor (how many samples are to be averaged)
	*/	
	
  AFE4404_Enable_Read();

  AFE4404_Reg_Read(0x01);
  
  AFE4404_Reg_Read(0x02);
  AFE4404_Reg_Read(0x03);
  
  AFE4404_Reg_Read(0x23);

}


/**@brief read data from device 
 * @param reg :  write cmd
 */
void AFE4404_Reg_Read_Data(unsigned long * buf)
{	
    buf[0] = AFE4404_Reg_Read(44);  //read PPG Data
    buf[1] = AFE4404_Reg_Read(45);  //read AMB Data
}


/**@brief init AFE4404
 */
void AFE4404_Init(void)
{
  AFE4404_RESETZ_Init ();
  AFE4404_Enable_HWPDN ();
	
  AFE4404_Disable_HWPDN ();
  AFE4404_Trigger_HWReset ();
  AFE4404_ADCRDY_Interrupt_Init();
  AFE4404_Reg_Init();
    
}


/**@brief init AFE4404 RESETZ
 */
void AFE4404_RESETZ_Init (void)
{
  SET_AFE_RESETZ_AS_OUTPUT();
   // nrf_gpio_cfg_output(AFE_RST_PIN_NO);
   // nrf_gpio_pin_set(AFE_RST_PIN_NO);
  SET_AFE_RESETZ_PIN_HIGH();
}


/**@brief enable AFE4404 HWPDN
 */
void AFE4404_Enable_HWPDN (void)
{
  SET_AFE_RESETZ_PIN_LOW();
  //__delay_cycles(160000);        // ~10ms delay with 16MHz clock
  nrf_delay_us(500); // when the RESTZ pin is pulled low for more than 200us , the device will enters hardware powerdown mode
}


/**@brief disable AFE4404 HWPDN
 */
void AFE4404_Disable_HWPDN (void)
{
  SET_AFE_RESETZ_PIN_HIGH();
  //nrf_delay_us(10*1000);//__delay_cycles(160000);        // ~10ms delay with 16MHz clock
    nrf_delay_us(200);
}


/**@brief reset AFE4404 Trigger
 */
void AFE4404_Trigger_HWReset (void)
{
  SET_AFE_RESETZ_PIN_LOW();
  //__delay_cycles(480);           // ~30us delay
  nrf_delay_us(30); // pulsing the RESETZ pin low for a duration of time between 25 to 50us
  SET_AFE_RESETZ_PIN_HIGH();
  //nrf_delay_us(10*1000);//__delay_cycles(160000);        // ~10ms delay with 16MHz clock
   nrf_delay_us(30);
}


/**@brief enable AFE4404 SWPDN
 */
void AFE4404_Enable_SWPDN(void)
{
    signed long retVal;
    retVal = AFE4404_Reg_Read(0x23);
    AFE4404_Disable_Read();
    AFE4404_Reg_Write(0x23, retVal |0x000001); //DYN1, LEDCurr, DYN2, OSC, DYN3, DYN4 //0x000200); - 0x200 Osc mode //AFE_CONTROL2
    AFE4404_Enable_Read();
}


/**@brief disable AFE4404 SWPDN
 */
void AFE4404_Disable_SWPDN(void)
{
    signed long retVal;
    retVal = AFE4404_Reg_Read(0x23);
    AFE4404_Disable_Read();
    AFE4404_Reg_Write(0x23, retVal &0xfffffe); //DYN1, LEDCurr, DYN2, OSC, DYN3, DYN4 //0x000200); - 0x200 Osc mode //AFE_CONTROL2
    AFE4404_Enable_Read();
}



/**@brief write AFE4404 reg
 */
void AFE4404_Reg_Write (unsigned char reg_address, unsigned long data)
{
  unsigned char configData[4];
  configData[0] =reg_address;
  configData[1]=(unsigned char)(data >>16);
  configData[2]=(unsigned char)(((data & 0x00FFFF) >>8));
  configData[3]=(unsigned char)(((data & 0x0000FF)));
  I2C_write(AFE4404_I2C_DEFAULT_ADDRESS, configData, 4);
}


/**@brief read AFE4404 reg
 */
signed long AFE4404_Reg_Read(unsigned char Reg_address)
{
  unsigned char configData[3];
  signed long retVal;
  I2C_read (AFE4404_I2C_DEFAULT_ADDRESS, Reg_address, configData, 3);
  retVal = configData[0];
  retVal = (retVal << 8) | configData[1];
  retVal = (retVal << 8) | configData[2];
  if (Reg_address >= 0x2A && Reg_address <= 0x2F)
  {
	if (retVal & 0x00200000) 	// check if the ADC value is positive or negative
	{
	  retVal &= 0x003FFFFF;		// convert it to a 22 bit value
	  return (retVal^0xFFC00000);
	}
  }
  return retVal;
}


/**@brief enable AFE4404 read
 */
void AFE4404_Enable_Read (void)
{
  unsigned char configData[4];
  configData[0]=CONTROL0;
  configData[1]=0x00;
  configData[2]=0x00;
  configData[3]=0x01;
  I2C_write (AFE4404_I2C_DEFAULT_ADDRESS,  configData, 4);
}


/**@brief disable AFE4404 read
 */
void AFE4404_Disable_Read (void)
{
  unsigned char configData[4];
  configData[0]=CONTROL0;
  configData[1]=0x00;
  configData[2]=0x00; 
  configData[3]=0x00;
  I2C_write (AFE4404_I2C_DEFAULT_ADDRESS,configData, 4);
}


/**@brief init AFE4404 ADCRDY Interrup
 */
void AFE4404_ADCRDY_Interrupt_Init (void)
{
  
  //SET_AFE_ADC_RDY_AS_INPUT();
  //ENABLE_MSP_INT_PU_RES_ON_ADC_RDY_PIN();
  //SET_MSP_INT_PU_RES_ON_ADC_RDY_PIN();
  //SEL_H2L_EDGE_ON_ADC_RDY_PIN();
  AFE4404_ADCRDY_Interrupt_Disable();
}


/**@brief enable  AFE4404 AFE4404 adcrdy interrupt
 */
void AFE4404_ADCRDY_Interrupt_Enable (void)
{
  //CLEAR_INT_FLAG_ON_ADC_RDY_PIN();      //P2IFG &= ~AFE_ADC_DRDY;                           	// P2.3 IFG cleared
  //ENABLE_INT_ON_ADC_RDY_PIN();          //P2IE |= AFE_ADC_DRDY;                             	// P2.3 interrupt enabled
}


/**@brief disable AFE4404 adcrdy interrupt
 */
void AFE4404_ADCRDY_Interrupt_Disable (void)
{
  //CLEAR_INT_FLAG_ON_ADC_RDY_PIN();      //P2IFG &= ~AFE_ADC_DRDY;                           	// P2.3 IFG cleared
  //DISABLE_INT_ON_ADC_RDY_PIN();         //P2IE &= ~AFE_ADC_DRDY;                             	// P2.3 interrupt disabled
}
// End of file
