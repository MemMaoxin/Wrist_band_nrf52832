/*
 * AFE4409.c
 *
 * 125Hz sampling rate
 * Set sampling rate at line 325: AFE4409_set_dryFreq(0x000200,0x0001F9);
 *
 * Provides AFE4409 API
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 * ALL RIGHTS RESERVED  
 *
*/
#include "app_twi.h"
#include "nrf_delay.h"
//#include "slice_drv_twi.h"
#include "slice_cfg.h"
#include "afe4409.h"
#include "app_uart.h"
#include "deepbp_drv_afe4900.h"
#include "nrf_drv_twi.h"
//#include "slice_drv_afe4409.h"
#include "twi.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

unsigned int egc_reg_init[MAX_REG_NUMBER][2] = {
{0x00, 0x000020}, /*CONTROL0*/
{0x1E, 0x000101}, /*CONTROL1*/
{0x23, 0x124218}, /*CONTROL2*/
{0x31, 0x000020}, /*CONTROL3*/
{0x4B, 0x000009}, /*CONTROL4*/
{0x50, 0x100018}, /*CONTROL5*/
/****************************************************/
{0x01, 0x00000B}, /*LED2STC*/
{0x02, 0x00000D}, /*LED2ENDC*/
{0x03, 0x000014}, /*LED1LEDSTC*/
{0x04, 0x000017}, /*LED1LEDENDC*/
{0x05, 0x000010}, /*ALED2STC*/
{0x06, 0x000012}, /*ALED2ENDC*/
{0x07, 0x000015}, /*LED1STC*/
{0x08, 0x000017}, /*LED1ENDC*/
{0x09, 0x00000A}, /*LED2LEDSTC*/
{0x0A, 0x00000D}, /*LED2LEDENDC*/
{0x0B, 0x00001A}, /*ALED1STC*/
{0x0C, 0x00001C}, /*ALED1ENDC*/
{0x0D, 0x00000F}, /*LED2CONVST*/
{0x0E, 0x000012}, /*LED2CONVEND*/
{0x0F, 0x000014}, /*ALED2CONVST*/
{0x10, 0x000017}, /*ALED2CONVEND*/
{0x11, 0x000019}, /*LED1CONVST*/
{0x12, 0x00001C}, /*LED1CONVEND*/
{0x13, 0x00001E}, /*ALED1CONVST*/
{0x14, 0x000021}, /*ALED1CONVEND*/
{0x1D, 0x000200}, /*PRPCOUNT*/
{0x1F, 0x000000}, /*TIAGAIN_2_3*/
{0x20, 0x000003}, /*TIAGAIN*/
{0x21, 0x000003}, /*TIA_AMB_GAIN*/
{0x22, 0x0030C3}, /*LEDCNTRL1*/
{0x24, 0x000000}, /*LEDCNTRL2*/
{0x28, 0x000000}, /*TOGGLE*/
{0x29, 0x000000}, /*CLKDIV1*/
{0x2A, 0x000000}, /*LED2VAL*/
{0x2B, 0x000000}, /*ALED2VAL*/
{0x2C, 0x000000}, /*LED1VAL*/
{0x2D, 0x000000}, /*ALED1VAL*/
{0x2E, 0x000000}, /*LED2-ALED2VAL*/
{0x2F, 0x000000}, /*LED1-ALED1VAL*/
{0x34, 0x000000}, /*PROG_INT2_STC*/
{0x35, 0x000000}, /*PROG_INT2_ENDC*/
{0x36, 0x00000F}, /*LED3LEDSTC*/
{0x37, 0x000012}, /*LED3LEDENDC*/
{0x39, 0x000004}, /*CLKDIV2*/
{0x3A, 0x100000}, /*OFFDAC*/
{0x3B, 0x000000}, /*THRDETLOW*/
{0x3C, 0x000000}, /*THRDETHIGH*/
{0x3D, 0x000000}, /*THRDET*/
{0x3E, 0x000000}, /*I_OFFDAC*/
{0x3F, 0x000000}, /*AVG_LED2_ALED2VAL*/
{0x40, 0x000000}, /*AVG_LED1_ALED1VAL*/
{0x42, 0x000000}, /*FIFO*/
{0x43, 0x000019}, /*LED4LEDSTC*/
{0x44, 0x00001C}, /*LED4LEDENDC*/
{0x45, 0x000000}, /*TG_PD1STC*/
{0x46, 0x000000}, /*TG_PD1ENDC*/
{0x47, 0x000000}, /*TG_PD2STC*/
{0x48, 0x000000}, /*TG_PD2ENDC*/
{0x49, 0x000000}, /*TG_PD3STC*/
{0x4A, 0x000000}, /*TG_PD3ENDC*/
{0x4E, 0x360004}, /*DUAL_PD*/
{0x51, 0x000000}, /*FIFO_OFFSET*/
{0x52, 0x00002B}, /*DATA_RDY_STC*/
{0x53, 0x00002B}, /*DATA_RDY_ENDC*/
{0x54, 0x000000}, /*MASK_PPG*/
{0x57, 0x000000}, /*PROG_INT1_STC*/
{0x58, 0x000000}, /*PROG_INT1_ENDC*/
{0x61, 0x080000}, /*ECG_CHOP*/
{0x62, 0x800000}, /*ECG_RLD*/
{0x63, 0x000000}, /*RCOMP*/
{0x64, 0x000000}, /*DYN_TIA_STC*/
{0x65, 0x000023}, /*DYN_TIA_ENDC*/
{0x66, 0x000000}, /*DYN_ADC_STC*/
{0x67, 0x000023}, /*DYN_ADC_ENDC*/
{0x68, 0x000000}, /*DYN_CLOCK_STC*/
{0x69, 0x000023}, /*DYN_CLOCK_ENDC*/
{0x6A, 0x00002E}, /*DEEP_SLEEP_STC*/
{0x6B, 0x0001F9}, /*DEEP_SLEEP_ENDC*/
{0x6C, 0x000000}, /*PD_SHORT*/
{0x6D, 0x000000}, /*REG_POINTER*/
{0x72, 0x00000F}, /*LED_DRIVER_CONTROL*/
{0x73, 0x000000}, /*THR_DETECT_LOGIC*/
};

unsigned int reg_init[MAX_REG_NUMBER][2] = {		
//{0x00, 0x000020}, /*CONTROL0*/
//{0x1E, 0x000101}, /*CONTROL1*/
//{0x23, 0x12C218}, /*CONTROL2*/
//{0x31, 0x000020}, /*CONTROL3*/
//{0x4B, 0x000009}, /*CONTROL4*/
//{0x50, 0x000018}, /*CONTROL5*/
{CONTROL0_ADDR, CONTROL0_VAL}, /*CONTROL0*/
{CONTROL1_ADDR, CONTROL1_VAL}, /*CONTROL1*/
{CONTROL2_ADDR, CONTROL2_VAL}, /*CONTROL2*/
{CONTROL3_ADDR, CONTROL3_VAL}, /*CONTROL3*/
{CONTROL4_ADDR, CONTROL4_VAL}, /*CONTROL4*/
{CONTROL5_ADDR, CONTROL5_VAL}, /*CONTROL5*/
/****************************************************/
//{0x1F, 0x000103}, /*TIAGAIN_2_3*/
//{0x20, 0x008001}, /*TIAGAIN*/
//{0x21, 0x000001}, /*TIA_AMB_GAIN*/
//{0x22, 0xFC3203}, /*LEDCNTRL1*/
//{0x24, 0x004600}, /*LEDCNTRL2*/
//{0x3A, 0x184210}, /*OFFDAC*/
//{0x3E, 0x000000}, /*I_OFFDAC*/
{TIAGAIN_2_3_ADDR, 	TIAGAIN_2_3_VAL}, /*TIAGAIN_2_3*/
{TIAGAIN_ADDR, 			TIAGAIN_VAL}, /*TIAGAIN*/
{TIA_AMB_GAIN_ADDR, TIA_AMB_GAIN_VAL}, /*TIA_AMB_GAIN*/
{LEDCNTRL1_ADDR, 		LEDCNTRL1_VAL}, /*LEDCNTRL1*/
{LEDCNTRL2_ADDR, 		LEDCNTRL2_VAL}, /*LEDCNTRL2*/
{OFFDAC_ADDR, 			OFFDAC_VAL}, /*OFFDAC*/
{I_OFFDAC_ADDR, 		I_OFFDAC_VAL}, /*I_OFFDAC*/
/***************************************************/
{0x01, 0x00000B}, /*LED2STC*/
{0x02, 0x00000D}, /*LED2ENDC*/
{0x03, 0x000014}, /*LED1LEDSTC*/
{0x04, 0x000017}, /*LED1LEDENDC*/
{0x05, 0x000010}, /*ALED2STC*/
{0x06, 0x000012}, /*ALED2ENDC*/
{0x07, 0x000015}, /*LED1STC*/
{0x08, 0x000017}, /*LED1ENDC*/
{0x09, 0x00000A}, /*LED2LEDSTC*/
{0x0A, 0x00000D}, /*LED2LEDENDC*/
{0x0B, 0x00001A}, /*ALED1STC*/
{0x0C, 0x00001C}, /*ALED1ENDC*/
{0x0D, 0x00000F}, /*LED2CONVST*/
{0x0E, 0x000012}, /*LED2CONVEND*/
{0x0F, 0x000014}, /*ALED2CONVST*/
{0x10, 0x000017}, /*ALED2CONVEND*/
{0x11, 0x000019}, /*LED1CONVST*/
{0x12, 0x00001C}, /*LED1CONVEND*/
{0x13, 0x00001E}, /*ALED1CONVST*/
{0x14, 0x000021}, /*ALED1CONVEND*/
{0x1D, 0x000200}, /*PRPCOUNT*/
{0x28, 0x103531}, /*TOGGLE*/
{0x29, 0x000000}, /*CLKDIV1*/
{0x2A, 0x190BE7}, /*LED2VAL*/
{0x2B, 0x01509C}, /*ALED2VAL*/
{0x2C, 0x002282}, /*LED1VAL*/
{0x2D, 0x1FFF19}, /*ALED1VAL*/
{0x2E, 0x18195A}, /*LED2-ALED2VAL*/
{0x2F, 0xE02A8F}, /*LED1-ALED1VAL*/ 
{0x34, 0x000000}, /*PROG_INT2_STC*/
{0x35, 0x000000}, /*PROG_INT2_ENDC*/
{0x36, 0x00000F}, /*LED3LEDSTC*/
{0x37, 0x000012}, /*LED3LEDENDC*/
{0x39, 0x000004}, /*CLKDIV2*/
{0x3B, 0x000000}, /*THRDETLOW*/
{0x3C, 0x000000}, /*THRDETHIGH*/
{0x3D, 0x000000}, /*THRDET*/
{0x3F, 0x000000}, /*AVG_LED2_ALED2VAL*/
{0x40, 0x000000}, /*AVG_LED1_ALED1VAL*/
{0x42, 0x000000}, /*FIFO*/
{0x43, 0x000019}, /*LED4LEDSTC*/
{0x44, 0x00001C}, /*LED4LEDENDC*/
{0x45, 0x000000}, /*TG_PD1STC*/
{0x46, 0x000000}, /*TG_PD1ENDC*/
{0x47, 0x000000}, /*TG_PD2STC*/
{0x48, 0x000000}, /*TG_PD2ENDC*/
{0x49, 0x000000}, /*TG_PD3STC*/
{0x4A, 0x000000}, /*TG_PD3ENDC*/
{0x4E, 0x000000}, /*DUAL_PD*/
{0x51, 0x000000}, /*FIFO_OFFSET*/
{0x52, 0x00002F}, /*DATA_RDY_STC*/
{0x53, 0x00002F}, /*DATA_RDY_ENDC*/
{0x54, 0x000000}, /*MASK_PPG*/
{0x57, 0x000000}, /*PROG_INT1_STC*/
{0x58, 0x000000}, /*PROG_INT1_ENDC*/
{0x61, 0x080000}, /*ECG_CHOP*/
{0x62, 0x800000}, /*ECG_RLD*/
{0x63, 0x000000}, /*RCOMP*/
{0x64, 0x000000}, /*DYN_TIA_STC*/
{0x65, 0x000023}, /*DYN_TIA_ENDC*/
{0x66, 0x000000}, /*DYN_ADC_STC*/
{0x67, 0x000023}, /*DYN_ADC_ENDC*/
{0x68, 0x000000}, /*DYN_CLOCK_STC*/
{0x69, 0x000023}, /*DYN_CLOCK_ENDC*/
{0x6A, 0x00002E}, /*DEEP_SLEEP_STC*/
{0x6B, 0x0001F9}, /*DEEP_SLEEP_ENDC*/
{0x6C, 0x000000}, /*PD_SHORT*/
{0x6D, 0x00007F}, /*REG_POINTER*/
{0x72, 0x000000}, /*LED_DRIVER_CONTROL*/
{0x73, 0x000000}, /*THR_DETECT_LOGIC*/
};

/*
* SEN is 0: addr = 0x5B
* SEN is 1: addr = 0x5A
*/ 


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

    twi_transfer(&seq);
    
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

    twi_transfer(&seq);

    
}
/*
*
 * For 500Hz PRF
 * {0x1D, 0x00007F}, *PRPCOUNT*
 * {0x6B, 0x000078}, *DEEP_SLEEP_ENDC*
 *
 * For 250Hz PRF
 * {0x1D, 0x0000FF}, *PRPCOUNT*
 * {0x6B, 0x0000F8}, *DEEP_SLEEP_ENDC*
 *
 * For 125Hz PRF
 * {0x1D, 0x000200}, *PRPCOUNT*
 * {0x6B, 0x0001F9}, *DEEP_SLEEP_ENDC**
 *

*/
static void AFE4409_set_dryFreq(unsigned int prpcount, unsigned int deep_sleep_endc){
    
    AFE4409_Reg_Write(0x1d, prpcount);
    AFE4409_Reg_Write(0x6b, deep_sleep_endc);
    AFE4409_EGC_Reg_Write(0x1d, prpcount);
    AFE4409_EGC_Reg_Write(0x6b, deep_sleep_endc);		
}

/**@brief init AFE4409 Reg 
 */
static void AFE4409_Reg_Init(void)
{
    
    int i = 0;
		signed long retVal =0;
		
		
		//nrf_delay_us(2000);
		
		if(1){
			
			#if 1
			AFE4409_Reg_Read(0x23,(unsigned char*)&retVal);		
			AFE4409_Reg_Write(0, 0x08);
			nrf_delay_us(20);
			for(i = 0; i< MAX_REG_NUMBER;i++){
			AFE4409_Reg_Write(reg_init[i][0],reg_init[i][1]);
			}
		
			retVal = AFE4409_Reg_Read(0x23,(unsigned char*)&retVal);
			AFE4409_Reg_Write(0, 0x20);	//ENABLE ULP mode
			NRF_LOG_INFO("init ppg");
			#endif
			// init EGC sensor
			#if 1
			AFE4409_EGC_Reg_Write(0, 0x08);
			nrf_delay_us(20);
			for(i = 0; i< MAX_REG_NUMBER;i++){
				AFE4409_EGC_Reg_Write(egc_reg_init[i][0],egc_reg_init[i][1]);
				//AFE4409_EGC_Reg_Write(reg_init[i][0],reg_init[i][1]);
			}
			
			AFE4409_EGC_Reg_Read(0x23,(unsigned char*)&retVal);
			AFE4409_EGC_Reg_Write(0, 0x20);	//ENABLE ULP mode
			NRF_LOG_INFO("init egc");
			#endif
            /*
            *
             * For 500Hz PRF
             * {0x1D, 0x00007F}, *PRPCOUNT*
             * {0x6B, 0x000078}, *DEEP_SLEEP_ENDC*
						 * Real Measured Frequency: 501.5-502.0Hz, Period: 1.992-1.994ms
             *
             * For 250Hz PRF
             * {0x1D, 0x0000FF}, *PRPCOUNT*
             * {0x6B, 0x0000F8}, *DEEP_SLEEP_ENDC*
						 * Real measured Frequency: 250.6-250.8Hz, Period: 3.988-3.990ms
						 *
             * For 125Hz PRF
             * {0x1D, 0x000200}, *PRPCOUNT*
             * {0x6B, 0x0001F9}, *DEEP_SLEEP_ENDC**
						 * Real measured Frequency: 125.0Hz, Period: 7.999-8.001ms
             *
            */
            AFE4409_set_dryFreq(0x0000FF,0x0000F8);
		
		}
		
}


/**@brief read data from device 
 * @param reg :  write cmd
 */
void AFE4409_volReg_Read_Data(unsigned char * buf)
{	
		signed long val = 0;
	#if 1
    val = AFE4409_Reg_Read(0x2a,buf);  //read PPG Data
		//memcpy(buf,(unsigned char*)val,3);
    val = AFE4409_Reg_Read(0x2b,&buf[3]);  //read AMB Data
    val = AFE4409_Reg_Read(0x2c,&buf[6]); 
    val = AFE4409_Reg_Read(0x2d,&buf[9]); 
    #else
    I2C_read (AFE4409_I2C_DEFAULT_ADDRESS, 0x2a, buf, 12);
	#endif
	
}


/**@brief init AFE4409
 */
void AFE4900_Init(void)
{
  AFE4409_RESETZ_Init ();
  //AFE4409_Enable_HWPDN ();
	
  //AFE4409_Disable_HWPDN ();
  AFE4409_Trigger_HWReset ();
  AFE4409_ADCRDY_Interrupt_Init();
  AFE4409_Reg_Init();
  AFE4409_Enable_SWPDN();
}


/**@brief init AFE4409 RESETZ
 */
void AFE4409_RESETZ_Init (void)
{
	SET_AFE_RESETZ_AS_OUTPUT();
}


/**@brief enable AFE4409 HWPDN
 */
void AFE4409_Enable_HWPDN (void)
{
 
	
  //__delay_cycles(160000);        // ~10ms delay with 16MHz clock
  nrf_delay_us(500); // when the RESTZ pin is pulled low for more than 200us , the device will enters hardware powerdown mode
}


/**@brief disable AFE4409 HWPDN
 */
void AFE4409_Disable_HWPDN (void)
{
	
    
 
  //nrf_delay_us(10*1000);//__delay_cycles(160000);        // ~10ms delay with 16MHz clock
    nrf_delay_us(200);
}


/**@brief reset AFE4409 Trigger
 */
void AFE4409_Trigger_HWReset (void)
{
  SET_AFE_RESETZ_PIN_LOW();
  //__delay_cycles(480);           // ~30us delay
  nrf_delay_us(30); // pulsing the RESETZ pin low for a duration of time between 25 to 50us
  SET_AFE_RESETZ_PIN_HIGH();
  nrf_delay_us(10*1000);//__delay_cycles(160000);        // ~10ms delay with 16MHz clock
  // nrf_delay_us(30);
}


/**@brief enable AFE4409 SWPDN
 */
void AFE4409_Enable_SWPDN(void)
{
    
    unsigned long val = 0;
	
	AFE4409_Reg_Read(0x23, (unsigned char *)&val);
	
	val = val |0x000001;
	AFE4409_Reg_Write(0x23,val);
	
	AFE4409_EGC_Reg_Read(0x23, (unsigned char *)&val);
	
	val = val |0x000001;
	AFE4409_Reg_Write(0x23,val);
    
}


/**@brief disable AFE4409 SWPDN
 */
void AFE4409_Disable_SWPDN(void)
{
    
    unsigned long val = 0;
	
	AFE4409_Reg_Read(0x23, (unsigned char *)&val);
	
	val = val & 0xfffffe;
	AFE4409_Reg_Write(0x23,val);
	
	AFE4409_EGC_Reg_Read(0x23, (unsigned char *)&val);
	
	val = val & 0xfffffe;
	AFE4409_Reg_Write(0x23,val);
    
    AFE4409_Reg_Init();
    
}


/**@brief read data from device 
 * @param reg :  write cmd
 */
void AFE4409_EGC_volReg_Read_Data(unsigned char * buf)
{	
		signed long val = 0;

    //AFE4409_EGC_Reg_Read(0x2a,buf);  //read PPG Data
    //AFE4409_EGC_Reg_Read(0x2b,&buf[3]);  //read AMB Data
    //AFE4409_EGC_Reg_Read(0x2c,&buf[6]); 
    val = AFE4409_EGC_Reg_Read(0x2d,&buf[12]); 
		//slice_drv_afe4409_disable_interrupt();
}
/**@brief write AFE4409 reg
 */
void AFE4409_EGC_Reg_Write (unsigned char reg_address, unsigned int data)
{
  unsigned char configData[4];
  configData[0] =reg_address;
  configData[1]=(unsigned char)(data >>16);
  configData[2]=(unsigned char)(((data & 0x00FFFF) >>8));
  configData[3]=(unsigned char)(((data & 0x0000FF)));
  I2C_write(AFE4409_I2C_DEFAULT_ADDRESS_EGC, configData, 4);
}

/**@brief read AFE4409 reg
 */
signed long AFE4409_EGC_Reg_Read(unsigned char Reg_address,unsigned char *configData)
{
  //unsigned char configData[3];
  signed long retVal;
  I2C_read (AFE4409_I2C_DEFAULT_ADDRESS_EGC, Reg_address, configData, 3);
  retVal = configData[0];
  retVal = (retVal << 8) | configData[1];
  retVal = (retVal << 8) | configData[2];
	/*
	//retVal = retVal/1000;
	if (Reg_address >= 0x2A && Reg_address <= 0x2F)
  {
		if (retVal & 0x00200000) 	// check if the ADC value is positive or negative
		{
			retVal &= 0x003FFFFF;		// convert it to a 22 bit value
			return (retVal^0xFFC00000);// wrong convertion: it will return the same value or retVal
		}
  }
	*/
  return retVal;
}


/**@brief write AFE4409 reg
 */
void AFE4409_Reg_Write (unsigned char reg_address, unsigned int data)
{
  unsigned char configData[4];
  configData[0] =reg_address;
  configData[1]=(unsigned char)(data >>16);
  configData[2]=(unsigned char)(((data & 0x00FFFF) >>8));
  configData[3]=(unsigned char)(((data & 0x0000FF)));
  I2C_write(AFE4409_I2C_DEFAULT_ADDRESS, configData, 4);
}



/**@brief read AFE4409 reg
 */
signed long AFE4409_Reg_Read(unsigned char Reg_address,unsigned char *configData)
{
  //unsigned char configData[3];
  signed long retVal;
  I2C_read (AFE4409_I2C_DEFAULT_ADDRESS, Reg_address, configData, 3);
  retVal = configData[0];
  retVal = (retVal << 8) | configData[1];
  retVal = (retVal << 8) | configData[2];
	/*
  if (Reg_address >= 0x2A && Reg_address <= 0x2F)
  {
	if (retVal & 0x00200000) 	// check if the ADC value is positive or negative
	{
	  retVal &= 0x003FFFFF;		// convert it to a 22 bit value
	  return (retVal^0xFFC00000);
	}
  }
	*/
  return retVal;
}


/**@brief enable AFE4409 read
 */
void AFE4409_Enable_Read (void)
{
}


/**@brief disable AFE4409 read
 */
void AFE4409_Disable_Read (void)
{
}


/**@brief init AFE4409 ADCRDY Interrup
 */
void AFE4409_ADCRDY_Interrupt_Init (void)
{
  
  //SET_AFE_ADC_RDY_AS_INPUT();
  //ENABLE_MSP_INT_PU_RES_ON_ADC_RDY_PIN();
  //SET_MSP_INT_PU_RES_ON_ADC_RDY_PIN();
  //SEL_H2L_EDGE_ON_ADC_RDY_PIN();
  AFE4409_ADCRDY_Interrupt_Disable();
}


/**@brief enable  AFE4409 AFE4409 adcrdy interrupt
 */
void AFE4409_ADCRDY_Interrupt_Enable (void)
{
  //CLEAR_INT_FLAG_ON_ADC_RDY_PIN();      //P2IFG &= ~AFE_ADC_DRDY;                           	// P2.3 IFG cleared
  //ENABLE_INT_ON_ADC_RDY_PIN();          //P2IE |= AFE_ADC_DRDY;                             	// P2.3 interrupt enabled
}


/**@brief disable AFE4409 adcrdy interrupt
 */
void AFE4409_ADCRDY_Interrupt_Disable (void)
{
  //CLEAR_INT_FLAG_ON_ADC_RDY_PIN();      //P2IFG &= ~AFE_ADC_DRDY;                           	// P2.3 IFG cleared
  //DISABLE_INT_ON_ADC_RDY_PIN();         //P2IE &= ~AFE_ADC_DRDY;                             	// P2.3 interrupt disabled
}
// End of file

void slice_drv_afe4409_disable_interrupt(void)
{
    nrf_drv_gpiote_in_event_disable(AFE_IRQ_PIN_NO);
}


void slice_drv_afe4409_enable_interrupt(void)
{
    nrf_drv_gpiote_in_event_enable(AFE_IRQ_PIN_NO, true);
}

void temp(){
	AFE4409_Reg_Init();
}