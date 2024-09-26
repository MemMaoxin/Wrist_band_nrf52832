/*
 * AFE4409.h
 *
 * Provides AFE4409 API
 
 *
*/

#ifndef _AFE44xx_H_
#define _AFE44xx_H_

#define __AFE4409__
//#include "slice_drv_twi.h"
#include "nrf_drv_twi.h"
#include "slice_cfg.h"
#include "nrf_gpio.h"
#include "app_twi.h"
#define INTERNAL_CLOCK_4409
#define AFE4409_FREQ_4MHZ 2
#define AFE4409_FREQ_8MHZ 1

#define SET_AFE_RESETZ_AS_OUTPUT()                      nrf_gpio_cfg_output(AFE_RST_PIN_NO)
#define SET_AFE_RESETZ_PIN_LOW()                        nrf_gpio_pin_clear(AFE_RST_PIN_NO)
#define SET_AFE_RESETZ_PIN_HIGH()                       nrf_gpio_pin_set(AFE_RST_PIN_NO) 

#define AFE4409_I2C_DEFAULT_ADDRESS 0x5B
#define AFE4409_I2C_DEFAULT_ADDRESS_EGC 0x5A

#define MAX_REG_NUMBER 0x74

/****************************************************************/
/* Global functions												*/
/****************************************************************/
void AFE4900_Init(void);
void AFE4409_RESETZ_Init (void);
void AFE4409_Enable_HWPDN (void);
void AFE4409_Disable_HWPDN (void);
void AFE4409_Enable_SWPDN(void);
void AFE4409_Disable_SWPDN(void);
void AFE4409_Trigger_HWReset (void);
void AFE4409_ADCRDY_Interrupt_Init (void);
void AFE4409_ADCRDY_Interrupt_Enable (void);
void AFE4409_ADCRDY_Interrupt_Disable (void);

//unsigned char AFE4409_Diagnostics_Check (void);
void 	AFE4409_volReg_Read_Data(unsigned char * buf);

void AFE4409_Reg_Write(unsigned char reg_address, unsigned int reg_data);
signed long AFE4409_Reg_Read(unsigned char reg_address,unsigned char *buf);

void AFE4409_EGC_Reg_Write(unsigned char reg_address, unsigned int reg_data);
void 	AFE4409_EGC_volReg_Read_Data(unsigned char * buf);
signed long AFE4409_EGC_Reg_Read(unsigned char reg_address,unsigned char *buf);

void AFE4409_Enable_Read (void);
void AFE4409_Disable_Read (void);

void temp(void);
#endif /*_AFE44xx_H_*/