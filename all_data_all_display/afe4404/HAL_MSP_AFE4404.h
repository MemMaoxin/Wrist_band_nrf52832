/*
 * HAL_MSP_AFE4404.h
 *
 * Provides AFE4404 interface definitions with MSP430F552x
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 * ALL RIGHTS RESERVED  
 *
*/

#ifndef _HAL_MSP_AFE4404_H_
#define _HAL_MSP_AFE4404_H_

#include "nrf.h"   
#include "slice_cfg.h"
#include "afe4404_slice.h"

/*----------------------------------------------------------------------------+
| Bits Definition                                                         |
+----------------------------------------------------------------------------*/
#define AFE_RESETZ      1
#define AFE_ADC_DRDY    1<<3

#define st(x)      do { x } while (__LINE__ == -1)

//#define SET_AFE_ADC_RDY_AS_INPUT()                      st(P2DIR &= ~AFE_ADC_DRDY;)
//#define ENABLE_MSP_INT_PU_RES_ON_ADC_RDY_PIN()          st(P2REN |= AFE_ADC_DRDY;)
//#define SET_MSP_INT_PU_RES_ON_ADC_RDY_PIN()             st(P2OUT |= AFE_ADC_DRDY;)
//#define SEL_H2L_EDGE_ON_ADC_RDY_PIN()                   st(P2IES |= AFE_ADC_DRDY;)
//#define CLEAR_INT_FLAG_ON_ADC_RDY_PIN()                 st(P2IFG &= ~AFE_ADC_DRDY;)
//#define ENABLE_INT_ON_ADC_RDY_PIN()                     st(P2IE |= AFE_ADC_DRDY;)
//#define DISABLE_INT_ON_ADC_RDY_PIN()                    st(P2IE &= ~AFE_ADC_DRDY;)

#define SET_AFE_RESETZ_AS_OUTPUT()                      nrf_gpio_cfg_output(AFE_RST_PIN)
#define SET_AFE_RESETZ_PIN_LOW()                        nrf_gpio_pin_clear(AFE_RST_PIN)
#define SET_AFE_RESETZ_PIN_HIGH()                       nrf_gpio_pin_set(AFE_RST_PIN) 

#define I2C_write       UCB1_MasterI2C_write
#define I2C_read        UCB1_MasterI2C_read

#endif /*_HAL_MSP_AFE4404_H_*/
