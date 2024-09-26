#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"                        // Device header
#include "app_twi.h"
#include "slice_drv_afe4404.h"
#include "app_error.h"
#include "app_uart.h"
//#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "Si114x_types.h"
#include "Si114x_functions.h"
#include "si114x_sys_out.h"
#include "HRM_Config.h"


// GGG LEDs config
HrmConfiguration_t defaultWristHRMConfig = {
  0x0001+0x0010+0x0100,      //ledCurrent
  0x37,           //tasklist
  0x0003+0x0030+0x0300,      //psLedSelect
  0xa0,        //measurementRate
  3,           //adcgain 3
  0x04,        //adcMisc
  0x333,       //adcMux 0x333
  0,           //ps_align
};
HrmConfiguration_t *currentHRMConfig;
static HANDLE si114xHandle;

int32_t HeartRateMonitor_Configure(HrmConfiguration_t *configuration)
{	int16_t current[3];
  int16_t adcMux[3];
  int16_t psLedSelect;
  int16_t  retval=0;
    
  currentHRMConfig=configuration;
  current[0] = configuration->ledCurrent & 0xF;
  current[1] = (configuration->ledCurrent>>4) & 0xF;
  current[2] = (configuration->ledCurrent>>8) & 0xF;
  adcMux[0] = (configuration->adcMux>>0) & 0xF;
  adcMux[1] = (configuration->adcMux>>4) & 0xF;
  adcMux[2] = (configuration->adcMux>>8) & 0xF;
  psLedSelect = configuration->psLedSelect & ((1<<(10+1))-1); //Get bits[10:0]


// Note that the Si114xReset() actually performs the following functions:
//     1. Pauses all prior measurements
//     2. Clear  i2c registers that need to be cleared
//     3. Clears irq status to make sure INT* is negated
//     4. Delays 10 ms
//     5. Sends HW Key
  retval += Si114xReset(si114xHandle);
    // Write Hardware Key, do it in Si114xReset
    //retval+=Si114xWriteToRegister(si114x_handle, REG_HW_KEY, HW_KEY_VAL0);//0x07 0x17
// Get the LED current passed from the caller
  {
    uint8_t i21, i3;
    i21 = (current[1] << 4) + current[0];
    i3 = current[2];
    retval += Si114xWriteToRegister(si114xHandle, REG_PS_LED21, i21);//0x0f
    retval += Si114xWriteToRegister(si114xHandle, REG_PS_LED3, i3);//0x10
	}
   retval += Si114xWriteToRegister(si114xHandle, REG_INT_CFG, ICG_INTOE);//0x03 , 0x01

  //Set IRQ Modes and INT CFG to interrupt on every sample
  //Note: Interrupts are setup here but not enabled until _Run().
    retval = Si114xWriteToRegister(si114xHandle, REG_IRQ_ENABLE, 0x10 );//0x04
    
  retval += Si114xWriteToRegister(si114xHandle, REG_IRQ_MODE1,
      IM1_ALS_EVRYSAMPLE +
      IM1_PS1_EVRYSAMPLE +
      IM1_PS2_EVRYSAMPLE);//0x05

  retval += Si114xWriteToRegister(si114xHandle, REG_IRQ_MODE2,
      IM2_PS3_EVRYSAMPLE);//0x06
   

// Initialize CHLIST Parameter from caller to enable measurement
// Valid Tasks are: ALS_VIS_TASK, ALS_IR_TASK, PS1_TASK
//                  PS2_TASK, PS3_TASK and AUX_TASK
  retval += Si114xParamSet(si114xHandle, PARAM_CH_LIST, configuration->tasklist);  //0x01

  retval += Si114xParamSet(si114xHandle, PARAM_PSLED12_SELECT, psLedSelect & 0x77);//0x02
  retval += Si114xParamSet(si114xHandle, PARAM_PSLED3_SELECT, (psLedSelect >> 8) & 0x7);//0x03

  retval += Si114xParamSet(si114xHandle, PARAM_PS_ADC_GAIN, configuration->adcGain);
  retval += Si114xParamSet(si114xHandle, PARAM_PS_ADC_MISC, configuration->adcMisc);
  retval += Si114xParamSet(si114xHandle, PARAM_PS_ENCODING, configuration->psAlign);//Default: PS1_ALIGN=PS2_ALIGN=PS3_ALIGN=0
  retval += Si114xParamSet(si114xHandle, PARAM_PS1_ADC_MUX, adcMux[0]);
  retval += Si114xParamSet(si114xHandle, PARAM_PS2_ADC_MUX, adcMux[1]);
  retval += Si114xParamSet(si114xHandle, PARAM_PS3_ADC_MUX, adcMux[2]);
 

// Configure the ALS VIS range bit
// retval += Si114xParamSet(si114xHandle, PARAM_ALSVIS_ADC_MISC, 0x0);		//This should be zero

// Configure the ALS IR channel for the same settings as PS
  retval += Si114xParamSet(si114xHandle, PARAM_ALSIR_ADC_GAIN,0x00);// configuration->adcGain);
//  retval += Si114xParamSet(si114xHandle, PARAM_ALSIR_ADC_MISC, configuration->adcMisc & 0x20);
  retval += Si114xParamSet(si114xHandle, PARAM_IR_ADC_MUX, 0x03);//adcMux[0]);

#if 0
// added
  retval += Si114xParamSet(si114xHandle, PARAM_ALSIR_ADC_COUNTER, 0x20);//ALS_IR_ADC_CNTR
  retval += Si114xParamSet(si114xHandle, PARAM_PS_ADC_COUNTER, 0x00);//ALS_IR_ADC_CNTR
#endif

/*	Initially set the measurement rate register to 0 to enable forced measurements.  The specified
 * 	measurement rate is set in the function _Run just before autonomous measurements are
 * 	started.
 */

 // retval += Si114xWriteToRegister(si114xHandle, REG_MEAS_RATE, 0);

// if 0x08, VIS, IR, AUX Measurements every time device wakes up.
  retval += Si114xWriteToRegister(si114xHandle, REG_ALS_RATE, 0x08);//0x09 0x08

// if 0x08, PS1, PS2 and PS3 made every time device wakes up.
  retval += Si114xWriteToRegister(si114xHandle, REG_PS_RATE, 0x08);//0x0a 0x08

  return retval;
}

int32_t HeartRateMonitor_init(si114x_port_config_t *Port)
{
    int32_t error = 0;
    Si114xInit(Port,0,&si114xHandle);
    si114xHandle = Port;
    HeartRateMonitor_Configure(&defaultWristHRMConfig);
    return error;   
}

void HeartRateMonitor_interrupt_handle(void){
   Si114xProcessIrq(si114xHandle,0);
}
/*  This function will start the device's autonomous measurement operation.
 * 	The device must be configured before calling this function.
 *
 */
int32_t HeartRateMonitor_Run(void)
{	int error = 0;

//  uint16_t meas_rate_uv;

// Set up how often the device wakes up to make measurements
// measurementRate, for example can be the following values:
//    0xa0 = Device Wakes up every ~30 ms
//    0x94 = Device Wakes up every ~20 ms
//    0x84 = Device Wakes up every ~10 ms
//    0xB9 = Device Wakes up every ~100 ms
//    0xFF = Device Wakes up every ~2 sec
  HeartRateMonitor_EnableInterrupts();
  Si114xWriteToRegister(si114xHandle, REG_MEAS_RATE, (uint8_t)currentHRMConfig->measurementRate);//0x08

  Si114xPsAlsAuto(si114xHandle);

  return error;
}


int32_t HeartRateMonitor_Pause(void)
{	int error = 0;

  //Set the measurement rate register to 0 to allow for forced measurements
  Si114xWriteToRegister(si114xHandle, REG_MEAS_RATE, 0);

  // Pause Autonomous Operation
  Si114xPauseAll(si114xHandle);

  HeartRateMonitor_DisableInterrupts();

  return error;
}

int HeartRateMonitor_EnableInterrupts(void)
{	int error = 0;
   
  error = Si114xWriteToRegister(si114xHandle, REG_IRQ_ENABLE,
      IE_ALS_EVRYSAMPLE +
	    IE_PS1_EVRYSAMPLE +
      IE_PS2_EVRYSAMPLE +
	    IE_PS3_EVRYSAMPLE );
   
    //error = Si114xWriteToRegister(si114xHandle, REG_IRQ_ENABLE, 0x10 );//0x04
  return error;
}

int HeartRateMonitor_DisableInterrupts(void)
{	int error = 0;

  error = Si114xWriteToRegister(si114xHandle, REG_IRQ_ENABLE, 0);				//disable interrupts
  return error;
}
