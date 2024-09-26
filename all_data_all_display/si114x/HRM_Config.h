#ifndef HRM_CONFIG_H
#define HRM_CONFIG_H

#include "si114x_sys_out.h"

typedef struct HrmConfiguration
{
  int16_t ledCurrent;	      /**< bits 15:12 = unused, bits 11:8=led3 current, bits 7:4=led2 current, bits 3:0=led1 current */
  int16_t tasklist;
  int16_t psLedSelect;
  int16_t measurementRate;   /**< Always compressed. The algorithm will read uv_part from the chip and then program the chip with correct measurementRate accordingly.*/
  int16_t adcGain;
  int16_t adcMisc;
  int16_t adcMux;					/**< bits 15:12 = unused, bits 11:8=ps3, bits 7:4=ps2, bits 3:0=ps1 */
  int16_t psAlign;
} HrmConfiguration_t;

/**@brief Function for config the HeartRateMonitor, this function will be called by HeartRateMonitor_init using default param
 *        so, call this function after Monitor inited.
 *
 * @param[in]   HrmConfiguration_t .
 *
 * @return   0 on success, otherwise an error code.
 */

int32_t HeartRateMonitor_Configure(HrmConfiguration_t *configuration);

/**@brief Function for initializing the HeartRateMonitor.
 *
 * @param[in]   si114x_port_config_t   Information for Si114x pin and i2c address.
 *
 * @return   0 on success, otherwise an error code.
 */
int32_t HeartRateMonitor_init(si114x_port_config_t *Port);

/**@brief This function will start the device's autonomous measurement operation.
 * 	      The device must be configured before calling this function.
 *
 */
int32_t HeartRateMonitor_Run(void);

int32_t HeartRateMonitor_Pause(void);

int HeartRateMonitor_EnableInterrupts(void);

int HeartRateMonitor_DisableInterrupts(void);

/**
 * @brief Function for Si114x interrupt callback handle
 */

void HeartRateMonitor_interrupt_handle(void);

#endif
