
#ifndef _SLICE_DRV_AGC_H_
#define _SLICE_DRV_AGC_H_

#include "stdint.h"
#include <stdbool.h>

/**@brief This function will get onwrist_value
 */
void slice_driver_get_onwrist_value(bool onwrist_value);

/**@brief This function will agc_calc
 */
void slice_driver_agc_calc(int32_t ppg_curr, int32_t amb_curr, float snr);

/**@brief This function will set afe current
 */
void slice_driver_set_afe_current(void);
    
#endif
