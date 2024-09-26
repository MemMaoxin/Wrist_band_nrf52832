#ifndef _AFE4404_SLICE_H_
#define _AFE4404_SLICE_H_

#include "slice_drv_twi.h"
#include "slice_cfg.h"
#include "nrf_gpio.h"

#define SET_AFE_RESETZ_AS_OUTPUT()                      nrf_gpio_cfg_output(AFE_RST_PIN_NO)
#define SET_AFE_RESETZ_PIN_LOW()                        nrf_gpio_pin_clear(AFE_RST_PIN_NO)
#define SET_AFE_RESETZ_PIN_HIGH()                       nrf_gpio_pin_set(AFE_RST_PIN_NO) 

#endif
