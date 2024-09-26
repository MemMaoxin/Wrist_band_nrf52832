// TODO: Add file header / copyright notice

#ifndef SLICE_SM_BUTTON_H__
#define SLICE_SM_BUTTON_H__

#include <stdint.h>
#include "nrf_gpio.h"

/**@brief Button State Machine event type */
typedef enum
{
    SLICE_SM_BUTTON_PRESS_SINGLE,                   /**< Short button press */
    SLICE_SM_BUTTON_PRESS_LONG,                     /**< Long button press */
    SLICE_SM_BUTTON_PRESS_VERYLONG                  /**< Very long button press */
} slice_sm_button_evt_type_t;

/**@brief Button State Machine event */
typedef struct
{
    slice_sm_button_evt_type_t evt_type;            /**< Type of event. */
    uint8_t pin_no;
} slice_sm_button_evt_t;

/**@brief Button State Machine event handler type */
typedef void (*slice_sm_button_evt_handler_t)(slice_sm_button_evt_t * p_evt);

/**@brief Button State Machine configuration structure */
typedef struct
{
    uint8_t                       button_id;        /**< Id of Slice button. */
    uint8_t                       pin_no;           /**< GPIO pin number of Slice button. */
    slice_sm_button_evt_handler_t evt_handler;      /**< Handler to be called when button is pushed. */
} slice_sm_button_init_t;
    
/**@brief Function for initializing the Button State Machine.
 *
 * @param[in]   p_slice_sm_button_init   Information needed to initialize the module.
 *
 * @return   NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t slice_sm_button_init(const slice_sm_button_init_t * p_slice_sm_button_init);

/**@brief Function for handling events from the app_button module.
 *
 * @param[in]   pin_no          The pin that the event applies to.
 * @param[in]   button_action   The button action (press/release).
 */
void slice_sm_button_on_app_button_evt(uint8_t pin_no, uint8_t button_action);

#endif // SLICE_SM_BUTTON_H__
