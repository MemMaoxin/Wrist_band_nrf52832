// TODO: Add file header / copyright notice

#include "slice_sm_button.h"
#include "nrf_assert.h"
#include "app_timer.h"
#include "app_button.h"
#include "sdk_common.h"
#include "slice_cfg.h"


#define IDLE_TO_LONG_DELAY       APP_TIMER_TICKS(1000)     /**< Time from button is pushed until LONG button event is reported. */
#define LONG_TO_VERYLONG_DELAY   APP_TIMER_TICKS(3000)     /**< Time from LONG button event is reported until VERYLONG button event is reported. */

/**@brief Button State Machine event type */
typedef enum
{
    FSM_EVT_BUTTON_PUSH,                        /**< Button has been pushed. */
    FSM_EVT_BUTTON_RELEASE,                     /**< Button has been released. */
    FSM_EVT_TIMEOUT                             /**< The state machine timer has expired. */
} fsm_evt_t;

/**@brief Button State Machine state type */
typedef enum
{
    FSM_STATE_IDLE,                             /**< Button is not pushed. */
    FSM_STATE_WAITING_LONG,                     /**< Button is pushed, waiting for time to report LONG event. */
    FSM_STATE_WAITING_VERYLONG                  /**< Button is pushed, waiting for time to report VERYLONG event. */
} fsm_state_t;

static fsm_state_t            m_fsm_state;      /**< Current state. */
static slice_sm_button_init_t m_cfg;            /**< State machine configuration. */

APP_TIMER_DEF(m_button_timer_id);                      /**< Button press type timer. */

uint8_t m_button_pin_no = 0;

/**@brief Start the Button State Machine timer.
 *
 * @param[in]   timeout   Number of app_timer ticks until the timer shall expire.
 */
static void timer_start(uint32_t timeout)
{
    uint32_t err_code;
    
    err_code = app_timer_start(m_button_timer_id, timeout, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Stop the Button State Machine timer.
 */
static void timer_stop(void)
{
    uint32_t err_code;
    
    err_code = app_timer_stop(m_button_timer_id);
    APP_ERROR_CHECK(err_code);
}


/**@brief Notify application about a Button State Machine event.
 *
 * @param[in]   evt_type   Type of event.
 */
static void event_generate(uint8_t pin_no,slice_sm_button_evt_type_t evt_type)
{
    slice_sm_button_evt_t m_evt;
    
    m_evt.pin_no = pin_no;
    m_evt.evt_type = evt_type;
    m_cfg.evt_handler(&m_evt);//test//////////////////////////////////////////////////////////////////////////////
}


/**@brief Handle event in WAITING_LONG state.
 *
 * @param[in]   fsm_evt   Button event.
 */
static void idle_state_event(fsm_evt_t fsm_evt)
{
    switch (fsm_evt)
    {
        case FSM_EVT_BUTTON_PUSH:
            timer_start(IDLE_TO_LONG_DELAY);
            m_fsm_state = FSM_STATE_WAITING_LONG;
						//EnableRR();
            break;
            
        case FSM_EVT_BUTTON_RELEASE:
            // Do nothing
            break;
            
        case FSM_EVT_TIMEOUT:
        default:
            APP_ERROR_HANDLER((int)fsm_evt);
            break;
    }
}


/**@brief Handle event in WAITING_LONG state.
 *
 * @param[in]   fsm_evt   Button event.
 */
static void waiting_long_state_event(uint8_t pin_no,fsm_evt_t fsm_evt)
{
    switch (fsm_evt)
    {
        case FSM_EVT_BUTTON_RELEASE:
            timer_stop();
            event_generate(pin_no,SLICE_SM_BUTTON_PRESS_SINGLE);
            m_fsm_state = FSM_STATE_IDLE;
            break;
            
        case FSM_EVT_TIMEOUT:
            timer_start(LONG_TO_VERYLONG_DELAY);
            event_generate(pin_no,SLICE_SM_BUTTON_PRESS_LONG);
            m_fsm_state = FSM_STATE_WAITING_VERYLONG;
            break;
            
        case FSM_EVT_BUTTON_PUSH:
        default:
            APP_ERROR_HANDLER((int)fsm_evt);
            break;
    }
}


/**@brief Handle event in WAITING_VERYLONG state.
 *
 * @param[in]   fsm_evt   Button event.
 */
static void waiting_verylong_state_event(uint8_t pin_no,fsm_evt_t fsm_evt)
{
    switch (fsm_evt)
    {
        case FSM_EVT_BUTTON_RELEASE:
            // BUTTON_PRESS_LONG event has already been generated, so just stop timer
            timer_stop();
            m_fsm_state = FSM_STATE_IDLE;
            break;
            
        case FSM_EVT_TIMEOUT:
            event_generate(pin_no,SLICE_SM_BUTTON_PRESS_VERYLONG);
            m_fsm_state = FSM_STATE_IDLE;
            break;
            
        case FSM_EVT_BUTTON_PUSH:
        default:
            APP_ERROR_HANDLER((int)fsm_evt);
            break;
    }
}


/**@brief Button state machine.
 *
 * @param[in]   fsm_evt   Button event.
 */
static void fsm(uint8_t pin_no,fsm_evt_t fsm_evt)
{
    switch (m_fsm_state)
    {
        case FSM_STATE_IDLE:
            idle_state_event(fsm_evt);
            break;
            
        case FSM_STATE_WAITING_LONG:
            waiting_long_state_event(pin_no,fsm_evt);/////////////////////////////////////
            break;
            
        case FSM_STATE_WAITING_VERYLONG:
            waiting_verylong_state_event(pin_no,fsm_evt);
            break;
            
        default:
            APP_ERROR_HANDLER((int)fsm_evt);
            break;
    }
}

void slice_sm_button_on_app_button_evt(uint8_t pin_no, uint8_t button_action)///////////////////////////////////
{
    UNUSED_PARAMETER(pin_no);
    ASSERT(pin_no == m_cfg.pin_no);
    
    m_button_pin_no = pin_no;
    
    switch (button_action)
    {
        case APP_BUTTON_PUSH:
            fsm(pin_no,FSM_EVT_BUTTON_PUSH);
            break;
            
        case APP_BUTTON_RELEASE:
            fsm(pin_no,FSM_EVT_BUTTON_RELEASE);
            break;
            
        default:
            APP_ERROR_HANDLER(button_action);
            break;
    }
}


/**@brief Timeout handler for destinguishing different button event types.
 *
 * @param[in]   p_context   Pointer used to pass context information to timeout event.
 */
static void button_evt_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    fsm(m_button_pin_no,FSM_EVT_TIMEOUT);
}


uint32_t slice_sm_button_init(const slice_sm_button_init_t * p_slice_sm_button_init)
{
    m_cfg       = *p_slice_sm_button_init;
    m_fsm_state = FSM_STATE_IDLE;
    
    return app_timer_create(&m_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT, button_evt_timeout_handler);
}
