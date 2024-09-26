
#include "slice_driver_agc_calc.h"
#include "afe4404.h"
#include "nrf.h" 
#include "app_uart.h"

#define SLICE_AGC_LED_CURRENT_MAX_VALUE   (25)                                      /**< LED MAX value */
#define SLICE_AGC_LED_CURRENT_LOW_VALUE   (10)                                       /**< Lower level of LED current */
#define SLICE_AGC_LED_CURRENT_MIN_VALUE   (0)                                       /**< LED MIN value */
#define SLICE_AGC_BIAS_DEFAULT_VALUE      (-7)                                      /**< Default bias value */
#define SLICE_AGC_GAIN_DEFAULT_VALUE      AFE_TIA_GAIN_SEP_100K                     /**< Default gain value */
#define GAIN_DEFAULT_INDEX      (1)                                                     /**< Index of default gain value */

#define PPG_MAX                 (1992294)                             /**< PPG saturation value. */
#define LED_CURRENT_RATE_COUNT  (4)                       /**< Sample count before changing LED current value */

#define AFE_TIA_GAIN_SEP_50K    0x000003ul
#define AFE_TIA_GAIN_SEP_100K   0x000002ul
#define AFE_TIA_GAIN_SEP_250K   0x000001ul

static const uint8_t m_gains_arr[] = { AFE_TIA_GAIN_SEP_50K, AFE_TIA_GAIN_SEP_100K, AFE_TIA_GAIN_SEP_250K };

//m_on_wrist is a global variable which is true if watch is on hand, and false otherwise.
//m_current_curr_value is a global variable which holds the AFE led current. 
//m_on_wrist_inner is a global variable
//m_gain_curr_location is a global variable which holds the AFE gain. 
//m_led_current_rate is a global variable

//afrer each run of the agc_calc function, the AGC struct is being updated:
//agc_setting.bias = m_bias_curr_value;
//agc_setting.gain = m_gains_arr[m_gain_curr_location];
//agc_setting.led  = m_current_curr_value;

static bool         m_on_wrist;                                                         /**< TRUE if Slice is currently being worn on the wrist, FALSE otherwise. */

static uint8_t      m_gain_curr_location;                                               /**< Current gain */
static uint8_t      m_current_curr_value;    
static int          m_led_current_rate;                                                 /**< Counter used to control AGC adjustment frequency */
static int          m_led_old_current_rate;                                             /**< Counter used to control AGC adjustment frequency */
static bool         m_on_wrist_inner;                                                   /**< Previous on-wrist state */

void slice_driver_get_onwrist_value(bool onwrist_value)
{
    m_on_wrist = onwrist_value;
}

void slice_driver_agc_calc(int32_t ppg_curr, int32_t amb_curr, float snr)
{
//    NRF_LOG("slice_driver_agc_calc\n");
    // debug_pin_uint32(ppg_curr);
    
    // In case slice is off hand -> LED is off
    if (!m_on_wrist)
    {
        m_current_curr_value = SLICE_AGC_LED_CURRENT_MIN_VALUE;
        m_on_wrist_inner = false;
        return;
    }
    
    // In case slice is on hand -> LED is on
    if ((m_on_wrist) && (!m_on_wrist_inner))
    {
        m_current_curr_value = SLICE_AGC_LED_CURRENT_MAX_VALUE;
        //m_bias_curr_value = -15;
        m_gain_curr_location = GAIN_DEFAULT_INDEX;
        m_on_wrist_inner = true;
        return;
    }
    else    // On wrist
    {
        if (ppg_curr>=0)
        {
            /*Updating rate 16Hz*/
            if (ppg_curr >= PPG_MAX)        // Saturation
            {
                if (m_gain_curr_location > 0) /*it means 50k*/ 
                {
                    m_gain_curr_location--;
                }
                else if (m_current_curr_value > SLICE_AGC_LED_CURRENT_LOW_VALUE)
                    m_current_curr_value--;
            } 
            /*Updating rate 4Hz*/
            else if (ppg_curr<1000000)
            {
                m_led_current_rate++;
                if (m_led_current_rate == LED_CURRENT_RATE_COUNT)
                {
                    m_led_current_rate = 0;
                    if ((m_gain_curr_location < 1) && (ppg_curr < 350000))/*it means 200k*/ 
                        m_gain_curr_location++; 
                                    else if (m_current_curr_value < SLICE_AGC_LED_CURRENT_MAX_VALUE) 
                        m_current_curr_value++;
                    
                }
            }
        }
        else //negative PPG
        {
            /*Updating rate 16Hz*/
            if (ppg_curr <= -PPG_MAX)        // Saturation
            {
                if (m_gain_curr_location > 0) /*it means 50k*/ 
                {
                    m_gain_curr_location--;
                }
                else if (m_current_curr_value < SLICE_AGC_LED_CURRENT_MAX_VALUE) 
                    m_current_curr_value++;
            } 
            /*Updating rate 4Hz*/
            else if (ppg_curr>-1000000)
            {
                m_led_current_rate++;
                if (m_led_current_rate == LED_CURRENT_RATE_COUNT)
                {
                    m_led_current_rate = 0;
                    if ((m_gain_curr_location < 1) && (ppg_curr > -350000))/*it means 200k*/ 
                        m_gain_curr_location++; 
                }
            }
        }
            
    }
}

void slice_driver_set_afe_current(void)
{
    if(m_led_old_current_rate != m_led_current_rate)
    {
//        //print log
//        NRF_LOG("slice_driver_set_afe_current: ");
//        NRF_LOG_HEX_CHAR(m_led_current_rate);
//        NRF_LOG("\r\n");
        
        //disable
        AFE4404_Disable_Read();
        //DOF1 Current
        AFE4404_Reg_Write(0x22, m_led_current_rate);  
        //enable
        AFE4404_Enable_Read();    
        
        //get old current
        m_led_old_current_rate = m_led_current_rate;
    }
}

