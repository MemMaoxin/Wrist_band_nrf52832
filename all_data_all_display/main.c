/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
//#include "ble_hrs.h"
#include "ble_sensor.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#include "afe4409.h"
#include "nrf_drv_gpiote.h"
#include "mma8451.h"
#include "afe4409.h"
#include "slice_cfg.h"
#include "twi.h"
#include "nrf_delay.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_saadc.h"
#include "slice_sm_button.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "ble_dfu.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "ble_nus.h"

#define DEVICE_NAME                         "ISL_WB_DFU_1"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
//#define APP_ADV_DURATION										18000
#define APP_ADV_DURATION                    0                                  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(40)                   	/**< Heart rate measurement interval (ticks). */
#define SENSOR_DATA_INTERVAL    			      APP_TIMER_TICKS(40)                    	/**< SENSOR Data interval (ticks). */
#define SWITCH_INTERVAL											APP_TIMER_TICKS(40)											/**< SWITCH INTERVAL (ticks). */
#define MMA8451_INTERVAL										APP_TIMER_TICKS(4)											/**< MMA8451 interval (ticks). */
#define AFE4900_INTERVAL										APP_TIMER_TICKS(4)											/**< AFE4900 interval (ticks). */
#define SAADC_INTERVAL											APP_TIMER_TICKS(4)											/**< SAADC interval (ticks). */
#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(120)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(20, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(150, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        5                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      1                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED


#define APP_BUTTON_COUNT  (sizeof(m_app_buttons) / sizeof(m_app_buttons[0]))
static app_button_cfg_t m_app_buttons[] =
{
    { 24, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, slice_sm_button_on_app_button_evt },
    
};

BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_sensor_data_timer_id);                              /**< RR interval timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */
APP_TIMER_DEF(m_afe4900_timer_id);
APP_TIMER_DEF(m_mma8451_timer_id);
APP_TIMER_DEF(m_saadc_timer_id);
APP_TIMER_DEF(m_switch_timer_id);


static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool     m_sensor_data_enabled = false;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */
static bool     m_mma8451_enabled = false;
static bool     m_afe4900_enabled = false;
static bool     m_saadc_enabled = false;
static bool     m_sensor_contact_enabled = false;
static bool     m_switch_enabled = false;
static bool     start = false;
static uint16_t cnt = 0;

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
/*
mma_sensor_data_t fakeData[10] = {{0, 0, 0}, 
																	{200, 200, 200},
																	{500, 500, 500},
																	{6000, 6000, 6000},
																	{1000, 1000, 1000},
																	{0, 0, 0}, 
																	{-2000, -2000, -2000},
																	{500, 500, 500},
																	{400, 400, 400},
																	{0, 0, 0} };
*/
static int fakeData = 0;
static int data_loop = 0;
static bool arrFull = false;

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE},
		{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}
};

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static void start_all(){
	if(start == false){
		m_mma8451_enabled = !m_mma8451_enabled;
		m_afe4900_enabled = !m_afe4900_enabled;
		m_sensor_data_enabled = !m_sensor_data_enabled;
		m_saadc_enabled = !m_saadc_enabled;
		m_sensor_contact_enabled = !m_sensor_contact_enabled;
		m_switch_enabled = !m_switch_enabled;
		nrf_drv_gpiote_in_event_enable(AFE_IRQ_PIN_NO, true);
		AFE4409_Disable_SWPDN();
		nrf_gpio_pin_toggle(18);
		start = true;
	}else{
		m_mma8451_enabled = !m_mma8451_enabled;
		m_afe4900_enabled = !m_afe4900_enabled;
		m_sensor_data_enabled = !m_sensor_data_enabled;
		m_saadc_enabled = !m_saadc_enabled;
		m_sensor_contact_enabled = !m_sensor_contact_enabled;
		m_switch_enabled = !m_switch_enabled;
		AFE4409_Enable_SWPDN();
		nrf_gpio_pin_toggle(18);
		start = false;
	}
}
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
			
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
				if(p_evt->params.rx_data.length == 2){
					start_all();
				}else if(p_evt->params.rx_data.length == 3 && m_sensor_data_enabled == false){
					
				}
				
				
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_INFO("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}

static uint8_t dataLogger = 1;
static uint8_t afe_state = 0;
static int flash_writing = 0;
static uint8_t data[150];
static int mma_i = 0;
static int afe_i = 0;
static int adc_i = 0;
mma_sensor_data_t mmadata [10];
static uint16_t adcData[10];
#define SAMPLES_BUFFER_LEN 10
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_BUFFER_LEN];
static uint32_t       m_adc_evt_counter;
static bool current_mode = true;
static bool previous_mode = true;
static int varTicks = 40;
static int counter = 40;
static int recordx[2];
static int recordy[2];
static int recordz[2];
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */



void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
	  float  val;  //保存SAADC采样数据计算的实际电压值
	  
	  if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        //设置好缓存，为下一次采样准备
		    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_BUFFER_LEN);
        APP_ERROR_CHECK(err_code);
        int i;
			  //串口输出ADC采样值。
				//NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_BUFFER_LEN; i++)
        {
            //如果直接输出采样结果，使用这个代码
					//NRF_LOG_INFO("Sample value: %d    ", p_event->data.done.p_buffer[i]);
					adcData[i] = p_event->data.done.p_buffer[i];
					
			      //串口输出采样值计算得到的电压值。电压值 = 采样值 * 3.6 /2^10
			      val = p_event->data.done.p_buffer[i] * 3.6 /1024;	
			      //NRF_LOG_INFO("Voltage = %.3fV", val);
        }
        //事件次数加1
				
		    m_adc_evt_counter++;
    }
}
	
static void saadc_init(void)
{
    ret_code_t err_code;
	  //定义ADC通道配置结构体，并使用单端采样配置宏初始化，
	  //NRF_SAADC_INPUT_AIN2是使用的模拟输入通道
    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    //初始化SAADC，注册事件回调函数。
	  err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);
    //初始化SAADC通道0
    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
		
		//配置缓存1，将缓存1地址赋值给SAADC驱动程序中的控制块m_cb的一级缓存指针
	  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_BUFFER_LEN);
    APP_ERROR_CHECK(err_code);
    //配置缓存2，将缓存1地址赋值给SAADC驱动程序中的控制块m_cb的二级缓存指针
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_BUFFER_LEN);
    APP_ERROR_CHECK(err_code);
}



void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt1 = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(p_context);
		
    heart_rate = 0;
		
    cnt1++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
		
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    //m_rr_interval_enabled = ((cnt1 % 3) != 0);
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_data_timeout_handler(void * p_context)
{
		//Pass the full set data to ble_hrs buffer
    UNUSED_PARAMETER(p_context);
    if (m_sensor_data_enabled)
    {
			for(int i = 0; i < 10; i++){
				ble_hrs_data_add(&m_hrs, mmadata[i].x);
				ble_hrs_data_add(&m_hrs, mmadata[i].y);
				ble_hrs_data_add(&m_hrs, mmadata[i].z);
				//NRF_LOG_INFO("mmadata: %x %x %x ", mmadata[i].x, mmadata[i].y, mmadata[i].z);
				NRF_LOG_INFO("mmadata: %d %d %d ", mmadata[i].x, mmadata[i].y, mmadata[i].z);
				ble_hrs_data_add(&m_hrs, data[i*15 + 0] << 8 | data[i*15 + 1]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 2] << 8 | data[i*15 + 3]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 4] << 8 | data[i*15 + 5]);
				NRF_LOG_INFO("ppg[0] to ppg[5]: %x %x %x %x %x %x", data[i*15 + 0], data[i*15 + 1], data[i*15 + 2], data[i*15 + 3], data[i*15 + 4], data[i*15 + 5]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 6] << 8 | data[i*15 + 7]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 8] << 8 | data[i*15 + 9]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 10] << 8 | data[i*15 + 11]);
				NRF_LOG_INFO("ppg[6] to ppg[11]: %x %x %x %x %x %x", data[i*15 + 6], data[i*15 + 7], data[i*15 + 8], data[i*15 + 9], data[i*15 + 10], data[i*15 + 11]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 12] << 8 | data[i*15 +13]);
				ble_hrs_data_add(&m_hrs, data[i*15 + 14] << 8 | 0xff);
				ble_hrs_data_add(&m_hrs, adcData[i]);
				NRF_LOG_INFO("ecg[0] to ecg[2] + 0xff + ps: %x %x %x %x %d ", data[i*15 + 12], data[i*15 +13], data[i*15 + 14], 0xff, adcData[i]);
				}
			}
				cnt++;
				//NRF_LOG_INFO("Total Package: %d", cnt);
				//NRF_LOG_INFO("motion");

				
		
				
}



/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void saadc_interval_timeout_handler(void * p_context){
	UNUSED_PARAMETER(p_context);
	
	if(m_saadc_enabled){
		nrfx_saadc_sample();
	}
}
static void mma8451_interval_timeout_handler(void * p_context){
	UNUSED_PARAMETER(p_context);
	
	if(m_mma8451_enabled){
		mma_sensor_data_t tempdata;
		slice_drv_mma8451_read(&tempdata);
		mmadata[mma_i].x = tempdata.x;
		mmadata[mma_i].y = tempdata.y;
		mmadata[mma_i].z = tempdata.z;
		mma_i++;
		if(mma_i >= 10){
			mma_i = 0;
		}
	}
}
static void sensor_contact_detected_timeout_handler(void * p_context)
{

    UNUSED_PARAMETER(p_context);
		recordx[1] = (mmadata[9].x + mmadata[0].x) / 2;
		recordy[1] = (mmadata[9].y + mmadata[0].y) / 2;
		recordz[1] = (mmadata[9].z + mmadata[0].z) / 2;
		if(m_sensor_contact_enabled){
			//if(mmadata[9].x > 2000){
			//if((abs(mmadata[9].x - mmadata[0].x) > 50) || (abs(mmadata[9].y - mmadata[0].y) > 50) || (abs(mmadata[9].z - mmadata[0].z) > 50))	{
			if((abs(recordx[1] - recordx[0])) > 50 || (abs(recordy[1] - recordy[0])) > 50 || (abs(recordz[1] - recordz[0])) > 50){
				previous_mode = current_mode;
				current_mode = true;
			}else{
				previous_mode = current_mode;
				current_mode = true;
			}
		}else{
			
		}
	  recordx[0] = recordx[1];
		recordy[0] = recordy[1];
		recordz[0] = recordz[1];
    ble_hrs_sensor_contact_detected_update(&m_hrs, current_mode);
}


static void afe4900_interval_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);

		if(m_afe4900_enabled){
			uint8_t *buffer;
				
			buffer = &data[15*afe_i];
			AFE4409_volReg_Read_Data(buffer); //data will store in buffer[0] to buffer[11]
			AFE4409_EGC_volReg_Read_Data(buffer);
			afe_i++;
			if(afe_i >= 10){
				afe_i = 0;
			}
		}
}

static void switch_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);
		if(m_switch_enabled){
		if(previous_mode == true && current_mode == false){
			m_saadc_enabled = true;
			m_afe4900_enabled = false;
			AFE4409_Enable_SWPDN();
			varTicks = 2000;
		}else if(previous_mode == false && current_mode == true){
			m_saadc_enabled = false;
			m_afe4900_enabled = true;
			nrf_drv_gpiote_in_event_enable(AFE_IRQ_PIN_NO, true);
			AFE4409_Disable_SWPDN();
			varTicks = 2000;
		}else {
			if(counter >= varTicks){
				if(varTicks < 2000){
					varTicks = varTicks * 2;
				}
				NRF_LOG_INFO("double");
				m_sensor_contact_enabled = true;
				counter = 40;
			}else{
				NRF_LOG_INFO("%d", counter);
				m_sensor_contact_enabled = false;
				counter += 40;
			}
		}
	}
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */


static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_data_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_data_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_mma8451_timer_id, 
																APP_TIMER_MODE_REPEATED, 
																mma8451_interval_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_afe4900_timer_id,
																APP_TIMER_MODE_REPEATED,
																afe4900_interval_timeout_handler);
																
		err_code = app_timer_create(&m_saadc_timer_id,
																APP_TIMER_MODE_REPEATED,
																saadc_interval_timeout_handler);
																
		err_code = app_timer_create(&m_switch_timer_id,
																APP_TIMER_MODE_REPEATED,
																switch_timeout_handler);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//注册优先级为0的应用程序关闭处理程序
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

//SoftDevice状态监视者
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
			  //表明Softdevice在复位之前已经禁用，告之bootloader启动时应跳过CRC
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //进入system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

//注册SoftDevice状态监视者，用于SoftDevice状态改变或者即将改变时接收SoftDevice事件
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}
//断开当前连接，设备准备进入bootloader之前，需要先断开连接
static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    //断开当前连接
    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        //该事件指示设备正在准备进入bootloader
			  case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            //防止设备在断开连接时广播
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
					  //连接断开后设备不自动进行广播
            config.ble_adv_on_disconnect_disabled = true;
					  //修改广播配置
            ble_advertising_modes_config_set(&m_advertising, &config);

			      //断开当前已经连接的所有其他绑定设备。在设备固件更新成功（或中止）后，需要在启动时接收服务更改指示
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }
        //该事件指示函数返回后设备即进入bootloader
        case BLE_DFU_EVT_BOOTLOADER_ENTER:
				    //如果应用程序有数据需要保存到Flash，通过app_shutdown_handler返回flase以延迟复位，从而保证数据正确写入到Flash
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;
        //该事件指示进入bootloader失败
        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
					  //进入bootloader失败，应用程序需要采取纠正措施来处理问题
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            break;
        //该事件指示发送响应失败
        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            //发送响应失败，应用程序需要采取纠正措施来处理问题
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
		ble_dfu_buttonless_init_t dfus_init = {0};
		ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
		dfus_init.evt_handler = ble_dfu_evt_handler;
		err_code = ble_dfu_buttonless_init(&dfus_init);
		APP_ERROR_CHECK(err_code);
		
		memset(&nus_init, 0, sizeof(nus_init));
		
		nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);
										NRF_LOG_INFO("%c%c%c", data_array[0], data_array[1], data_array[2]);
                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = 13,
        .tx_pin_no    = 19,
        .rts_pin_no   = NULL,
        .cts_pin_no   = NULL,
        .flow_control = UART_HWFC,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
		/*
    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
		*/
}



/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_data_timer_id, SENSOR_DATA_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_start(m_mma8451_timer_id, MMA8451_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_afe4900_timer_id, AFE4900_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_saadc_timer_id, SAADC_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_start(m_switch_timer_id, SWITCH_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


static void in_pin_handler(slice_sm_button_evt_t * p_evt)
{
		ret_code_t err_code;
		// event of pin 24 (Button)
		
    if(p_evt->pin_no == 24)
    {
			switch (p_evt->evt_type){
				case SLICE_SM_BUTTON_PRESS_SINGLE: 
				
				// toggle pin 18 (LED)
				nrf_gpio_pin_toggle(18);
				// start receiving all data
				m_mma8451_enabled = !m_mma8451_enabled;
			
				if(m_mma8451_enabled){
					m_afe4900_enabled = true;
					m_sensor_data_enabled = true;
					m_saadc_enabled = true;
					m_sensor_contact_enabled = true;
					m_switch_enabled = true;
					nrf_drv_gpiote_in_event_enable(AFE_IRQ_PIN_NO, true);
					AFE4409_Disable_SWPDN();
				}else{
					//nrf_drv_gpiote_in_event_disable(AFE_IRQ_PIN_NO);
					m_afe4900_enabled = false;
					m_sensor_data_enabled = false;
					m_saadc_enabled = false;
					m_sensor_contact_enabled = false;
					AFE4409_Enable_SWPDN();
					m_switch_enabled = false;
				}
				break;
		
				case SLICE_SM_BUTTON_PRESS_VERYLONG: 
					NRF_LOG_INFO("Reset...");	
					for(int i = 0; i < 20; i++){
						nrf_gpio_pin_toggle(18);
						nrf_delay_ms(100);
					}
					NVIC_SystemReset();
					
					break;
                
        case SLICE_SM_BUTTON_PRESS_LONG:
          //NRF_LOG("SLICE_SM_BUTTON_PRESS_VERYLONG--SLICE_BUTTON_PIN_NO\r\n");
          break;
                
        default:
         //NRF_LOG("Unknown button event!\r\n");
          break;
    }
	}		
		
}
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;
		slice_sm_button_init_t button_init;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_button_init(m_app_buttons, APP_BUTTON_COUNT, APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);
		
		memset(&button_init, 0, sizeof(button_init));
		button_init.button_id   = 0;
    button_init.pin_no      = SLICE_BUTTON_PIN_NO;
    button_init.evt_handler = in_pin_handler;
		
		err_code = slice_sm_button_init(&button_init);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void afe_irq_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(afe_state == 1)
    {
        //counter++;    
        //AFE4409_Reg_Read_Data(m_afe44xx_spo2_data_buf);
        
        //slice_drv_afe4409_write_data_to_sdcard();
        #if 1
        //slice_drv_afe4409_read_data_to_buffer();
        //app_timer_start(m_4409_timer_id, CURRENT_TIME_INTERVAL, NULL);
        #else
        app_timer_start(m_4409_store_timer_id, CURRENT_TIME_INTERVAL, NULL);
        #endif
    }
}


/**@brief This function will irq handle for AFE4409
 */
static void afe_egc_irq_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(afe_state == 1)
    {
        //AFE4409_Reg_Read_Data(m_afe44xx_spo2_data_buf);
        
        //slice_drv_afe4409_write_data_to_sdcard();
       // slice_drv_afe4409_egc_write_data_to_sdcard();
        //afe4409_time_start();
    }
}




static void gpio_pin_init(void){
		ret_code_t err_code = NRF_SUCCESS;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
	
		nrf_gpio_cfg_output(ACC_EN_B_PIN_NO);
    nrf_gpio_pin_clear(ACC_EN_B_PIN_NO);
	
		nrf_gpio_cfg_output(BATTERY_MONITOR_CTRL_PIN_NO);
    nrf_gpio_pin_set(BATTERY_MONITOR_CTRL_PIN_NO);

		nrf_gpio_cfg_output(30);
    
    nrf_gpio_pin_clear(30); 
	
		nrf_gpio_cfg_output(VDD_LED_CTRL_PIN_NO);
    nrf_gpio_pin_clear(VDD_LED_CTRL_PIN_NO); 
   
		nrf_gpio_cfg_output(VDD_AFE_CTRL_PIN_NO);
    nrf_gpio_pin_clear(VDD_AFE_CTRL_PIN_NO);

    nrf_gpio_pin_set(VDD_AFE_CTRL_PIN_NO);       // AFE PS ON
    
    nrf_gpio_pin_set(VDD_LED_CTRL_PIN_NO);    //  
		
		if (!nrf_drv_gpiote_is_init())
    {
        nrf_drv_gpiote_init();
        
    }
		
		//nrf_gpio_cfg_input(AFE_CLK_PIN_NO,NRF_GPIO_PIN_NOPULL);
    err_code = nrf_drv_gpiote_in_init(AFE_IRQ_PIN_NO, &in_config, afe_irq_handle);
		APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(AFE_CLK_PIN_NO, &in_config, afe_egc_irq_handle);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(AFE_IRQ_PIN_NO, true);
		//nrf_drv_gpiote_in_event_enable(AFE_CLK_PIN_NO, true);
		//err_code = nrf_drv_gpiote_in_init(24, &in_config, in_pin_handler);
		//APP_ERROR_CHECK(err_code);
		//nrf_drv_gpiote_in_event_enable(24, true);

		#ifndef INTERNAL_CLOCK_4409
    afe4409_clock_out_init(AFE4409_FREQ_4MHZ);
		#endif
		
		
	}

	


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
		uart_init();
    timers_init();
		NRF_LOG_INFO("START!");
    
		buttons_leds_init(&erase_bonds);
		gpio_pin_init();
		NRF_LOG_INFO("init twi...");
		twi_init();
		NRF_LOG_INFO("twi setup");
	
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();
		
		slice_drv_mma8451_init();
		AFE4900_Init();
    saadc_init(); 
		NRF_LOG_INFO("Heart Rate Sensor example started.");
    application_timers_start();
    advertising_start(erase_bonds);
		
    // Enter main loop.
    for (;;)
    {
		   idle_state_handle();
    }
}


