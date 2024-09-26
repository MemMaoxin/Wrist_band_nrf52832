// TODO: Add file header / copyright notice

#ifndef _SLICE_CFG_H_
#define _SLICE_CFG_H_

#include "ble.h"

//#ifdef BOARD_PCA10040
#if 0

    #define SLICE_TEST_BUTTON_1_PIN_NO      13          /**< Generic test button 1 (Button 1 on the PCA10040 board). */
    #define SLICE_TEST_BUTTON_2_PIN_NO      14          /**< Generic test button 2 (Button 2 on the PCA10040 board). */
    #define SLICE_ERASE_BONDS_BUTTON_PIN_NO 15          /**< Button for erasing bonds (Button 3 on the PCA10040 board). */
    #define SLICE_BUTTON_PIN_NO             16          /**< Slice button (Button 4 on the PCA10040 board). */

    #define SLICE_LED_BLE_STATE_PIN_NO      17          /**< LED for indicating BLE state (LED 1 on the PCA10040 board). */
    #define SLICE_LED_WHITELIST_PIN_NO      18          /**< LED for indicating use of whitelist when advertising (LED 2 on the PCA10040 board). */
    #define SLICE_TEST_LED_3_PIN_NO         19          /**< Generic test LED 3 (LED 3 on the PCA10040 board). */
    #define SLICE_TEST_LED_4_PIN_NO         20          /**< Generic test LED 4 (LED 4 on the PCA10040 board). */
    
#ifndef NO_DISPLAY
        #define LCD_SCLK_PIN_NO             3           /**< LCD clock. */
        #define LCD_SDAT_PIN_NO             30          /**< LCD data. */
        #define LCD_DC_PIN_NO               4           /**< LCD data/command selection. */
        #define LCD_CSB_PIN_NO              29          /**< LCD chip selection. */
        #define LCD_RSTB_PIN_NO             2           /**< LCD chip reset. */
        #define VDD_LCD_CTRL_PIN_NO         15          /**< Display power supply ctrl*/
        #define VDD_AFE_CTRL_PIN_NO         11          /**< AFE 2.8V Ctrl*/
#endif

    #define VIBRATOR_PWM_PIN_NO             20          /**< Pin connected to vibrator device (using LED 4 on the PCA10040 board). */
    #define VIBRATOR_PIN_ACTIVE_STATE       0           /**< State of GPIO when vibrator is active. */

    #define BATTERY_CHARGER_STATUS_PIN_NO   12          /**< Battery charger status, Input, pull up */   
    
#else 

    //start slice board
    
    //#define SI114X      1                            
    //#define AFE4404     1
    #define AFE4409     1
		
    //for button in the main.c
    #define SLICE_BUTTON_PIN_NO             24          /**< GPIO pin number for Slice button. */
    #define SLICE_KEY2_PIN_NO               16          /**< GPIO pin number for Slice button. */

    #define MODE_LED_PIN_NO             18          /**< Vibrator motor ctrl/pwm (pwm is optional)*/

    //for lcd in the slice_drv_display.c
    #define LCD_SCLK_PIN_NO                  3          /**< LCD clock. */
    #define LCD_SDAT_PIN_NO                 30          /**< LCD data. */
    #define LCD_DC_PIN_NO                    4          /**< LCD data/command selection. */
    #define LCD_CSB_PIN_NO                  29          /**< LCD chip selection. */
    #define LCD_RSTB_PIN_NO                  2          /**< LCD chip reset. */

    //for vibrator in the slice_sm_vibrator.c
    #define VIBRATOR_PWM_PIN_NO             18          /**< Vibrator motor ctrl/pwm (pwm is optional)*/
    #define VIBRATOR_PIN_ACTIVE_STATE       1           /**< State of GPIO when vibrator is active. */

    #define VDD_AFE_CTRL_PIN_NO             11          /**< AFE 2.8V Ctrl*/
    #define VDD_LED_CTRL_PIN_NO             17          /**< LED's 5V PS control*/
    #define VDD_LCD_CTRL_PIN_NO             15          /**< Display power supply ctrl*/

    #define I2C_SCL_PIN_NO                  6           /**< SCL signal pin*/
    #define I2C_SDA_PIN_NO                  5           /**< SDA signal pin*/

    #define SD_MISO_PIN_NO                  30          /**< SPI MISO (DIN), use with SD card. */
        
    #define MEMS_IRQ1_PIN_NO                26          /**< IRQ1, Input, IRQ, wake up, pull up */
    #define MEMS_IRQ2_PIN_NO                27          /**< IRQ2, Input, IRQ, wake up, pull up */

    #define BATTERY_MONITOR_CTRL_PIN_NO     23          /**< Battery monitor Switch */
    #define BATTERY_CHARGER_STATUS_PIN_NO   12          /**< Battery charger status, Input, pull up */   
    #define BATTERY_VOLTAGE_PIN_N0         28          /**< Analog input, AIN4, battery voltage monitor */

    #define TEST_POINT_2_PIN_NO             17          /**< TP2 */
    #define TEST_POINT_3_PIN_NO             19          /**< TP3 */
    #define TEST_POINT_4_PIN_NO             16          /**< TP4 input */
    #define TEST_POINT_6_PIN_NO             23          /**< TP6 */
     
    #define UART_TX_PIN_NO                  19          /**< TP4 Output */
    #define UART_RX_PIN_NO                  13          /**< TP3 Input */ //test
    #define ANI6            30 /*ADC IN 6 */
   
    #define DCIN_DET_B_PIN_NO               31          /**< DCIN detect pin*/

    #define ACC_EN_B_PIN_NO                 20          /**< ACC EN pin*/
    
    #define SI114X_IRQ_PIN_NO               14          /**< IRQ9, Input, IRQ, wake up, pull up */
    #define SI114X_RST_PIN_NO                7          /**< si114x rst output, pull up */
    #define SI114X_CLK_PIN_NO                8          /**< si114x clk output, pull up */
                                     
    #if defined(SI114X)                                
    #define AFE_RST_PIN_NO      SI114X_RST_PIN_NO       /**< AFE reset */
    #define AFE_CLK_PIN_NO      SI114X_CLK_PIN_NO       /**< AFE Clk (AFE4404 only, optional) */
    #define AFE_IRQ_PIN_NO      SI114X_IRQ_PIN_NO       /**< 9 Input, wake up, Interrupt on change, pull up */ 
    #elsif
		#define AFE_RST_PIN_NO      7       /**< AFE reset */
    #define AFE_CLK_PIN_NO      8       /**< AFE Clk (AFE4404 only, optional) */
    #define AFE_IRQ_PIN_NO      14       /**< 9 Input, wake up, Interrupt on change, pull up */ 
  
		#else
    #define AFE_RST_PIN_NO                   7          /**< AFE reset */
    #define AFE_CLK_PIN_NO                   8          /**< AFE Clk (AFE4404 only, optional) */
    #define AFE_IRQ_PIN_NO                  14          /**< 9 Input, wake up, Interrupt on change, pull up */    
		//#define AFE_IRQ_PIN_NO									17
    #endif
    
    // Low frequency clock source to be used by the SoftDevice
    #define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                     .rc_ctiv       = 0,                                \
                                     .rc_temp_ctiv  = 0,                                \
                                     .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
    //end slice board
#endif

                                     
#define APP_TIMER_PRESCALER                 0           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE             8           /**< Size of timer operation queues. */

#define SLICE_DATA_SYNC_SERVICE_UUID        0x1530      /**< The UUID of the Data Sync Service. */
#define SLICE_DATA_SYNC_CTRL_PT_UUID        0x1531      /**< The UUID of the Data Sync Service Control Point. */
#define SLICE_DATA_SYNC_PKT_CHAR_UUID       0x1532      /**< The UUID of the Data Sync Service Packet Characteristic. */
#define SLICE_DATA_SYNC_REV_CHAR_UUID       0x1533      /**< The UUID of the Data Sync Service Revision Characteristic. */
#define SLICE_DATA_STREAM_SERVICE_UUID      0x1540      /**< The UUID of the Data Stream Service. */
#define SLICE_DATA_STREAM_PKT_CHAR_UUID     0x1541      /**< The UUID of the Data Stream Service Packet Characteristic. */
#define SLICE_DATA_STREAM_REV_CHAR_UUID     0x1542      /**< The UUID of the Data Stream Service Revision Characteristic. */
#define SLICE_CONFIG_SERVICE_UUID           0x1550      /**< The UUID of the Config Service. */
#define SLICE_CONFIG_CTRL_PT_UUID           0x1551      /**< The UUID of the Config Service Control Point. */
#define SLICE_CONFIG_CONN_PARAMS_UUID       0x1552      /**< The UUID of the Config Current Connection Parameters Characteristic. */

#define DATA_SYNC_SRV_REV_MAJOR             0x00                                                            /** Data Sync Major revision number to be exposed. */
#define DATA_SYNC_SRV_REV_MINOR             0x01                                                            /** Data Sync Minor revision number to be exposed. */
#define DATA_SYNC_SRV_REVISION              ((DATA_SYNC_SRV_REV_MAJOR << 8) | DATA_SYNC_SRV_REV_MINOR)      /** Data Sync Revision number to be exposed. Combined of major and minor versions. */

#define DATA_STREAM_SRV_REV_MAJOR           0x00                                                            /** Data Sync Major revision number to be exposed. */
#define DATA_STREAM_SRV_REV_MINOR           0x01                                                            /** Data Sync Minor revision number to be exposed. */
#define DATA_STREAM_SRV_REVISION            ((DATA_STREAM_SRV_REV_MAJOR << 8) | DATA_STREAM_SRV_REV_MINOR)  /** Data Sync Revision number to be exposed. Combined of major and minor versions. */

#define CONFIG_SRV_REV_MAJOR                0x00                                                            /** Data Sync Major revision number to be exposed. */
#define CONFIG_SRV_REV_MINOR                0x01                                                            /** Data Sync Minor revision number to be exposed. */
#define CONFIG_SRV_REVISION                 ((CONFIG_SRV_REV_MAJOR << 8) | CONFIG_SRV_REV_MINOR)            /** Data Sync Revision number to be exposed. Combined of major and minor versions. */

#ifdef NO_ENCRYPTION
    #define SEC_MODE_SET_PROTECTED          BLE_GAP_CONN_SEC_MODE_SET_OPEN                                  /**< Define for selecting protected access to a characteristic. */
#else
    #define SEC_MODE_SET_PROTECTED          BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM                         /**< Define for selecting protected access to a characteristic. */
#endif
#define SEC_MODE_SET_OPEN                   BLE_GAP_CONN_SEC_MODE_SET_OPEN                                  /**< Define for selecting open access to a characteristic. */
#define SEC_MODE_SET_NO_ACCESS              BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS                             /**< Define for selecting no access to a characteristic. */

#define FLASH_STORAGE_PRIORITY_PSTORAGE     0xFE        /**< Flash storage priority for the pstorage. This is a trick to make pstorage and fstorage coexist. NOTE: It must be removed when pstorage is removed" */
#define FLASH_STORAGE_PRIORITY_CONFIG       0xFD        /**< Flash storage priority for the config module. */
#define FLASH_STORAGE_PRIORITY_MEAS_DATA    0xFC        /**< Flash storage priority for the measurement data storage. */
#define FLASH_STORAGE_PRIORITY_SENOR_DATA   0xFB        /**< Flash storage priority for the senor data storage. */
#define FLASH_STORAGE_PRIORITY_CONFIG_TEMP  0x01        /**< Flash storage priority for the temporary flash block in the config module. */

#define SLICE_FLASH_PAGE_SIZE               4096        /**< Flash page size for the nRF52. */

#define FLASH_MAGIC_CONFIG                  0xA55AB66BU /**< Bit pattern for identifying a config block. */
#define FLASH_MAGIC_DATA_UNCOMPRESSED       0x5AA56BB6U /**< Bit pattern for identifying a uncompressed data block. */
#define FLASH_MAGIC_DATA_COMPRESSED         0x6BB65AA5U /**< Bit pattern for identifying a compressed data block. */

#define NUM_MEAS_DATA_FLASH_PAGES           10          /**< Number of flash pages to allocate for storing measurement data. TODO: Set proper value (allocate all unused space). */

extern const ble_uuid128_t g_mio_base_uuid128;

#endif
