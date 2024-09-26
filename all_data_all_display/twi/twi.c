#include "twi.h"
#include "slice_cfg.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_twi.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"

#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static volatile bool m_xfer_done = false;
volatile static uint8_t m_twi_write_done = 0;                       /**< twi write done flag. */
volatile static uint8_t m_twi_read_done = 0;                        /**< twi read done flag. */

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
   switch (p_event->type)
   {
       case NRF_DRV_TWI_EVT_DONE:
					 //NRF_LOG_INFO("NRF_DRV_TWI_EVT_DONE");
					 if(p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX){
						 m_twi_write_done = 1;
					 }else{
						 m_twi_read_done = 1;
					 }
           m_xfer_done = true;
           break;
       default:
						NRF_LOG_INFO("rx/tx failed");
           break;
   }
}

void twi_init (void)
{
    uint32_t err_code;

    static nrf_drv_twi_config_t const config = {
       .scl                = I2C_SCL_PIN_NO,
       .sda                = I2C_SDA_PIN_NO,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority =  APP_IRQ_PRIORITY_HIGH,
			 .clear_bus_init     = false
    };

   err_code = nrf_drv_twi_init(&m_twi, &config, twi_handler, NULL);
   APP_ERROR_CHECK(err_code);

   nrf_drv_twi_enable(&m_twi); 
}

void twi_transfer(I2C_TransferSeq_TypeDef *seq){
	ret_code_t err_code;
	switch(seq->flags){
       
        case I2C_FLAG_WRITE:  
						m_twi_write_done = 0;
            m_xfer_done = false;
						err_code = nrf_drv_twi_tx(&m_twi, seq->addr, seq->buf[0].data, seq->buf[0].len, false);
						APP_ERROR_CHECK(err_code);
						while (m_xfer_done == false || !m_twi_write_done){}
						m_xfer_done = false;
						if( seq->buf[1].len > 0 && m_twi_read_done==0)
               {
									err_code = nrf_drv_twi_rx(&m_twi, seq->addr , seq->buf[1].data,  seq->buf[1].len);
									APP_ERROR_CHECK(err_code);
									while (m_xfer_done == false || !m_twi_read_done){}
               }
						break;
        case I2C_FLAG_WRITE_READ: 
        case I2C_FLAG_READ:
						m_twi_write_done = 0;
						m_twi_read_done = 0;
            m_xfer_done = false;
						err_code = nrf_drv_twi_tx(&m_twi, seq->addr, seq->buf[0].data, 1, true);
						APP_ERROR_CHECK(err_code);
						while (m_xfer_done == false || !m_twi_write_done){}
						
						if( seq->buf[1].len > 0 && m_twi_read_done==0)
               {
									m_xfer_done = false;
									err_code = nrf_drv_twi_rx(&m_twi, seq->addr , seq->buf[1].data,  seq->buf[1].len);
									APP_ERROR_CHECK(err_code);
									while (m_xfer_done == false || !m_twi_read_done){}
               }
            break;
        default:
            
            break;
        
    }
}