
#include "nrf.h"                        // Device header
#include "app_twi.h"
//#include "slice_drv_mma8451.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
//#include "slice_drv_twi.h"
#include "slice_cfg.h"
#include "nrf_drv_twi.h"
#include "mma8451.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "twi.h"
 

static uint8_t m_mmm8451_buffer[6];


static I2C_TransferSeq_TypeDef m_seq;

static void mma8451_irq_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
    //TODO
}   






static void mma8451_write_reg(uint8_t reg , uint8_t val)
{ 
    I2C_TransferSeq_TypeDef    seq;
    uint8_t i2c_write_data[3];
    uint8_t i2c_read_data[1];

    seq.addr  = MMA8451_ADDR;
    seq.flags = I2C_FLAG_WRITE;
    
    /* Select register and data to write */
    i2c_write_data[0] = reg;
    i2c_write_data[1] = val;
    
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 3;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = 0;
	
		twi_transfer(&seq);
}

static uint8_t mma8451_read_reg(uint8_t reg)
{
    I2C_TransferSeq_TypeDef    seq;
		
    uint8_t i2c_write_data[2];
    uint8_t i2c_read_data[1];

    seq.addr  = MMA8451_ADDR;
    seq.flags = I2C_FLAG_READ;
    //NRF_LOG("mma8451 addr = ");
		//printf("%x", MMA8451_ADDR);
    //Select register and data to write
    i2c_write_data[0] = reg;
    i2c_write_data[1] = 0;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 1;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = 1;
    
		ret_code_t err_code;
		twi_transfer(&seq);

    return i2c_read_data[0];
	
}


void slice_drv_mma8451_init(void){
	volatile uint8_t x = 5;
	ret_code_t err_code = NRF_SUCCESS;
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
	
	if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
		
	//MMA8451 IRQ1 init
    err_code = nrf_drv_gpiote_in_init(MEMS_IRQ1_PIN_NO, &in_config, mma8451_irq_in_pin_handler);
    APP_ERROR_CHECK(err_code);
    
    //do not use interrupt , enable it if needed
    //nrf_drv_gpiote_in_event_enable(MEMS_IRQ1_PIN_NO, true);
    //MMA8451 IRQ2 init
    err_code = nrf_drv_gpiote_in_init(MEMS_IRQ2_PIN_NO, &in_config, mma8451_irq_in_pin_handler);
    APP_ERROR_CHECK(err_code);

		mma8451_write_reg(MMA8451_REG_CTRL,0x20);  // Set active and otput data rate 50 Hz
    mma8451_write_reg(MMA8451_REG_XYZ_DATA_CFG,0x02);
    mma8451_write_reg(MMA8451_REG_CTRL,0x21);  // Set active and otput data rate 50 Hz
		//mma8451_write_reg(MMA8451_REG_CTRL,0x23);  // Set active and otput data rate 50 Hz
    x = mma8451_read_reg(MMA8451_REG_XYZ_DATA_CFG);
    x  = mma8451_read_reg(MMA8451_REG_CTRL);
    x = mma8451_read_reg(MMA8451_REG_XYZ_DATA_CFG);
}



static void mma8451_data_proccessing (uint8_t* m_mmm8451_buffer, mma_sensor_data_t *data)
{
	
    sbu_t        sbu;        // Unit 
    uint8_t     i;          // for test purpose only 
	/*
    sbu.vbyte[1] = m_mmm8451_buffer[0]; // X MSB
    sbu.vbyte[0] = m_mmm8451_buffer[1]; // X LSB
    data->x = sbu.vbyte[1]; // x word
    sbu.vbyte[1] = m_mmm8451_buffer[2]; // y MSB
    sbu.vbyte[0] = m_mmm8451_buffer[3]; // y LSB
		data->y = sbu.vbyte[1]; // y word
    sbu.vbyte[1] = m_mmm8451_buffer[4]; // z MSB
    sbu.vbyte[0] = m_mmm8451_buffer[5]; // z LSB
    data->z = sbu.vbyte[1]; // z word
    i++;
		*/
		
		data->x = (m_mmm8451_buffer[0] << 8) | m_mmm8451_buffer[1];
		if(data->x & 0x8000) data->x-=65536;

		data->y= (m_mmm8451_buffer[2] << 8) | m_mmm8451_buffer[3];
		if(data->y & 0x8000) data->y-=65536;

		data->z = (m_mmm8451_buffer[4] << 8) | m_mmm8451_buffer[5];
		if(data->z & 0x8000) data->z-=65536;
		
}

void slice_drv_mma8451_read(mma_sensor_data_t *data)
{
    I2C_TransferSeq_TypeDef    seq;
    uint8_t i2c_write_data[2];
    
    seq.addr  = MMA8451_ADDR;
    seq.flags = I2C_FLAG_READ;
    //Select register and data to write
    i2c_write_data[0] = MMA8451_REG_XOUT_H;
    i2c_write_data[1] = 0;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len  = 1;
    seq.buf[1].data = m_mmm8451_buffer;
    seq.buf[1].len  = 6;
		
		twi_transfer(&seq);
			
    //NRF_LOG_INFO("%x %x %x ", seq.buf[0].data[0], seq.buf[0].data[1], seq.buf[1].data[0]);
    mma8451_data_proccessing(m_mmm8451_buffer,data);

}

void slice_drv_mma8451_get_id(uint8_t *id)
{
	*id = mma8451_read_reg(0x0D); // it much return 0x1A, or it fail to read
}
