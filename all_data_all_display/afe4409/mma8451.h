

#ifndef MMA8451_H__
#define MMA8451_H__


#ifndef APP_TWI_H__
    #include "app_twi.h"
#endif
#ifndef NRF_DRV_GPIOTE__
    #include "nrf_drv_gpiote.h"
#endif    



#include "nrf_drv_twi.h"

typedef struct 
{
        //uint8_t x;
        //uint8_t y;
        //uint8_t z;
				signed short x;
        signed short y;
        signed short z;
}mma_sensor_data_t;             /**< mma sensor data type. */


#ifndef _SBU_
#define _SBU_
typedef union 
{
    //uint16_t value;
		signed short value;
    //uint8_t vbyte[2];
} sbu_t;                        /**< sbu type. */    
#endif




// 0x38 is the MMA8451's address in the mbed Application Shield, it contains
// R/W bit and "nrf_drv_twi" (and consequently "app_twi") requires slave
#define MMA8451_ADDR        (0x38U >> 1) // Shift right because stuped shift left by app_twi.h
#define MMA8451_REG_STATUS          0x00U
#define MMA8451_REG_XOUT_H          0x01U
#define MMA8451_REG_XOUT_L          0x02U
#define MMA8451_REG_YOUT_H          0x03U
#define MMA8451_REG_YOUT_L          0x04U
#define MMA8451_REG_ZOUT_H          0x05U
#define MMA8451_REG_ZOUT_L          0x06U
#define MMA8451_REG_CTRL            0x2AU
#define MMA8451_REG_XYZ_DATA_CFG    0x0E


// The Status bit (3) set signals that the data ready 
// Applies to XOUT, YOUT, ZOUT .
#define MMA8451_DATA_IS_VALID(reg_data)  ((reg_data) & (8U))

#define MMA8451_WRITE(p_buffer, byte_cnt) \
    APP_TWI_WRITE(MMA8451_ADDR, p_buffer,   byte_cnt, 0)

#define MMA8451_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(MMA8451_ADDR, p_reg_addr, 1,  APP_TWI_NO_STOP), \
    APP_TWI_READ (MMA8451_ADDR, p_buffer,   byte_cnt, 0)


/**@brief This function will init mma8451 
 */
void slice_drv_mma8451_init(void);


/**@brief This function will read mma8451 x y x data
 */
void slice_drv_mma8451_read(mma_sensor_data_t *data);


/**@brief This function get mma8451 id
 */
void slice_drv_mma8451_get_id(uint8_t *id);

#endif // MMA8451_H__




