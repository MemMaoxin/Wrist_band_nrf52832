#ifndef SLICE_TWI_H__
#define SLICE_TWI_H__

#include "nrf_drv_twi.h"


typedef enum
{
  I2C_FLAG_WRITE_READ,  
  I2C_FLAG_WRITE,
  I2C_FLAG_READ,
}twi_transfer_t;            /**< twi transfer type. */


typedef struct 
{
  uint8_t *data;
  uint16_t len;
}i2c_transfer_buf_t;        /**< twi transfer buf. */


typedef enum
{
    i2cTransferDone,
    i2cTransferFaile,
}i2c_transfer_result_t;     /**< twi transfer result type. */


typedef struct 
{
   uint8_t addr;
   twi_transfer_t flags;
   i2c_transfer_buf_t buf[2]; /**< buf[0] for write, buf[1] for read */
}I2C_TransferSeq_TypeDef;   /**< twi transfer seq type. */

void twi_transfer(I2C_TransferSeq_TypeDef *seq);

void twi_init (void);
#endif