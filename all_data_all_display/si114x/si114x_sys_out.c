/**************************************************************************//**
 * @brief Implementation specific functions for HRM code
 * @version 4.2.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "nrf.h"   
#include "app_error.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "si114x_functions.h"
#include "si114x_sys_out.h"
#include "slice_drv_twi.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "nrf_drv_gpiote.h"     // Used by I/O IRQ
#include "slice_drv_mma8451.h"

#define SI1144_I2C_ADDR 0x5A
/*  I2C port configuration */
static si114x_port_config_t _handle;
/*  interrupt sequence counter  */
static uint16_t irqSequence = 0;
/*  interrupt queue size in bytes  */
#define	IRQ_QUEUE_SIZE	270
/*  interrupt queue data */
static u8 IrqQueue[IRQ_QUEUE_SIZE];
/*  interrupt queue current get index  */
static u16 irqQueueGetIndex = 0;
/*  interrupt queue current put index  */
static u16 irqQueuePutIndex = 0;

/*  Non-exported Function Prototypes  */
static s16 Si114x_i2c_smbus_write_byte_data(HANDLE si114x_handle, u8 address, u8 data, bool block);
static s16 Si114x_i2c_smbus_read_byte_data(HANDLE si114x_handle, u8 address, u8 *data, bool block);
static s16 Si114x_i2c_smbus_write_i2c_block_data(HANDLE si114x_handle, u8 address, u8 length, u8 const* values, bool block);
static s16 Si114x_i2c_smbus_read_i2c_block_data(HANDLE si114x_handle, u8 address, u8 length, u8* values, bool block);
static s16 Si114xIrqQueue_Put(si114x_IRQ_sample_t *samples);


/**************************************************************************//**
 * @brief Write to Si114x register
 *****************************************************************************/
int16_t Si114xWriteToRegister(HANDLE si114x_handle, uint8_t address, uint8_t data)
{
  return Si114x_i2c_smbus_write_byte_data(si114x_handle, address, data, true);
}

/**************************************************************************//**
 * @brief Read from Si114x register.
 *****************************************************************************/
int16_t Si114xReadFromRegister(HANDLE si114x_handle, uint8_t address)
{
  u8 data;
  Si114x_i2c_smbus_read_byte_data(si114x_handle, address, &data, true);
  return data;
}

/**************************************************************************//**
 * @brief block write to si114x
 *****************************************************************************/
int16_t Si114xBlockWrite(HANDLE si114x_handle,
                     uint8_t address, uint8_t length, uint8_t *values)
{
  return Si114x_i2c_smbus_write_i2c_block_data(si114x_handle,
                                               address,
                                               length,
                                               values,
                                               true);
}

/**************************************************************************//**
 * @brief Block read from Si114x.
 *****************************************************************************/
int16_t Si114xBlockRead(HANDLE si114x_handle,
                    uint8_t address, uint8_t length, uint8_t *values)
{
    return Si114x_i2c_smbus_read_i2c_block_data(si114x_handle,
                           address,    length,     values, true);
}

/**************************************************************************//**
 * @brief Disable GPIO interrupt for Si114x interrupt.
 *****************************************************************************/
void DisableSi114xInterrupt ()
{
    //TODO.......
    uint8_t irq = _handle.irqPin;
    nrf_drv_gpiote_in_event_disable(irq);
    //nrf_drv_gpiote_in_event_disable(_handle.irqPin);
	//GPIO_IntDisable(1<<_handle.irqPin);
}

/**************************************************************************//**
 * @brief Enable GPIO interrupt for Si114x.
 *****************************************************************************/
void EnableSi114xInterrupt ()
{
    uint8_t irq = _handle.irqPin;
    nrf_drv_gpiote_in_event_enable(irq,true);
    //TODO.......
    //nrf_drv_gpiote_in_event_enable(_handle.irqPin,true);
    /*
	if (GPIO_PinInGet(_handle.irqPort, _handle.irqPin) == 0)
		GPIO_IntSet(1<<_handle.irqPin);
	GPIO_IntEnable(1<<_handle.irqPin);
    */
}
int counter;
/**************************************************************************//**
 * @brief Main interrupt processing routine for Si114x.
 *****************************************************************************/
s16 Si114xProcessIrq(HANDLE si114x_handle, u16 timestamp)
{
    static int flag;
    volatile int x;
    
    mma_sensor_data_t mmadata; 
    u8 data_buffer[13];
    s16 error;
    si114x_IRQ_sample_t sample;
    if(flag)
       return 0;
    else
        flag =1;    
  
    irqSequence++;
    Si114x_i2c_smbus_read_i2c_block_data(si114x_handle, 0x21, 13, data_buffer, false);		// Grab all data registers
    Si114x_i2c_smbus_write_byte_data(si114x_handle, 0x21, data_buffer[0], false);      // Clear interrupts

    sample.sequence = irqSequence;       // sequence number
    sample.timestamp = timestamp;      // 16-bit Timestamp to record
    sample.pad= 0;
    sample.irqstat = data_buffer[0];        // 8-bit irq status
    sample.vis = (((u16)(data_buffer[2]) << 8) & 0xff00) | data_buffer[1];            // VIS
    sample.ir = (((u16)(data_buffer[4]) << 8) & 0xff00) | data_buffer[3];             // IR
    sample.ps1 = (((u16)(data_buffer[6]) << 8) & 0xff00) | data_buffer[5];            // PS1
    sample.ps2 = (((u16)(data_buffer[8]) << 8) & 0xff00) | data_buffer[7];            // PS2
    sample.ps3 = (((u16)(data_buffer[10]) << 8) & 0xff00) | data_buffer[9];            // PS3
    sample.aux = (((u16)(data_buffer[12]) << 8) & 0xff00) | data_buffer[11];;            // AUX
   
    slice_drv_mma8451_read(&mmadata);
    
    //enable it for debug, please remember to init uart: slice_uart_init
#if 0  
    uint8_t cr;
    uint32_t  i = 0;
    char string[64];
    sprintf(string,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \n ",counter,sample.vis,sample.ir,sample.ps1,sample.ps2,sample.ps3,sample.aux,mmadata.x,mmadata.y,mmadata.z);
    for(i = 0; i < strlen(string); i++) 
    {    
        while(app_uart_put(string[i]) != NRF_SUCCESS);
    }
    app_uart_get(&cr);
    counter++;
#endif
    
  //error = Si114xIrqQueue_Put(&sample);
  flag =0;
  return error;
}


/**************************************************************************//**
 * @brief Query number of entries in the interrupt queue.
 *****************************************************************************/
s16 Si114xIrqQueueNumentries(HANDLE si114x_handle)
{
  (void) si114x_handle;
  u16 runnerIndex = irqQueueGetIndex;
  s16 count=0;
  while (runnerIndex != irqQueuePutIndex)
  {
    runnerIndex++;
    count++;
    if(runnerIndex == IRQ_QUEUE_SIZE)
      runnerIndex = 0;
  }
  return (count/sizeof(si114x_IRQ_sample_t));

}

/**************************************************************************//**
 * @brief Get sample from the interrupt queue.
 *****************************************************************************/
s16 Si114xIrqQueue_Get(HANDLE si114x_handle, si114x_IRQ_sample_t *samples)
{
  (void) si114x_handle;
  int16_t error = 0;
  uint16_t i;
  int8_t *data = (int8_t *)samples;
  DisableSi114xInterrupt ();
  if (irqQueueGetIndex == irqQueuePutIndex)
    error = -1;
  else
  {
    for(i=0; i<sizeof(si114x_IRQ_sample_t); i++)
    {
      data[i] = IrqQueue[irqQueueGetIndex];
      irqQueueGetIndex++;
      if(irqQueueGetIndex == IRQ_QUEUE_SIZE)
        irqQueueGetIndex = 0;

    }

  }
  EnableSi114xInterrupt();
  return error;
}

/**************************************************************************//**
 * @brief Put new sample in the interrupt queue.
 *****************************************************************************/
static s16 Si114xIrqQueue_Put(si114x_IRQ_sample_t *samples)
{
  uint16_t i;
  u8 *data = (u8 *)samples;
  for(i=0; i<sizeof(si114x_IRQ_sample_t); i++)
  {
    IrqQueue[irqQueuePutIndex] = data[i];
    irqQueuePutIndex++;
    if(irqQueuePutIndex == IRQ_QUEUE_SIZE)
      irqQueuePutIndex = 0;
  }
  if (irqQueueGetIndex == irqQueuePutIndex)
  {
    irqQueueGetIndex += sizeof(si114x_IRQ_sample_t);			// if we have wrapped around then we must delete one sample
    return -1; //indicate to caller something bad happened
  }
  return 0;
}

/**************************************************************************//**
 * @brief Empty the interrupt queue.
 *****************************************************************************/
s16 Si114xIrqQueue_Clear(HANDLE si114x_handle)
{
  (void) si114x_handle;
  irqQueueGetIndex = 0;
  irqQueuePutIndex = 0;
  return 0;
}

/**************************************************************************//**
 * @brief Initialize low level handle and clear irq queue.
 *****************************************************************************/
s16 Si114xInit(void *port, int options, HANDLE *si114x_handle)
{
  s16 error = 0;
#if 1 
  u8 data;
// something wrong here
  (void) options;

  *si114x_handle = (HANDLE)&_handle;
  _handle.i2cPort = ((si114x_port_config_t*)port)->i2cPort;
  _handle.i2cAddress = ((si114x_port_config_t*)port)->i2cAddress << 1;
  _handle.irqPort = ((si114x_port_config_t*)port)->irqPort;
  _handle.irqPin = ((si114x_port_config_t*)port)->irqPin;

  data = Si114xReadFromRegister(*si114x_handle, REG_PART_ID);

  if ((_handle.i2cAddress == (0x60 << 1)) && (data != 0x46) && (data != 0x47))
    error = -1;
  if ((_handle.i2cAddress == (0x5A << 1)) && (data != 0x43))
    error = -1;

  Si114xIrqQueue_Clear(*si114x_handle);
#else
  int addr;
  _handle.i2cPort = ((si114x_port_config_t*)port)->i2cPort;
  addr= ((si114x_port_config_t*)port)->i2cAddress << 1;
  _handle.i2cAddress =addr;
  _handle.irqPort = ((si114x_port_config_t*)port)->irqPort;
  _handle.irqPin = ((si114x_port_config_t*)port)->irqPin;

#endif
  return error;
}

/**************************************************************************//**
 * @brief Close Si114x.
 *****************************************************************************/
s16 Si114xClose(HANDLE si114x_handle)
{
  (void) si114x_handle;
  _handle.i2cAddress = 0xff;
  return 0;
}

/**************************************************************************//**
 * @brief Reset not implemented.
 *****************************************************************************/
s16 Si114xSysReset(HANDLE si114x_handle)
{
 // (void) si114x_handle;
  return 0;
}

/**************************************************************************//**
 * @brief 10ms delay required by Si114x reset sequence.
 *****************************************************************************/
void delay_10ms(void)
{
  //RTCDRV_Delay(10);
  nrf_delay_ms(10);
  return;
}

/**************************************************************************//**
 * @brief 1ms delay required by Si114x polling sequence.
 *****************************************************************************/
void delay_1ms(void)
{
  nrf_delay_ms(1);
  //RTCDRV_Delay(1);
  return;
}

I2C_TransferReturn_TypeDef I2CSPM_Transfer(int8_t   *i2cPort, I2C_TransferSeq_TypeDef    *seq){
   
    ret_code_t ret =  slice_drv_twi_transfer(seq);
    
    if(!ret)
    {
        return i2c_return_error;
    }else{
        return i2c_return_ok;
    }
    
}

/**************************************************************************//**
 * @brief Write to Si114x i2c.
 *****************************************************************************/
static s16 Si114x_i2c_smbus_write_byte_data(HANDLE si114x_handle , u8 address, u8 data, bool block)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  si114x_port_config_t* handle;
  uint8_t i2c_write_data[2];
  uint8_t i2c_read_data[1];

  if (block)
    DisableSi114xInterrupt();
  seq.addr  = _handle.i2cAddress >> 1;//SI1144_I2C_ADDR;//
  seq.flags = I2C_FLAG_WRITE;
  /* Select register and data to write */
  i2c_write_data[0] = address;
  i2c_write_data[1] = data;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 2;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  handle = (si114x_port_config_t *)si114x_handle;

  ret = I2CSPM_Transfer(handle->i2cPort, &seq);

  if (block)
    EnableSi114xInterrupt();
  if (ret != i2cTransferDone)
  {
    return (s16)ret;
  }
  return (s16)0;
}

/**************************************************************************//**
 * @brief Read from Si114x i2c.
 *****************************************************************************/
static s16 Si114x_i2c_smbus_read_byte_data(HANDLE si114x_handle, u8 address, u8 *data, bool block)
{
  //  si114x_handle is not used in the EFM32.   We use a global instead
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[1];
  si114x_port_config_t* i2cDrvHandle;

  if (block)
    DisableSi114xInterrupt ();
  seq.addr  = _handle.i2cAddress>>1;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to start reading from */
  i2c_write_data[0] = address;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1;
  /* Select length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len  = 1;

  i2cDrvHandle = (si114x_port_config_t *)si114x_handle;

  ret = I2CSPM_Transfer(i2cDrvHandle->i2cPort, &seq);

  if (block)
    EnableSi114xInterrupt();
  if (ret != i2cTransferDone)
  {
    *data = 0xff;
    return((int) ret);
  }
  return((int) 1);
}

/**************************************************************************//**
 * @brief Write block of data to Si114x i2c.
 *****************************************************************************/
static s16 Si114x_i2c_smbus_write_i2c_block_data(HANDLE si114x_handle, u8 address, u8 length, u8 const* data, bool block)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[10];
  uint8_t i2c_read_data[1];
  si114x_port_config_t* handle;
  int i;

  if (block)
    DisableSi114xInterrupt ();
  seq.addr  =_handle.i2cAddress>>1;// SI1144_I2C_ADDR;//_handle.i2cAddress;
  seq.flags = I2C_FLAG_WRITE;
  /* Select register to start writing to*/
  i2c_write_data[0] = address;
  for (i=0; i<length;i++)
  {
    i2c_write_data[i+1] = data[i];
  }
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1+length;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  handle = (si114x_port_config_t *)si114x_handle;

  ret = I2CSPM_Transfer(handle->i2cPort, &seq);

  if (block)
    EnableSi114xInterrupt ();
  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }

  return((int) 0);
}

/**************************************************************************//**
 * @brief Read block of data from Si114x i2c.
 *****************************************************************************/
static s16 Si114x_i2c_smbus_read_i2c_block_data(HANDLE si114x_handle, u8 address, u8 length, u8* data, bool block)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[1];
  si114x_port_config_t* handle;

  seq.addr  =_handle.i2cAddress>>1;// SI1144_I2C_ADDR;// _handle.i2cAddress;
  seq.flags = I2C_FLAG_WRITE_READ;
  if (block)
    DisableSi114xInterrupt ();
  /* Select register to start reading from */
  i2c_write_data[0] = address;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1;

  /* Select length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len  = length;

  handle = (si114x_port_config_t *)si114x_handle;

  ret = I2CSPM_Transfer(handle->i2cPort, &seq);

  if (block)
    EnableSi114xInterrupt ();
  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }

  return((int) 0);
}


