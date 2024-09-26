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
#ifndef SI114X_SYS_OUT
#define SI114X_SYS_OUT
#include <stdint.h>
#include "Si114x_types.h"
      
typedef struct 
{
    uint16_t    sequence;       /**< sequence number */
    uint16_t    timestamp ;     /**< 16-bit Timestamp to record */
    uint16_t    pad;            /**< pad */
    uint8_t     irqstat;        /**< 8-bit irq status */
    uint16_t    vis;            /**< VIS */
    uint16_t    ir;             /**< IR */
    uint16_t    ps1;            /**< PS1 */
    uint16_t    ps2;            /**< PS2 */
    uint16_t    ps3;            /**< PS3 */
    uint16_t    aux;            /**< AUX */
}si114x_IRQ_sample_t;

typedef struct Si114xPortConfig
{
  uint8_t       i2cAddress;     /**< I2C address of Si114x */
  int8_t        irqPort;        /**< Port for Si114x INT pin */
  uint8_t       irqPin;         /**< Pin for Si114x INT pin */
  uint8_t       rstPin;         /**< Pin for reset Si114x   */
  int8_t       *i2cPort;        /**< I2C port Si114x is connected to */
}si114x_port_config_t;

s16 Si114xProcessIrq(HANDLE si114x_handle, u16 timestamp);
int16_t Si114xWriteToRegister(HANDLE si114x_handle, uint8_t address, uint8_t data);
int16_t Si114xBlockWrite(HANDLE si114x_handle,
                     uint8_t address, uint8_t length, uint8_t *values);
int16_t Si114xReadFromRegister(HANDLE si114x_handle, uint8_t address);

int16_t Si114xBlockRead(HANDLE si114x_handle,
                    uint8_t address, uint8_t length, uint8_t *values);
s16 Si114xInit(void *port, int options, HANDLE *si114x_handle);
#endif
