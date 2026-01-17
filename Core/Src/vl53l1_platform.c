/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include <i2c.h>

extern I2C_HandleTypeDef hi2c1;

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_StatusTypeDef hal_status;
    uint8_t buffer[count + 2];

    /* Split 16-bit register address into MSB + LSB */
    buffer[0] = (uint8_t)(index >> 8);      // register MSB
    buffer[1] = (uint8_t)(index & 0xFF);    // register LSB

    /* Copy payload after register address */
    for (uint32_t i = 0; i < count; i++)
    {
        buffer[i + 2] = pdata[i];
    }

    /* Send register address + data in ONE I2C transaction */
    hal_status = HAL_I2C_Master_Transmit(
                    &hi2c1,
                    dev,
                    buffer,
                    count + 2,
                    HAL_MAX_DELAY
                 );

    if (hal_status == HAL_OK)
        return 0;
    else
        return -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	HAL_StatusTypeDef hal_status;
	    uint8_t reg[2];

	    /* Split 16-bit register address */
	    reg[0] = (uint8_t)(index >> 8);      // register MSB
	    reg[1] = (uint8_t)(index & 0xFF);    // register LSB

	    /* Step 1: send register address */
	    hal_status = HAL_I2C_Master_Transmit(
	                    &hi2c1,
	                    dev,
	                    reg,
	                    2,
	                    HAL_MAX_DELAY
	                 );

	    if (hal_status != HAL_OK)
	        return -1;

	    /* Step 2: read data */
	    hal_status = HAL_I2C_Master_Receive(
	                    &hi2c1,
	                    dev,
	                    pdata,
	                    count,
	                    HAL_MAX_DELAY
	                 );

	    if (hal_status == HAL_OK)
	        return 0;
	    else
	        return -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}
