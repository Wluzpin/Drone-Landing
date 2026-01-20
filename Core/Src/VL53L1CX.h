/*
 * VL53L1CX.h
 *
 *  Created on: Jan 4, 2026
 *      Author: wluzp
 */

#ifndef SRC_VL53L1CX_H_
#define SRC_VL53L1CX_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void VL53_WriteReg(uint16_t addr, uint8_t value);
uint8_t VL53_ReadReg(uint16_t addr);
void VL53_InitRegisters(void);
uint16_t VL53_ReadDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_VL53L1CX_H_ */
