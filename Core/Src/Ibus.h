/*
 * Ibus.h
 *
 *  Created on: Jan 11, 2026
 *      Author: wluzp
 */

#ifndef SRC_IBUS_H_
#define SRC_IBUS_H_

#include "Ibus.h"

#ifdef __cplusplus
extern "C" {
#endif

void ibus_test(void);
void ibus_decode(const uint8_t *b, uint16_t *ch);
void ibus_build(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_IBUS_H_ */
