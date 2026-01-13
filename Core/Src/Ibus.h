/*
 * Ibus.h
 *
 *  Created on: Jan 11, 2026
 *      Author: wluzp
 */

#ifndef SRC_IBUS_H_
#define SRC_IBUS_H_


#include <stdint.h>

#define IBUS_FRAME_LEN 32
#define IBUS_CHANNELS 14

extern volatile uint16_t ibus_ch[IBUS_CHANNELS];
extern volatile uint8_t  ibus_frame[IBUS_FRAME_LEN];
extern volatile uint8_t  ibus_rx_buf[IBUS_FRAME_LEN];
extern volatile uint8_t  ibus_rx_ready;

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
