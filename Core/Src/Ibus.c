/*
 * Ibus.c
 *
 *  Created on: Jan 11, 2026
 *      Author: wluzp
 */

#include <stdint.h>
#include "Ibus.h"

#define IBUS_FRAME_LEN 32
#define IBUS_CHANNELS 14

volatile uint16_t ibus_ch[IBUS_CHANNELS];
volatile uint8_t  ibus_frame[IBUS_FRAME_LEN];
volatile uint8_t  ibus_rx_buf[IBUS_FRAME_LEN];
volatile uint8_t  ibus_rx_ready;

void ibus_test(void)
{
    static uint16_t v = 1000;
    v += 10;
    if (v > 2000) v = 1000;

    for (int i = 0; i < IBUS_CHANNELS; i++)
        ibus_ch[i] = v;
}

void ibus_decode(const uint8_t *b, uint16_t *ch)
{
    uint16_t checksum = 0xFFFF;

    for (int i = 0; i < 30; i++)
        checksum -= b[i];

    uint16_t rx_checksum = b[30] | (b[31] << 8);
    if (checksum != rx_checksum)
        return; // invalid frame

    for (int i = 0; i < 14; i++)
    {
        ch[i] = b[2 + i*2] | (b[3 + i*2] << 8);
    }
}

void ibus_build(void)
{
    uint16_t checksum = 0xFFFF;

    ibus_frame[0] = 0x20;
    ibus_frame[1] = 0x40;

    checksum -= ibus_frame[0];
    checksum -= ibus_frame[1];

    for (int i = 0; i < IBUS_CHANNELS; i++)
    {
        uint16_t v = ibus_ch[i];

        ibus_frame[2 + i*2]     = v & 0xFF;
        ibus_frame[2 + i*2 + 1] = v >> 8;

        checksum -= ibus_frame[2 + i*2];
        checksum -= ibus_frame[2 + i*2 + 1];
    }

    ibus_frame[30] = checksum & 0xFF;
    ibus_frame[31] = checksum >> 8;
}


