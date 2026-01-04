/*
 * VL53L1CX.c
 *
 *  Created on: Jan 4, 2026
 *      Author: wluzp
 */

#include "VL53L1CX.h"

#define VL53L1CX_ADDR (0x52 << 1)
extern I2C_HandleTypeDef hi2c1;

// Tablica konfiguracji (jak wcześniej)
const uint8_t vl53l1x_long_range_config[] = {
    // ┌─────────────────────────────────────────────────────────────┐
    // │                    KONFIGURACJA CZASOWA                     │
    // └─────────────────────────────────────────────────────────────┘

    // TIMING_CONFIG__TIMING_BUDGET_MSB - Starszy bajt czasu pomiaru (50ms = 0x1388)
    0x00, 0x2D, 0x00,  // 0x002D = TIMING_CONFIG__TIMING_BUDGET_MSB

    // TIMING_CONFIG__TIMING_BUDGET_LSB - Młodszy bajt czasu pomiaru (50ms = 0x1388)
    0x00, 0x2E, 0x32,  // 0x002E = TIMING_CONFIG__TIMING_BUDGET_LSB (0x32 = 50 decimal)

    // SYSTEM__MODE_START - Rozpoczęcie pomiaru
    0x00, 0x80, 0x01,  // 0x0080 = SYSTEM__MODE_START (0x01 = start pomiaru)

    // ┌─────────────────────────────────────────────────────────────┐
    // │                   KONFIGURACJA GPIO                         │
    // └─────────────────────────────────────────────────────────────┘

    // GPIO_HV_MUX__CTRL - Kontrola multipleksera GPIO
    0x01, 0x00, 0x00,  // 0x0100 = GPIO_HV_MUX__CTRL (0x00 = domyślne ustawienie)

    // GPIO_HV_PAD_CTRL - Kontrola wyjść GPIO
    0x01, 0x01, 0x00,  // 0x0101 = GPIO_HV_PAD_CTRL (0x00 = domyślne)

    // ┌─────────────────────────────────────────────────────────────┐
    // │                 KALIBRACJA I TIMEOUTY                       │
    // └─────────────────────────────────────────────────────────────┘

    // VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND - Timeout kalibracji VHV
    0x01, 0x02, 0x00,  // 0x0102 = VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND (0x00 = standardowy)

    // PHASECAL_CONFIG__TIMEOUT_MACROP - Timeout kalibracji fazy
    0x01, 0x03, 0x08,  // 0x0103 = PHASECAL_CONFIG__TIMEOUT_MACROP (0x08 = zalecana wartość)

    // ┌─────────────────────────────────────────────────────────────┐
    // │                PARAMETRY VCSEL (LASER)                      │
    // └─────────────────────────────────────────────────────────────┘

    // RANGE_CONFIG__VCSEL_PERIOD_A - Okres impulsu VCSEL dla pomiaru A
    0x01, 0x04, 0x10,  // 0x0104 = RANGE_CONFIG__VCSEL_PERIOD_A (0x10 = dłuższy impuls = większy zasięg)

    // RANGE_CONFIG__VCSEL_PERIOD_B - Okres impulsu VCSEL dla pomiaru B
    0x01, 0x05, 0x01,  // 0x0105 = RANGE_CONFIG__VCSEL_PERIOD_B (0x01 = krótszy impuls)

    // RANGE_CONFIG__TIMEOUT_MACROP_A_HI - Starszy bajt timeoutu pomiaru A
    0x01, 0x06, 0x01,  // 0x0106 = RANGE_CONFIG__TIMEOUT_MACROP_A_HI (0x01 = część starsza)

    // RANGE_CONFIG__TIMEOUT_MACROP_A_LO - Młodszy bajt timeoutu pomiaru A
    0x01, 0x07, 0xE8,  // 0x0107 = RANGE_CONFIG__TIMEOUT_MACROP_A_LO (0xE8 = 232, razem 488)

    // RANGE_CONFIG__TIMEOUT_MACROP_B_HI - Starszy bajt timeoutu pomiaru B
    0x01, 0x08, 0x01,  // 0x0108 = RANGE_CONFIG__TIMEOUT_MACROP_B_HI

    // RANGE_CONFIG__TIMEOUT_MACROP_B_LO - Młodszy bajt timeoutu pomiaru B
    0x01, 0x09, 0xE8,  // 0x0109 = RANGE_CONFIG__TIMEOUT_MACROP_B_LO (taki sam jak A)

    // ┌─────────────────────────────────────────────────────────────┐
    // │                   PROGI I FILTRY                            │
    // └─────────────────────────────────────────────────────────────┘

    // RANGE_CONFIG__SIGMA_THRESH - Próg sigma (pewność pomiaru)
    0x01, 0x0A, 0x00,  // 0x010A = RANGE_CONFIG__SIGMA_THRESH (0x00 = brak ograniczeń)

    // RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS - Min. ilość odbitych fotonów
    0x01, 0x0B, 0x00,  // 0x010B = RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS (0x00 = brak ograniczeń)

    // RANGE_CONFIG__VALID_PHASE_HIGH - Górna granica prawidłowej fazy
    0x01, 0x10, 0x00,  // 0x0110 = RANGE_CONFIG__VALID_PHASE_HIGH (0x00 = domyślne)

    // RANGE_CONFIG__VALID_PHASE_LOW - Dolna granica prawidłowej fazy
    0x01, 0x11, 0x00,  // 0x0111 = RANGE_CONFIG__VALID_PHASE_LOW (0x00 = domyślne)

    // ┌─────────────────────────────────────────────────────────────┐
    // │                 OKRES POMIARÓW I ALARMÓW                    │
    // └─────────────────────────────────────────────────────────────┘

    // SYSTEM__INTERMEASUREMENT_PERIOD - Okres między pomiarami (0x00186A0 = 100ms)
    0x01, 0x20, 0x00,  // 0x0120 = SYSTEM__INTERMEASUREMENT_PERIOD_HI   (0x00)
    0x01, 0x21, 0x18,  // 0x0121 = SYSTEM__INTERMEASUREMENT_PERIOD_MID  (0x18)
    0x01, 0x22, 0x6A,  // 0x0122 = SYSTEM__INTERMEASUREMENT_PERIOD_LO   (0x6A)

    // SYSTEM__THRESH_HIGH - Górny próg alarmowy (nieużywany)
    0x01, 0x30, 0x00,  // 0x0130 = SYSTEM__THRESH_HIGH_HI (0x00 = wyłączone)
    0x01, 0x31, 0x00,  // 0x0131 = SYSTEM__THRESH_HIGH_LO (0x00 = wyłączone)

    // ┌─────────────────────────────────────────────────────────────┐
    // │              KONFIGURACJA DSS (SPADY)                       │
    // └─────────────────────────────────────────────────────────────┘

    // DSS_CONFIG__TARGET_TOTAL_RATE_MCPS - Docelowa szybkość sygnału
    0x01, 0x40, 0x00,  // 0x0140 = DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_HI (0x00)
    0x01, 0x41, 0x00,  // 0x0141 = DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_LO (0x00)

    // DSS_CONFIG__ROI_MODE_CONTROL - Tryb kontroli ROI
    0x01, 0x50, 0x00,  // 0x0150 = DSS_CONFIG__ROI_MODE_CONTROL (0x00 = domyślne)

    // DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT - Ręczny wybór aktywnych SPADów
    0x01, 0x51, 0x01,  // 0x0151 = DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT (0x01 = minimalna wartość)

    // DSS_CONFIG__APERTURE_ATTENUATION - Tłumienie apertury
    0x01, 0x60, 0x00,  // 0x0160 = DSS_CONFIG__APERTURE_ATTENUATION (0x00 = brak tłumienia)
};

void VL53_WriteReg(uint16_t addr, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, VL53L1CX_ADDR, addr, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
}

uint8_t VL53_ReadReg(uint16_t addr) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, VL53L1CX_ADDR, addr, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
    return data;
}

void VL53_InitRegisters(void) {
    VL53_WriteReg(0x0000, 0x00);
    HAL_Delay(1);
    VL53_WriteReg(0x0000, 0x01);

    while (VL53_ReadReg(0x00E5) != 1) {
        HAL_Delay(1);
    }

    for (int i = 0; i < sizeof(vl53l1x_long_range_config); i += 3) {
        uint16_t addr = (vl53l1x_long_range_config[i] << 8) | vl53l1x_long_range_config[i+1];
        uint8_t value = vl53l1x_long_range_config[i+2];
        VL53_WriteReg(addr, value);
    }

    VL53_WriteReg(0x0080, 0x01);
}

uint16_t VL53_ReadDistance(void) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c1, VL53L1CX_ADDR, 0x0096, I2C_MEMADD_SIZE_16BIT, data, 2, 100);
    return (data[0] << 8) | data[1];
}


