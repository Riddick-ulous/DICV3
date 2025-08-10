/*
 * RGBLED.h
 *
 *  Created on: Aug 7, 2025
 *      Author: lukas
 */

#ifndef RGBLED_H
#define RGBLED_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define RGBLED_COUNT 24

typedef struct {
    uint8_t r;          // Rot 0-255
    uint8_t g;          // Grün 0-255
    uint8_t b;          // Blau 0-255
    uint8_t brightness; // 0-31 (BB-2020BGR-TRB akzeptiert 5 Bit)
} RGBLED_t;

extern RGBLED_t leds[RGBLED_COUNT];

void RGBLED_Init(void);
void RGBLED_Set(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
void RGBLED_Update(SPI_HandleTypeDef *hspi);
void RGBLED_TestPattern(void);

#endif
