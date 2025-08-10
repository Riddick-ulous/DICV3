/*
 * RGBLED.c
 *
 *  Created on: Aug 7, 2025
 *      Author: lukas
 */

#include "RGBLED.h"

RGBLED_t leds[RGBLED_COUNT];
static uint8_t txBuffer[(RGBLED_COUNT * 4) + 6]; // Start + LEDs + End

void RGBLED_Init(void)
{
    for (uint8_t i = 0; i < RGBLED_COUNT; i++) {
        leds[i].r = 0;
        leds[i].g = 0;
        leds[i].b = 0;
        leds[i].brightness = 1; // min
    }
}

void RGBLED_Set(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (index >= RGBLED_COUNT) return;
    leds[index].r = r;
    leds[index].g = g;
    leds[index].b = b;
    leds[index].brightness = (brightness > 31) ? 31 : brightness;
}

void RGBLED_Update(SPI_HandleTypeDef *hspi)
{
    uint16_t bufIndex = 0;

    // Startframe (4x 0x00)
    for (uint8_t i = 0; i < 4; i++) {
        txBuffer[bufIndex++] = 0x00;
    }

    // LED-Daten
    for (uint8_t i = 0; i < RGBLED_COUNT; i++) {
        txBuffer[bufIndex++] = 0b11100000 | (leds[i].brightness & 0x1F); // Helligkeit + Header
        txBuffer[bufIndex++] = leds[i].b;
        txBuffer[bufIndex++] = leds[i].g;
        txBuffer[bufIndex++] = leds[i].r;
    }

    // Endframe: mindestens (N/2) Bits, hier sicherheitshalber 2 Bytes
    txBuffer[bufIndex++] = 0xFF;
    txBuffer[bufIndex++] = 0xFF;

    HAL_SPI_Transmit(hspi, txBuffer, bufIndex, HAL_MAX_DELAY);
}

void RGBLED_TestPattern(void)
{
    static uint8_t R = 0;
    static uint8_t G = 0;
    static uint8_t B = 0;

    R += 10;
    G += 20;
    B += 30;

    for (uint8_t i = 0; i < RGBLED_COUNT; i++) {
        RGBLED_Set(i, R, G, B, 1);
    }

    if (R >= 100) R = 0;
    if (G >= 150) G = 0;
    if (B >= 200) B = 0;
}

