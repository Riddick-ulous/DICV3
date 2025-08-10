/*
 * RGBLED.c
 *
 *  Created on: Aug 7, 2025
 *      Author: lukas
 */

#include "RGBLED.h"

RGBLED_t leds[RGBLED_COUNT];
static uint8_t txBuffer[(RGBLED_COUNT * 4) + 6]; // Start + LEDs + End

static uint32_t last_led_update_ms = 0;
static uint8_t  led_control_active = 0; // 0 = Testmode, 1 = CAN-Mode

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

    R += 1;
    G += 2;
    B += 3;

    for (uint8_t i = 0; i < RGBLED_COUNT; i++) {
        RGBLED_Set(i, R, G, B, 1);
    }

    if (R >= 100) R = 0;
    if (G >= 150) G = 0;
    if (B >= 200) B = 0;
}

// Erwartet CAN-Frame: ID 0x110 bis 0x110 + n, 8 Bytes pro LED-Paar
// Bytes: [Index, R, G, B, Bright, Index, R, G, B, Bright] (max. 2 LEDs pro Frame)
void RGBLED_HandleCAN(uint16_t id, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    // Grund-ID prüfen
    if (id < 0x110 || id > 0x11B) return;
    if (dlc < 8) return;

    uint8_t base = (uint8_t)((id - 0x110) * 2);

    RGBLED_Set(base + 0, data[0], data[1], data[2], data[3] & 0x1F);
    RGBLED_Set(base + 1, data[4], data[5], data[6], data[7] & 0x1F);

    last_led_update_ms = now_ms;
    led_control_active = 1;
}

void RGBLED_TimeoutCheck(uint32_t now_ms, SPI_HandleTypeDef *hspi)
{
    if (led_control_active) {
        // Prüfen ob wir das Timeout überschritten haben → zurück in Testmode
        if ((now_ms - last_led_update_ms) > 1000u) {
            led_control_active = 0;
        }
    }

    if (led_control_active) {
        RGBLED_Update(hspi);
    } else {
        static uint8_t was_active = 0;

        // Wenn wir gerade aus aktivem CAN-Mode kommen → einmal alles aus
        if (was_active) {
            RGBLED_AllOff();
            RGBLED_Update(hspi);
            was_active = 0;
        }

        RGBLED_TestPattern();
        RGBLED_Update(hspi);
    }

    // Merken ob wir vorher aktiv waren, um beim nächsten Wechsel reagieren zu können
    static uint8_t prev_active = 0;
    if (!prev_active && led_control_active) {
        // Wechsel von Testmode → CAN-Mode → vorher alle LEDs aus
        RGBLED_AllOff();
        RGBLED_Update(hspi);
    }
    prev_active = led_control_active;
}

void RGBLED_AllOff(void)
{
    for (uint8_t i = 0; i < RGBLED_COUNT; i++) {
        RGBLED_Set(i, 0, 0, 0, 0);
    }
}

