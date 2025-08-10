/*
 * ERROR.c
 *
 *  Created on: Aug 5, 2025
 *      Author: lukas
 */


#include "error.h"
#include <stdio.h>  // für Debugausgabe
#include "ssd1306.h"
#include "ssd1306_fonts.h"

static volatile uint32_t errorFlags = 0;

void Error_Register(ErrorCode_t code)
{
    if (code > 0 && code < 32) {
        errorFlags |= (1U << code);
        //printf("Error registered: %u\r\n", code);  // optional
    }
}

void Error_Clear(ErrorCode_t code)
{
    if (code > 0 && code < 32) {
        errorFlags &= ~(1U << code);
    }
}

uint32_t Error_GetAll(void)
{
    return errorFlags;
}

void Error_ReportAll(void)
{
    // Beispielhafte Anzeige oder Übertragung
   printf("Aktive Fehler: 0x%08lX\r\n", errorFlags);
}


void Display_ErrorStatus(void)
{
    uint32_t errors = Error_GetAll();

    ssd1306_SetCursor(0, 20);

    if (errors == 0) {
        ssd1306_WriteString("Status: OK", Font_6x8, White);
    } else {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Err: 0x%08lX", errors);
        ssd1306_WriteString(buffer, Font_6x8, White);
    }

    ssd1306_UpdateScreen();
}

// Interner Zustand für Blink-Timing
static uint8_t error_led_state = 0;

void Error_LED_Update(void)
{
    if (Error_GetAll() != 0)
    {
        // Mindestens ein Fehler aktiv → LED toggeln
        HAL_GPIO_TogglePin(Error_GPIO_Port, Error_Pin);
        error_led_state = 1;
    }
    else
    {
        // Keine Fehler → LED aus, Zustand merken
        if (error_led_state)
        {
            HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_RESET);
            error_led_state = 0;
        }
    }
}
