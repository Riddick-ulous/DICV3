/*
 * DISPLAYFUN.h
 *
 *  Created on: Aug 11, 2025
 *      Author: lukas
 */

#ifndef DISPLAYFUN_H
#define DISPLAYFUN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

// 128x64, Font_6x8 → 21 Zeichen je Zeile, 8 Zeilen
#define DISP_COLS 21
#define DISP_ROWS 8
#define DISP_MAX_PAGES 6    // anpassen, wie viele Pages du willst
#define DISP_CAN_ID_SET_PAGE   0x130
#define DISP_CAN_ID_STATUS     0x131

// Öffentliche API
void DISP_Init(void);                   // Display init + interner Buffer clear + erste Page rendern
void DISP_SetPage(uint8_t page);        // aktive Page setzen (0..DISP_MAX_PAGES-1)
uint8_t DISP_GetPage(void);
void DISP_ClearPage(uint8_t page);      // Page-Buffer leeren (Spaces)
void DISP_ClearActive(void);            // aktive Page leeren
void DISP_Render(void);                 // aktive Page → SSD1306 write

// Schreiben in den Page-Buffer (mit Clipping)
void DISP_WriteText(uint8_t row, uint8_t col, const char* text);
void DISP_WriteChar(uint8_t row, uint8_t col, char c);

// Zahlen helper:
// value ist Ganzzahl. decimals >= 0 → Dezimalpunkt einfügen (z. B. 1234, decimals=2 => "12.34")
// min_width: mind. Feldbreite (links gepadded mit Spaces). unit darf NULL sein.
void DISP_WriteNumber(int32_t value, int8_t decimals, uint8_t min_width,
                      uint8_t row, uint8_t col, const char* unit);

// Zeit helper: ms → "M:SS.mmm" (max 7+1 = 8 Zeichen)
void DISP_WriteLapTime(uint32_t ms, uint8_t row, uint8_t col);

// Praktische Kurzhelfer
static inline void DISP_WriteLabel(uint8_t row, uint8_t col, const char* label) {
    DISP_WriteText(row, col, label);
}

// CAN-Handler (optional nutzen): 0x130, DLC>=1, data[0] = page
void DISP_HandleCAN(uint16_t id, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

void DISP_GoHome(void);  // auf Home-Page (0) springen

void DISP_SendStatus(void);

// Direkter Zugriff (wenn du mal schummeln willst)
char* DISP_GetBufferForPage(uint8_t page);   // Zeilenweise: row*DISP_COLS + col

#endif // DISPLAYFUN_H

