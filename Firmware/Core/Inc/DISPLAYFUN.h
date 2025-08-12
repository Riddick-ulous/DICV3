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
#include "ERROR.h"

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

/**
 * @brief Schreibt eine Zahl auf das Display.
 *
 * @param value      Die Zahl, die angezeigt werden soll.
 * @param decimals   Anzahl der Nachkommastellen (0–6 empfohlen).
 * @param min_width  Mindestbreite des Feldes (auffüllen mit Leerzeichen).
 * @param row        Zielzeile (0–7).
 * @param col        Zielspalte (0–20).
 * @param unit       Optionaler Einheitstext (z. B. "V" oder "°C"), NULL wenn keine.
 *
 * Beispiel:
 * @code
 * DISP_WriteNumber(1234, 2, 6, 1, 0, "V"); // zeigt "  12.34V" in Zeile 1, Spalte 0
 * @endcode
 */
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

void Display_ErrorStatus(uint8_t page, uint8_t row, uint8_t col);

void DISP_DrawBitmap(const uint8_t* bmp, uint8_t w, uint8_t h, uint8_t x, uint8_t y);
void DISP_ShowBootLogo(const uint8_t* logo, uint16_t w, uint16_t h,
                       bool invert, bool xbm_row_major, bool xbm_msb_first);

void xbm_to_ssd1306_pages(uint8_t *dst, const uint8_t *src,
                                 uint16_t w, uint16_t h, bool msb_first);

static inline uint8_t bitrev8(uint8_t v);

static inline uint8_t XBM_GetBit(const uint8_t* xbm, uint16_t w,
                                 uint16_t x, uint16_t y, bool msb_first);

static void BLIT_XBM_ToDisplay(const uint8_t* xbm, uint16_t w, uint16_t h,
                               int16_t x0, int16_t y0, bool invert, bool msb_first);

#endif // DISPLAYFUN_H

