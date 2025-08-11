/*
 * ERROR.h
 *
 *  Created on: Aug 5, 2025
 *      Author: lukas
 */

#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>
#include "main.h"

// Fehlercodes definieren
typedef enum {
    ERROR_NONE = 0,
    ERROR_CAN_QUEUE_FULL = 1,
    ERROR_CAN_INIT_FAILED = 2,
    ERROR_CAN_TX_FAILED = 3,
    ERROR_DISPLAY_WRITE_FAILED = 4,
	// --- ADC / DMA Fehler ---
	ERROR_ADC_DMA_START_FAILED = 5,  // HAL_ADC_Start_DMA != HAL_OK
	ERROR_ADC_DMA_ERROR        = 6,  // HAL meldet DMA Fehler
	ERROR_ADC_OVERRUN          = 7,  // ADC Overrun (OVR)
	ERROR_ADC_INVALID_CHANNEL  = 8,  // Ungültiger Kanal an GetAverage()
	ERROR_ADC_NO_DATA          = 9,  // Noch keine Daten im FIFO
	ERROR_ADC_PARAM            = 10, // z.B. ADC_DMA_BLOCK_SIZE < ADC_SHORT_AVG_SAMPLES
    // Loop Overrun-Fehler
    ERROR_LOOP_OVERRUN_1MS = 11,
    ERROR_LOOP_OVERRUN_10MS = 12,
    ERROR_LOOP_OVERRUN_100MS = 13,
    ERROR_LOOP_OVERRUN_200MS = 14,

	ERROR_CAN_RX_EMPTY = 15,
	// DISPLAY / UI Fehler (ab 16)
	ERROR_DISPLAY_INVALID_PAGE      = 16, // Page >= DISP_MAX_PAGES
	ERROR_DISPLAY_INVALID_ROW       = 17, // row >= DISP_ROWS
	ERROR_DISPLAY_INVALID_COL       = 18, // col >= DISP_COLS
	ERROR_DISPLAY_WRITE_CLIPPED     = 19, // Text/Zahl wurde beim Schreiben abgeschnitten
	ERROR_DISPLAY_BAD_DECIMALS      = 20, // decimals < 0 oder zu groß (intern begrenzt)
	ERROR_DISPLAY_MINWIDTH_TOO_BIG  = 21, // min_width > verbleibende Spalten in der Zeile
    // weitere Fehler hier hinzufügen...
} ErrorCode_t;

// Fehler setzen (Eintrag in Liste oder Bitfeld)
void Error_Register(ErrorCode_t code);

// Fehler zurücksetzen
void Error_Clear(ErrorCode_t code);

// Alle Fehler als Bitmaske holen
uint32_t Error_GetAll(void);

// Ausgabe auf Display oder per CAN senden
void Error_ReportAll(void);

void Display_ErrorStatus(void);

void Error_LED_Update(void);

#endif

