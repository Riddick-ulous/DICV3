/*
 * ADC.h
 *
 *  Created on: Aug 7, 2025
 *      Author: lukas
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define ADC_CHANNEL_COUNT         4
#define ADC_DMA_BLOCK_SIZE        64      // Anzahl Samples im DMA-Puffer
#define ADC_SHORT_AVG_SAMPLES     12      // ~1 ms Fenster bei ~12 kHz/Kanal
#define ADC_AGGREGATION_LIMIT     100     // Max. Anzahl gespeicherter 1-ms-Mittelwerte

// --- Neue Enums für die Hardware-Schalter ---
typedef enum {
    ADC_SUPPLY_5V  = 0,   // CTRL = Low
    ADC_SUPPLY_12V = 1    // CTRL = High
} ADC_SupplyMode_t;

typedef enum {
    ADC_SENSOR_NORMAL = 0, // NTC_CTRL = Low
    ADC_SENSOR_NTC    = 1  // NTC_CTRL = High
} ADC_SensorMode_t;


void ADC_Init(void);
void ADC_Update(void);
uint16_t ADC_GetAverage(uint8_t channel);

// --- Neue Ctrl-API ---
void ADC_CtrlInitDefaults(void);                                      // hardcodierte Startkonfiguration anwenden
void ADC_CtrlApply(uint8_t ch, ADC_SupplyMode_t supply, ADC_SensorMode_t sensor);
void ADC_CtrlGet(uint8_t ch, ADC_SupplyMode_t *supply, ADC_SensorMode_t *sensor);
void ADC_FlushChannel(uint8_t ch);                                     // Mittelwert-FIFO dieses Kanals leeren

void ADC_HandleCAN(uint16_t id, const uint8_t *data, uint8_t dlc, uint32_t timestamp);
void ADC_SendCurrentConfig(void);

#endif /* INC_ADC_H_ */

