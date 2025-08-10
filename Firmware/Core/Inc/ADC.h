/*
 * ADC.h
 *
 *  Created on: Aug 7, 2025
 *      Author: lukas
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>

#define ADC_CHANNEL_COUNT         4
#define ADC_DMA_BLOCK_SIZE        64      // Anzahl Samples im DMA-Puffer
#define ADC_SHORT_AVG_SAMPLES     12      // ~1 ms Fenster bei ~12 kHz/Kanal
#define ADC_AGGREGATION_LIMIT     100     // Max. Anzahl gespeicherter 1-ms-Mittelwerte

void ADC_Init(void);
void ADC_Update(void);
uint16_t ADC_GetAverage(uint8_t channel);

#endif /* INC_ADC_H_ */

