#include "ADC.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "ERROR.h"   // <--- Fehler-API einbinden

extern ADC_HandleTypeDef hadc;

// DMA-Puffer: Interleaved [CH0, CH1, CH2, CH3, CH0, ...]
#define TOTAL_DMA_SAMPLES (ADC_CHANNEL_COUNT * ADC_DMA_BLOCK_SIZE)
static uint16_t adc_dma_buffer[TOTAL_DMA_SAMPLES];

// Ringpuffer für 1-ms-Mittelwerte
static uint16_t avg_fifo[ADC_CHANNEL_COUNT][ADC_AGGREGATION_LIMIT];
static uint8_t  fifo_head[ADC_CHANNEL_COUNT]  = {0};   // Schreibposition
static uint8_t  fifo_count[ADC_CHANNEL_COUNT] = {0};   // Anzahl gespeicherter Werte
static uint8_t  value_read[ADC_CHANNEL_COUNT] = {1};   // Wert seit letztem Update gelesen?

void ADC_Init(void)
{
    // Plausicheck Konfiguration: wir brauchen mind. so viele DMA-Samples,
    // wie für ~1ms-Mittelung nötig sind.
    if (ADC_DMA_BLOCK_SIZE < ADC_SHORT_AVG_SAMPLES) {
        Error_Register(ERROR_ADC_PARAM);
        // wir starten trotzdem, aber Update() wird dann 0..n behandeln
    } else {
        Error_Clear(ERROR_ADC_PARAM);
    }

    if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_dma_buffer, TOTAL_DMA_SAMPLES) != HAL_OK) {
        Error_Register(ERROR_ADC_DMA_START_FAILED);
        return;
    } else {
        Error_Clear(ERROR_ADC_DMA_START_FAILED);
        // Falls zuvor "keine Daten" gemeldet wurde, lassen wir das beim ersten Update löschen
    }
}

// Voll-Callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc_)
{
    if (hadc_->Instance != ADC1) return;
    ADC_Update();
}

// (optional) Half-Callback, falls du doppelt so oft updaten willst
// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc_)
// {
//     if (hadc_->Instance != ADC1) return;
//     // -> könntest du nutzen, wenn du 2x 1ms-Blöcke in einem DMA-Block löst
// }

// Fehler-Callback der HAL
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc_)
{
    if (hadc_->Instance != ADC1) return;

    uint32_t err = HAL_ADC_GetError(hadc_);
    if (err & HAL_ADC_ERROR_DMA) Error_Register(ERROR_ADC_DMA_ERROR);
    if (err & HAL_ADC_ERROR_OVR) Error_Register(ERROR_ADC_OVERRUN);

    // Weitere Fehlerarten (JQOVF etc.) hat F0 nicht, aber hier erweiterbar
}

// Rechnet 1ms-Mittelwert(e) und füllt Ringpuffer
void ADC_Update(void)
{
    // Sobald Daten reinkommen, können wir "NO_DATA" ggf. löschen
    // (wir löschen erst NACH erfolgreichem FIFO-Schreiben unten)
    uint32_t temp_sum[ADC_CHANNEL_COUNT] = {0};

    // Guard: Falls Konfiguration nicht reicht, setze PARAM-Error
    if (ADC_DMA_BLOCK_SIZE < ADC_SHORT_AVG_SAMPLES) {
        Error_Register(ERROR_ADC_PARAM);
        return;
    }

    // 1-ms-Mittelwert pro Kanal aus Rohsamples berechnen
    for (int i = 0; i < ADC_SHORT_AVG_SAMPLES; i++) {
        for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
            temp_sum[ch] += adc_dma_buffer[i * ADC_CHANNEL_COUNT + ch];
        }
    }

    for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        uint16_t one_ms_avg = (uint16_t)(temp_sum[ch] / ADC_SHORT_AVG_SAMPLES);

        if (value_read[ch]) {
            // FIFO leeren, wenn seit letzter Abfrage gelesen wurde
            fifo_count[ch] = 0;
            fifo_head[ch]  = 0;
            value_read[ch] = 0;
        }

        // Neuen 1ms-Wert in den Ringpuffer schreiben (zirkular)
        avg_fifo[ch][fifo_head[ch]] = one_ms_avg;
        fifo_head[ch] = (fifo_head[ch] + 1) % ADC_AGGREGATION_LIMIT;

        if (fifo_count[ch] < ADC_AGGREGATION_LIMIT) {
            fifo_count[ch]++;
        }
    }

    // Wenn wir hier waren, haben wir Daten -> NO_DATA zurücksetzen
    Error_Clear(ERROR_ADC_NO_DATA);
}

uint16_t ADC_GetAverage(uint8_t channel)
{
    if (channel >= ADC_CHANNEL_COUNT) {
        Error_Register(ERROR_ADC_INVALID_CHANNEL);
        return 0;
    }

    if (fifo_count[channel] == 0) {
        Error_Register(ERROR_ADC_NO_DATA);
        return 0;
    }

    // Summe bilden (O(n)); wenn du O(1) willst, sag Bescheid -> dann halten wir pro FIFO die laufende Summe mit.
    uint32_t sum = 0;
    // FIFO ist zirkular – aber wir mitteln über ALLE aktuell gespeicherten Werte,
    // Reihenfolge ist für den Durchschnitt egal, daher einfacher: linear über 0..fifo_count-1
    for (int i = 0; i < fifo_count[channel]; i++) {
        sum += avg_fifo[channel][i];
    }

    value_read[channel] = 1; // markiert, dass beim nächsten Update neu begonnen wird (FIFO wird geleert)

    // wir haben erfolgreich geliefert -> offensichtliche Fehler löschen
    Error_Clear(ERROR_ADC_INVALID_CHANNEL);
    // ERROR_ADC_NO_DATA wurde schon in Update() gelöscht, wenn Daten rein kamen

    return (uint16_t)(sum / fifo_count[channel]);
}
