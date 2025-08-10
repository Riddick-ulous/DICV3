/*
 * LOOP_TIMER.h
 * Double-buffered Max-Usage je Loopperiode
 */

#ifndef LOOP_TIMER_H
#define LOOP_TIMER_H

#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;
#define MICROS() ((uint32_t)__HAL_TIM_GET_COUNTER(&htim2))

typedef struct loop_timer_s {
    uint32_t period_ms;        // Soll-Periode
    uint32_t last_tick_ms;     // letzter Startzeitpunkt (ms)
    uint32_t start_us;         // Startzeitpunkt (µs)
    uint32_t exec_time_us;     // Ausführungszeit der letzten Runde (µs)

    uint16_t usage_x10;        // aktuelle Usage (0.1 %)
    uint16_t max_curr_x10;     // Max innerhalb der laufenden Periode
    uint16_t max_prev_x10;     // Max der vollständig ABGESCHLOSSENEN letzten Periode (für Reporting)

    struct loop_timer_s *parent; // schnellere Loop, NULL falls keine
} loop_timer_t;

// Debug/Release inlining
#ifdef DEBUG
  #define LOOP_INLINE
#else
  #define LOOP_INLINE static inline
#endif

LOOP_INLINE void loop_init(loop_timer_t *lt, uint32_t period_ms, loop_timer_t *parent)
{
    lt->period_ms      = period_ms;
    lt->last_tick_ms   = 0;
    lt->start_us       = 0;
    lt->exec_time_us   = 0;
    lt->usage_x10      = 0;
    lt->max_curr_x10   = 0;
    lt->max_prev_x10   = 0;
    lt->parent         = parent;
}

// fällig? (mit Parent-Kaskade)
LOOP_INLINE uint8_t loop_due(loop_timer_t *lt, uint32_t systime_ms)
{
    if (lt->period_ms == 0) return 0; // nicht init
    // Parent muss in dieser while-Iteration bereits gelaufen sein
    if (lt->parent && lt->parent->last_tick_ms == lt->last_tick_ms) return 0;
    return (systime_ms - lt->last_tick_ms) >= lt->period_ms;
}

// neue Periode starten: ROLLOVER des Max-Werts
LOOP_INLINE void loop_start(loop_timer_t *lt, uint32_t systime_ms)
{
    // Rollover: abgeschlossene Periode → prev, aktuellen Puffer leeren
    lt->max_prev_x10 = lt->max_curr_x10;
    lt->max_curr_x10 = 0;

    lt->last_tick_ms = systime_ms;
    lt->start_us     = MICROS();
}

// Periode beenden, Usage & Max der laufenden Periode aktualisieren
LOOP_INLINE void loop_end(loop_timer_t *lt, ErrorCode_t overrun_code)
{
    uint32_t now_us = MICROS();
    lt->exec_time_us = (now_us >= lt->start_us)
                     ? (now_us - lt->start_us)
                     : ((0xFFFFFFFFu - lt->start_us) + now_us + 1u);

    // 0.1%-Skala, „round to nearest“
    lt->usage_x10 = (uint16_t)((lt->exec_time_us + (lt->period_ms/2u)) / lt->period_ms);
    if (lt->usage_x10 > lt->max_curr_x10) lt->max_curr_x10 = lt->usage_x10;

    // Overrun?
    if (lt->exec_time_us > (lt->period_ms * 1000u)) {
        Error_Register(overrun_code);
    }
}

#ifdef __cplusplus
}
#endif
#endif // LOOP_TIMER_H
