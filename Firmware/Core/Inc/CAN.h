/*
 * CAN.h
 *
 *  Created on: Aug 5, 2025
 *      Author: lukas
 */

#ifndef CAN_H
#define CAN_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

#define MAX_CAN_MESSAGES         16   // Empfangs-Speicherplätze
#define CAN_TX_QUEUE_LENGTH      32   // Sende-Queue-Länge

typedef void (*CAN_RxCallback_t)(
    uint16_t id,
    const uint8_t *data,
    uint8_t dlc,
    uint32_t timestamp
);

void CAN_RegisterRxCallback(CAN_RxCallback_t cb);

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t timestamp;
    uint8_t valid;
} CAN_Message_t;

// Empfangs-Datenbank
extern CAN_Message_t CAN_MessageDB[MAX_CAN_MESSAGES];

// Initialisierung (Filter, Start, Interrupts)
void CAN_Init(void);

// Nachricht sofort oder gepuffert senden
HAL_StatusTypeDef CAN_QueueMessage(const CAN_Message_t* msg);

// Letzte empfangene Nachricht zur ID holen
CAN_Message_t* CAN_GetMessage(uint16_t id);


#endif  // CAN_H


