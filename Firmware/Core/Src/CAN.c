/*
 * CAN.c
 *
 *  Created on: Aug 5, 2025
 *      Author: lukas
 */

#include "CAN.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;
extern void Error_Handler(void);

CAN_Message_t CAN_MessageDB[MAX_CAN_MESSAGES];

// Software-Sendequeue
static CAN_Message_t TxQueue[CAN_TX_QUEUE_LENGTH];
static volatile uint8_t TxQueueHead = 0;
static volatile uint8_t TxQueueTail = 0;

static CAN_RxCallback_t can_rx_callback = NULL;

void CAN_RegisterRxCallback(CAN_RxCallback_t cb)
{
    can_rx_callback = cb;
}

// ---------- Empfang ----------

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        Error_Handler();
    }

    // Callback aufrufen, falls registriert
    if (can_rx_callback) {
        can_rx_callback(rxHeader.StdId, rxData, rxHeader.DLC, HAL_GetTick());
    }

    // Update bestehende Nachricht
    for (int i = 0; i < MAX_CAN_MESSAGES; i++) {
        if (CAN_MessageDB[i].valid && CAN_MessageDB[i].id == rxHeader.StdId) {
            CAN_MessageDB[i].dlc = rxHeader.DLC;
            memcpy(CAN_MessageDB[i].data, rxData, rxHeader.DLC);
            CAN_MessageDB[i].timestamp = HAL_GetTick();
            return;
        }
    }

    // Neuen Slot belegen
    for (int i = 0; i < MAX_CAN_MESSAGES; i++) {
        if (!CAN_MessageDB[i].valid) {
            CAN_MessageDB[i].id = rxHeader.StdId;
            CAN_MessageDB[i].dlc = rxHeader.DLC;
            memcpy(CAN_MessageDB[i].data, rxData, rxHeader.DLC);
            CAN_MessageDB[i].timestamp = HAL_GetTick();
            CAN_MessageDB[i].valid = 1;
            return;
        }
    }
}

// ---------- Initialisierung ----------

void CAN_Init(void)
{
    // Alle Filter durchlassen
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);  // wichtig

    memset(CAN_MessageDB, 0, sizeof(CAN_MessageDB));
    memset(TxQueue, 0, sizeof(TxQueue));
    TxQueueHead = TxQueueTail = 0;
}

// ---------- Senden ----------

static uint8_t TxQueue_IsFull(void)
{
    return ((TxQueueHead + 1) % CAN_TX_QUEUE_LENGTH) == TxQueueTail;
}

static uint8_t TxQueue_IsEmpty(void)
{
    return TxQueueHead == TxQueueTail;
}

static void TxQueue_Push(const CAN_Message_t* msg)
{
    TxQueue[TxQueueHead] = *msg;
    TxQueueHead = (TxQueueHead + 1) % CAN_TX_QUEUE_LENGTH;
}

static CAN_Message_t* TxQueue_Peek(void)
{
    if (TxQueue_IsEmpty()) return NULL;
    return &TxQueue[TxQueueTail];
}

static void TxQueue_Pop(void)
{
    if (!TxQueue_IsEmpty()) {
        TxQueueTail = (TxQueueTail + 1) % CAN_TX_QUEUE_LENGTH;
    }
}

// Diese Funktion prüft freie Mailboxen und sendet aus Queue
static void CAN_ProcessTxQueue(void)
{
    if (TxQueue_IsEmpty()) return;

    for (int mbox = 0; mbox < 3; mbox++) {
        uint32_t mailbox = (mbox == 0) ? CAN_TX_MAILBOX0 :
                           (mbox == 1) ? CAN_TX_MAILBOX1 :
                                         CAN_TX_MAILBOX2;

        if (!HAL_CAN_IsTxMessagePending(&hcan, mailbox)) {
            CAN_Message_t* msg = TxQueue_Peek();
            if (msg == NULL) return;

            CAN_TxHeaderTypeDef txHeader;
            uint32_t txMailbox;

            txHeader.StdId = msg->id;
            txHeader.IDE = CAN_ID_STD;
            txHeader.RTR = CAN_RTR_DATA;
            txHeader.DLC = msg->dlc;
            txHeader.TransmitGlobalTime = DISABLE;

            if (HAL_CAN_AddTxMessage(&hcan, &txHeader, msg->data, &txMailbox) == HAL_OK) {
                TxQueue_Pop();
            } else {
                return; // Busy, abbrechen
            }
        }
    }
}

// Wird bei Mailbox-Übertragung abgeschlossen aufgerufen
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_ProcessTxQueue(); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_ProcessTxQueue(); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_ProcessTxQueue(); }

HAL_StatusTypeDef CAN_QueueMessage(const CAN_Message_t* msg)
{
    if (TxQueue_IsFull()) {
        return HAL_ERROR;  // Queue voll
    }

    TxQueue_Push(msg);
    CAN_ProcessTxQueue();  // direkt versuchen zu senden
    return HAL_OK;
}

// ---------- Lesen empfangener Daten ----------

CAN_Message_t* CAN_GetMessage(uint16_t id)
{
    for (int i = 0; i < MAX_CAN_MESSAGES; i++) {
        if (CAN_MessageDB[i].valid && CAN_MessageDB[i].id == id) {
            return &CAN_MessageDB[i];
        }
    }
    return NULL;
}

