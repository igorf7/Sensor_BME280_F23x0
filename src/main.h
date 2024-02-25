/*
 * main.h
*/
#ifndef __MAIN_H
#define __MAIN_H

#include "msp430f2x.h"
#include "bme280.h"
#include <string.h>

/*  */
typedef struct {
    float temperature,
          pressure,
          humidity;
} Parameter_t;

/*  */
typedef struct {
    float voltage_tx, // Battery voltage in rf transmit mode
          voltage_sl; // Battery voltage in rf sleep mode
    uint32_t uptime;
} Diagnostic_t;

// Callbacks
void OnRfTxDone(void);
void OnRfTxError(void);

// Handlers
void BackgroundTask(void);
void TimerA0Task(void *prm);
void SendPacketTask(void *prm);
void SetRfSleepTask(void *prm);
#endif
