#ifndef VEHICLE_TX_QUEUE_H
#define VEHICLE_TX_QUEUE_H
#include <stdbool.h>
#include <stdint.h>
/* Single producer under an IRQ critical section; single IRQ consumer.
 * Priority is selected ONLY between complete AA55 envelopes. */
typedef struct {
 uint8_t data[4096],control[512];
 volatile uint16_t dh,dt,ch,ct;
 uint16_t remaining;
 bool priority_active;
} VehicleTxQueue;
void vehicle_tx_queue_init(VehicleTxQueue *q);
bool vehicle_tx_queue_push(VehicleTxQueue *q,const uint8_t *p,uint32_t n,bool priority);
bool vehicle_tx_queue_pop(VehicleTxQueue *q,uint8_t *byte);
#endif
