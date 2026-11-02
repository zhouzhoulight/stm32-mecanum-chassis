#ifndef NAV_BUS_H
#define NAV_BUS_H
#include "nav_types.h"
typedef struct {
 uint32_t errors,timeouts,rx_frames,last_reply,flags_at[4],position_at[4],tx_packets;
 uint32_t velocity_requested,velocity_done,enable_done;
 uint8_t flags[4],stop_ack,disable_ack;bool stopping,transaction,tx_fault;
 uint32_t rx_resyncs;
} NavBus;
void nav_bus_init(uint32_t now);
void nav_bus_tick(uint32_t now);
void nav_bus_feed(uint8_t byte,uint32_t at);
void nav_bus_uart_error(void);
void nav_bus_emergency(uint32_t now);
bool nav_bus_enable(void);
uint32_t nav_bus_velocity(const int wheel[4]);
const NavBus *nav_bus_data(void);
#endif
