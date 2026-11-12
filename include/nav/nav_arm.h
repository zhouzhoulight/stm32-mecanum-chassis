#ifndef NAV_ARM_H
#define NAV_ARM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void nav_arm_init(void);
void nav_arm_tick(bool permit_lift);
bool nav_arm_command(const char *line);
void nav_arm_reply(const uint8_t *,unsigned);
void nav_arm_sent(uint32_t);
void nav_arm_stop(void);
bool nav_arm_busy(void);
bool nav_arm_turn(unsigned pwm);
bool nav_arm_turn_done(unsigned pwm);
bool nav_arm_calibration_ready(void);
bool nav_arm_transport_ready(void);
/* Cached commanded hold only; permits STOP+refresh, never permits driving. */
bool nav_arm_transport_held(void);
bool nav_arm_clearance_ready(void);
bool nav_arm_idle_ready(void);
bool nav_arm_faulted(void);
bool nav_arm_job_start(const char *job,unsigned slot);
bool nav_arm_bus_allowed(void);
void nav_arm_suspend_reads(void);
bool nav_app_station_wait(void);
void nav_arm_hw_init(void);
bool nav_arm_prefetch(unsigned slot,unsigned turn,bool open_jaw);
bool nav_arm_envelope_ready(void);
#endif
