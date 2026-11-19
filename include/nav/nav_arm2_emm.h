#ifndef NAV_ARM2_EMM_H
#define NAV_ARM2_EMM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
 float machine_zero, travel_per_output_rev, motor_rev_per_output_rev;
 uint32_t pulses_per_motor_rev;
 int32_t driver_zero;
 int direction;
} Arm2Axis;
typedef struct {int32_t position;uint16_t rpm;uint8_t accel;} Arm2EmmTarget;
typedef enum {ARM2_EMM_IDLE,ARM2_EMM_LIFT_ACK,ARM2_EMM_RACK_ACK,ARM2_EMM_READY,
              ARM2_EMM_TRIGGERED,ARM2_EMM_BLOCKED} Arm2EmmStage;
typedef struct {Arm2EmmTarget target[2];Arm2EmmStage stage;} Arm2EmmBatch;

bool nav_arm2_pulses(const Arm2Axis *axis,float position,int32_t *out);
bool nav_arm2_emm_move(uint8_t id,Arm2EmmTarget target,bool sync,uint8_t frame[13]);
/* Pure transaction model. It never calls a UART. The bus owner must reserve
 * the line, bind ACK age to TX time, and verify feedback after the trigger. */
bool nav_arm2_emm_begin(Arm2EmmBatch *batch,Arm2EmmTarget lift,Arm2EmmTarget rack);
size_t nav_arm2_emm_next(const Arm2EmmBatch *batch,uint8_t frame[13]);
bool nav_arm2_emm_ack(Arm2EmmBatch *batch,const uint8_t frame[4]);
void nav_arm2_emm_abort(Arm2EmmBatch *batch);
bool nav_arm2_emm_trigger_sent(Arm2EmmBatch *batch);
bool nav_arm2_emm_clear(Arm2EmmBatch *batch,bool drivers_pending_cleared);
#endif
