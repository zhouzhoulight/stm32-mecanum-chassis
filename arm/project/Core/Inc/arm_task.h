#ifndef ARM_TASK_H
#define ARM_TASK_H
int ArmTask_Run(void);
void ArmTask_Init(void);
void ArmTask_Tick(void);
void ArmTask_MotorIRQ(void);
void ArmTask_DebugIRQ(void);
#endif
