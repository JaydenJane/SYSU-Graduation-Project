#ifndef __PWM_H
#define __PWM_H

void MotorPWMInit(void);
void PWMTimerInit(void);
void Set_Joint_Angel(int angle);
extern int joint_id; 

#endif
