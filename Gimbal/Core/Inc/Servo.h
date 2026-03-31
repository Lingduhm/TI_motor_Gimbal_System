#ifndef _SERVO_H
#define _SERVO_H

#define MIN_ANGLE_X 0 
#define MAX_ANGLE_X 180
#define MIN_ANGLE_Y 0  
#define MAX_ANGLE_Y 270 

void Servo_SetAngle1(float Angle1);
void Servo_SetAngle2(float Angle2);
uint16_t PWM_GetCompare2(void); 
uint16_t PWM_GetCompare3(void);  

#endif
