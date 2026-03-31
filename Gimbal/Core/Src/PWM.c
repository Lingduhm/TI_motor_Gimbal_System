#include "main.h"    
void PWM_SetCompare2(float Compare)
{
	TIM2->CCR3 = (uint16_t)Compare;
}
void PWM_SetCompare3(float Compare)
{
	TIM2->CCR2 = (uint16_t)Compare;
}
