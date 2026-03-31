#include "motor.h"

//PWM限幅函数 *a传入要限幅的参数  ABS_MAX限幅大小
void PWM_Limit(int *a, int ABS_MAX){
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < 0)
    *a = 0;
}

// /**
//  * @brief      设置定时器捕获比较值
//  *
//  * @param[in]  gptimer        外设寄存器结构体指针（指向哪个定时器）
//  * @param[in]  value          要写入捕获比较寄存器的值（就是你说的 CCR / 占空比值）
//  * @param[in]  ccIndex        捕获比较通道索引（指定设置哪个通道：通道0、通道1...）
//  *
//  */
// void DL_Timer_setCaptureCompareValue(GPTIMER_Regs *gptimer, uint32_t value, DL_TIMER_CC_INDEX ccIndex);
// value类型为uint32_t，因此不能传入负值。

// 设置电机A_A的PWM
void Set_MotorA_chanelA_PWM(int Target_PWM){
	PWM_Limit(&Target_PWM, 99);
	DL_TimerG_setCaptureCompareValue(Motor_A_INST, Target_PWM, GPIO_Motor_A_C0_IDX);
}
// 设置电机A_B的PWM
void Set_MotorA_chanelB_PWM(int Target_PWM){
	PWM_Limit(&Target_PWM, 99);
	DL_TimerG_setCaptureCompareValue(Motor_A_INST, Target_PWM, GPIO_Motor_A_C1_IDX);
}

// 设置电机B_A的PWM
void Set_MotorB_chanelA_PWM(int Target_PWM){
	PWM_Limit(&Target_PWM, 99);
	DL_TimerA_setCaptureCompareValue(Motor_B_INST, Target_PWM, GPIO_Motor_B_C0_IDX);
}
// 设置电机B_B的PWM
void Set_MotorB_chanelB_PWM(int Target_PWM){
	PWM_Limit(&Target_PWM, 99);
	DL_TimerA_setCaptureCompareValue(Motor_B_INST, Target_PWM, GPIO_Motor_B_C1_IDX);
}

//// 设置电机C_A的PWM
//void Set_MotorC_chanelA_PWM(int Target_PWM){
//	PWM_Limit(&Target_PWM,199);
//	DL_TimerA_setCaptureCompareValue(Motor_C_INST,Target_PWM,GPIO_Motor_C_C0_IDX);
//}
//// 设置电机C_B的PWM
//void Set_MotorC_chanelB_PWM(int Target_PWM){
//	PWM_Limit(&Target_PWM,199);
//	DL_TimerA_setCaptureCompareValue(Motor_C_INST,Target_PWM,GPIO_Motor_C_C1_IDX);
//}

//// 设置电机D_A的PWM
//void Set_MotorD_chanelA_PWM(int Target_PWM){
//	PWM_Limit(&Target_PWM,199);
//	DL_TimerA_setCaptureCompareValue(Motor_B_INST,Target_PWM,GPIO_Motor_B_C0_IDX);
//}
//// 设置电机D_B的PWM
//void Set_MotorD_chanelB_PWM(int Target_PWM){
//	PWM_Limit(&Target_PWM,199);
//	DL_TimerA_setCaptureCompareValue(Motor_B_INST,Target_PWM,GPIO_Motor_B_C3_IDX);
//}

void go_forward_a(int pwm){
	//A轮_前进
	//前进为10
  	Set_MotorA_chanelA_PWM(pwm);
	Set_MotorA_chanelB_PWM(0);
}

void go_backward_a(int pwm){
	//A轮_后退
	//后退为01
  	Set_MotorA_chanelA_PWM(0);
    Set_MotorA_chanelB_PWM(pwm);
}

void go_forward_b(int pwm){
	//B轮_前进
	//前进为01
  	Set_MotorB_chanelA_PWM(0);
	Set_MotorB_chanelB_PWM(pwm);
}

void go_backward_b(int pwm){
    //B轮_后退
	//后退为10
  	Set_MotorB_chanelA_PWM(pwm);
	Set_MotorB_chanelB_PWM(0);
}

//void go_backward_c(int pwm){
//	 //C轮_前进
//	 //前进为01
//  	Set_MotorC_chanelA_PWM(0);
//	  Set_MotorC_chanelB_PWM(pwm);
//}

//void go_forward_c(int pwm){
//	 //C轮_后退
//	 //后退为10
//  	Set_MotorC_chanelA_PWM(pwm);
//	  Set_MotorC_chanelB_PWM(0);
//}

//void go_backward_d(int pwm){
//	 //D轮_前进
//	 //前进为10
//  	Set_MotorD_chanelA_PWM(pwm);
//	  Set_MotorD_chanelB_PWM(0);
//}

//void go_forward_d(int pwm){
//	 //D轮_后退
//	 //后退为01
//  	Set_MotorD_chanelA_PWM(0);
//	  Set_MotorD_chanelB_PWM(pwm);
//}

//  void go_stop_b(){
//	 //A轮
//  	Set_MotorA_chanelA_PWM(0);
//	  Set_MotorA_chanelB_PWM(0);
// 	 //B轮
//   	Set_MotorB_chanelA_PWM(0);
// 	  Set_MotorB_chanelB_PWM(0);
//	 //C轮
//  	Set_MotorC_chanelA_PWM(0);
//	  Set_MotorC_chanelB_PWM(0);
//	 //D轮
//  	Set_MotorD_chanelA_PWM(0);
//	  Set_MotorD_chanelB_PWM(0);
//  }

  void go_stop(){
	//A轮
  	Set_MotorA_chanelA_PWM(0);
	Set_MotorA_chanelB_PWM(0);
	//B轮
  	Set_MotorB_chanelA_PWM(0);
	Set_MotorB_chanelB_PWM(0);
//	 //C轮
//  	Set_MotorC_chanelA_PWM(0);
//	  Set_MotorC_chanelB_PWM(0);
//	 //D轮
//  	Set_MotorD_chanelA_PWM(0);
//	  Set_MotorD_chanelB_PWM(0);
 }