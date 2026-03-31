#include "main.h"                  // Device header
#include "PWM.h"
#include "pid.h"
#include "math.h"
extern unsigned int first_down;
extern unsigned char flag;
extern unsigned int now_2;
extern unsigned char flag_3;
float Err_X = 0,Err_Y = 0;
float x_pwm = 0,
		now_x;
float y_pwm = 0,
		now_y,
		ks1 = 1500,
		ks2 = 2350;
		
float Err_S_Y = 0,
	  last_Err_S_Y = 0,
	  integral_Y = 0,
	  p_S = -0.09,     
	  i_S = -0.01,    
	  d_S = -0.02;

//float Err_S_X = 0,
//	  last_Err_S_X = 0,
//    integral_X = 0,
//	  p_S_X = 0.00018,
//	  i_S_X = 0.00,   
//	  d_S_X = 0.78,
//	  //【新增】前馈控制系数
//	  feedforward_gain = 0.00015;  // 前馈增益，可调整  


float Err_S_X = 0,
	  last_Err_S_X = 0,
    integral_X = 0,
	  p_S_X = 0.0002,//18
	  i_S_X = 0.000000007,   
	  d_S_X = 1 ,//0.118
	  //【新增】前馈控制系数
	  feedforward_gain = 0.0000018;  // 前馈增益，可调整  

void pid_trans()
{
	if(flag==1)
	{
		p_S_X = 0.0002,//18
	  i_S_X = 0.000000007,   
	  d_S_X = 1 ,//0.118
	  //【新增】前馈控制系数
	  feedforward_gain = 0.0000018;  // 前馈增益，可调整  
	}
	else if(flag==2)
	{
		p_S_X = 0.00004,//18
	  i_S_X = 0.000000001,   
	  d_S_X = -0.000015 ,//0.118
	  //【新增】前馈控制系数
	  feedforward_gain = 0.00000;  // 前馈增益，可调整  
		if(flag_3==1)
		{
		  ks2=now_2;
			flag_3=0;
		}
	}
}
void pid_S_X(float true_S, float tar_S)
{
		Err_S_X = tar_S - true_S;
		
		//【新增】前馈控制 - 预测目标变化
		static float last_target = 0;
		float target_velocity = tar_S - last_target;  // 目标变化速度
		float feedforward = target_velocity * feedforward_gain;  // 前馈输出
		last_target = tar_S;  // 更新上次目标值
		
    // 【新增】动态积分系数
    float dynamic_i_gain = i_S_X;
    if(fabs(Err_S_X) > 20.0f) {
        // 大误差时（转弯）使用较大积分
        dynamic_i_gain = i_S_X * 1.0f;  // 可调整倍数
    } else {
        // 小误差时减小积分，减少震荡
        dynamic_i_gain = i_S_X * 1.0f;
    }
    //x轴死区
    if(fabs(Err_S_X) <1.2f) {
        Err_S_X = 0;
        integral_X *= 0.7f;
    } else {
        integral_X += Err_S_X;
    }
    
    // 使用动态积分系数
    x_pwm = p_S_X * Err_S_X + d_S_X * (Err_S_X - last_Err_S_X) + dynamic_i_gain * integral_X + feedforward;
		
		last_Err_S_X = Err_S_X;
		now_x = ks2 + x_pwm;
		
		if(now_x > 2500)
		{
			now_x = 2500;
		}
		else if(now_x < 500)
		{
			now_x = 500;
		}
		ks2 = now_x;
    PWM_SetCompare3((int)now_x);
//		OLED_ShowNum(1, 1, now_x, 4);
//	  OLED_ShowSignedNum(2, 1, x_pwm*1000, 6);
}
void pid_S_Y(float true_S, float tar_S)
{
	
    // 【新增】Y轴死区处理，与X轴保持一致
    if(fabs(Err_S_Y) < 30.0f) {
        Err_S_Y = 0;
        // 【修改】死区内积分项缓慢衰减，避免突变
        integral_Y *= 0.9f;
    } else {
        // 【修改】积分计算移到else内，避免死区内继续积分
        integral_Y += Err_S_Y;
        
////        // 【新增】Y轴积分限幅，防止积分饱和
////        if(integral_Y > 50.0f) {
////            integral_Y = 50.0f;
////        } else if(integral_Y < -50.0f) {
////            integral_Y = -50.0f;
////        }
    }
		y_pwm=p_S * Err_S_Y+d_S*(Err_S_Y-last_Err_S_Y)+i_S * integral_Y;
		last_Err_S_Y = Err_S_Y;
		now_y = ks1 + y_pwm;
		if(now_y > 2000)
		{
			 now_y = 2000;
		}
		else if(now_y < 1000)
		{
		   now_y = 1000;
		}
		ks1 = now_y;
		PWM_SetCompare2(now_y);		
		//OLED_ShowNum(2, 1, now_y, 4);
}
