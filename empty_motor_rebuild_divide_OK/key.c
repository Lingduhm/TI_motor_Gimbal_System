#include "key.h"

unsigned char read_key() //读取当前按键
{
	if(DL_GPIO_readPins(KEY_1_PORT, KEY_1_PIN_15_PIN) == 0)          
	{
		return 1;
	} 
	else if(DL_GPIO_readPins(KEY_2_PORT, KEY_2_PIN_16_PIN) == 0)     
	{
		return 2;
	}
	else if(DL_GPIO_readPins(KEY_3_PORT, KEY_3_PIN_2_PIN) == 0)       
	{
		return 3;
	}
	else if(DL_GPIO_readPins(KEY_4_PORT, KEY_4_PIN_3_PIN) == 0)      
	{
		return 4;
	}
  else if(DL_GPIO_readPins(KEY_5_PORT, KEY_5_PIN_21_PIN) == 0)      
	{
		return 5;
	}
	else
	{
		return 0; //未按下则返回0
	}
}
