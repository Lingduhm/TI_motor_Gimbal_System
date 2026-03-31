#ifndef __TASK_H
#define __TASK_H

#include "ti_msp_dl_config.h"
#include "sensor.h"
#include "key.h"

// 外部变量声明
extern volatile uint8_t g_flag_need_update;
extern volatile uint8_t g_raw_key;
extern volatile uint32_t g_isr_tick;
// 外部变量声明
extern unsigned char turn_completed_flag;

// 函数声明
void TaskManager_Init(void);
void TaskManager_Update(void);

#endif