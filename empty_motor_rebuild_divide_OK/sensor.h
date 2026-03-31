#ifndef __sensor_h
#define __sensor_h

#include "ti_msp_dl_config.h"
#include "motor.h"

//函数声明
void car_init(void);
void sensor_read(void);
void sensor_pid(void);

#endif