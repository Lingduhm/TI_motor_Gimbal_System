#ifndef __motor_h
#define __motor_h

#include "ti_msp_dl_config.h"

void go_forward_a(int pwm);
void go_backward_a(int pwm);
void go_forward_b(int pwm);
void go_backward_b(int pwm);
//void go_forward_c(int pwm);
//void go_backward_c(int pwm);
//void go_forward_d(int pwm);
//void go_backward_d(int pwm);
//void go_stop_b();
void go_stop();

#endif