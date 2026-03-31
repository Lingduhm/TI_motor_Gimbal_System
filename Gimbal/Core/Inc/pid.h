#ifndef __PID_H
#define __PID_H

void pid_S_Y(float true_S, float tar_S);
void pid_S_X(float true_S, float tar_S);
extern unsigned int first_down;
void pid_trans(void);

#endif
