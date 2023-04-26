#ifndef SRC_PID_H_
#define SRC_PID_H_

#include <controller_functions.h>
#include <Energocycle.h>

#include "config.h"
#include "functions.h"
#include "SEGGER_RTT.h"
#include "pwm.h"

void pid_init(void);
void pid_iter(float target, float value);
void pid_I_update();

#endif /* SRC_PID_H_ */
