#ifndef INC_PWM_H_
#define INC_PWM_H_

#include <Energocycle.h>
#include "stm32f1xx_hal.h"
#include <string.h>
#include "config.h"
#include "functions.h"
#include "SEGGER_RTT.h"
#include <math.h>

#define HIGH_LIMIT 22
#define LOW_LIMIT 0

#define I_IN_TICK (HIGH_LIMIT - LOW_LIMIT) / MAX_TICKS
#define DIVISION_FACTOR 4
#define MAX_DELTA_TICK 1

void pwm_shift();
void pwm_init();
void pwm_set_up();
void pwm_start();
//void pwm_set_dutys(float* dutys);
void pwm_set_tick(uint16_t tick);
void pwm_set_ticks(uint16_t* ticks);
void pwm_stop();
//void pwm_balance_all_ticks();
void pwm_balance_min_max_ticks();
void pwm_update();

#endif /* INC_PWM_H_ */
