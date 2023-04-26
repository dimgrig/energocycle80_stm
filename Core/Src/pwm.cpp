#include "pwm.h"


#ifdef _UART_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
	extern UART_HandleTypeDef huart2;
#endif

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern Energocycle_settings_s esettings;
extern Energocycle_ADC_s eadc;
extern Energocycle_DEBUG_OUT_s edebug_out;

uint8_t started = 0;

uint16_t ticks[I_CHANNELS];

float I_measured[I_CHANNELS];
float I_delta[I_CHANNELS];
float I_ticks[I_CHANNELS];
int delta_ticks[I_CHANNELS];

float error;
uint8_t min_index = 0;
uint8_t max_index = 0;
uint8_t balanced = 0;
float Inom = 0;

//void pwm_set(float duty, uint16_t* pwm) {
//    if (duty < 0) duty = 0;
//    if (duty > MAX_DUTY) duty = MAX_DUTY;
//
//    uint16_t MAX_PWM = __HAL_TIM_GET_AUTORELOAD(&htim1);
//    *pwm = map(0, MAX_DUTY, 0, MAX_PWM, duty);
//}

void pwm_shift() {
	uint16_t MAX_PWM = __HAL_TIM_GET_AUTORELOAD(&htim1);
	uint16_t tim1_shift = 0; //MAX_PWM - 1;
	uint16_t tim2_shift = MAX_PWM * 1 / I_CHANNELS;
	uint16_t tim3_shift = MAX_PWM * 2 / I_CHANNELS;
	uint16_t tim4_shift = MAX_PWM * 3 / I_CHANNELS;

	__HAL_TIM_SET_COUNTER(&htim1, tim1_shift);
	__HAL_TIM_SET_COUNTER(&htim2, tim2_shift);
	__HAL_TIM_SET_COUNTER(&htim3, tim3_shift);
	__HAL_TIM_SET_COUNTER(&htim4, tim4_shift);

	HAL_Delay(10);

#if defined SEGGER_DEBUG || defined OCD_DEBUG
	LOG("MAX_PWM %u; htim1 %u; htim2 %u; htim3 %u; htim4 %u; \n", MAX_PWM, tim1_shift, tim2_shift, tim3_shift, tim4_shift);
#endif
}


void pwm_init() {

	started = 0;
	for (int i = 0; i < I_CHANNELS; ++i) {
		ticks[i] = (uint16_t)MAX_TICKS;
	}
}

void pwm_set_up() {

	pwm_set_ticks(ticks);

	if (started != 1) {
		pwm_shift();
	}

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void pwm_start() {

	if (started != 1) {
		pwm_shift();
	}

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	started = 1;
}

void pwm_set_tick(uint16_t tick) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tick);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tick);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, tick);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, tick);

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
//	LOG("CCR ");
//	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
//		LOG("%u; ", *(ticks + i));
//	}
//	LOG("\n");
#endif
}

void pwm_set_ticks(uint16_t* ticks) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *(ticks + 0));
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, *(ticks + 1));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, *(ticks + 2));
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, *(ticks + 3));

#ifdef PWM_TICKS_DEBUG
	edebug_out.CCR1 = *(ticks + 0);
	edebug_out.CCR2 = *(ticks + 1);
	edebug_out.CCR3 = *(ticks + 2);
	edebug_out.CCR4 = *(ticks + 3);
#endif

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
//	LOG("CCR ");
//	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
//		LOG("%u; ", *(ticks + i));
//	}
//	LOG("\n");
#endif
}

//void pwm_set_dutys(float* dutys) {
//	uint16_t pwms[I_CHANNELS];
//
//	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
//		pwm_set(*(dutys + i), pwms + i);
//	}
//
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *(pwms + 0));
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, *(pwms + 1));
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, *(pwms + 2));
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, *(pwms + 3));
//
//#if defined SEGGER_DEBUG || defined OCD_DEBUG
////	LOG("CCR ");
////	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
////		LOG("%u; ", *(pwms + i));
////	}
////	LOG("\n");
//#endif
//}

void pwm_smooth_stop() {

	uint8_t stoped = 0;
	while (!stoped) {
		stoped = 1;
		for (int i = 0; i < I_CHANNELS; ++i) {
			if (ticks[i] < (uint16_t)MAX_TICKS) {
				ticks[i] = ticks[i] + 1;
				stoped = 0;
			}
		}
		pwm_set_ticks(ticks);
		DWT_Delay(10);
	}

}

void pwm_stop() {

	pwm_smooth_stop();

//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

	started = 0;
	for (int i = 0; i < I_CHANNELS; ++i) {
		ticks[i] = (uint16_t)MAX_TICKS;
	}
}

//void pwm_balance_all_ticks() {
//
//	if (balanced == 0) {
//
//		for (int i = 0; i < I_CHANNELS; ++i) {
//			float error_ = (esettings.Inom / 4 - I_measured[i]) / DIVISION_FACTOR;
//			I_delta[i] = (eadc.I_avg - I_measured[i]) / DIVISION_FACTOR + error_;
//			I_ticks[i] = I_delta[i] / ((float)I_IN_TICK);
//			delta_ticks[i] = round(I_ticks[i]);
//		}
//
//		for (int i = 0; i < I_CHANNELS; ++i) {
//			if (delta_ticks[i] >= MAX_DELTA_TICK) {
//				delta_ticks[i] = MAX_DELTA_TICK;
//			}
//			if (delta_ticks[i] <= -MAX_DELTA_TICK) {
//				delta_ticks[i] = -MAX_DELTA_TICK;
//			}
//		}
//
//		for (int i = 0; i < I_CHANNELS; ++i) {
//			int delta = 0;
//			if (balanced == 0) {
//				delta = (int)ticks[i] - delta_ticks[i];
//			} else {
//				delta = (int)ticks[i];
//			}
//			if (delta <= 0) {
//				ticks[i] = 0;
//			} else if (delta >= MAX_TICKS) {
//				ticks[i] = MAX_TICKS;
//			} else {
//				ticks[i] = delta;
//			}
//		}
//	}
//}

void pwm_balance_min_max_ticks() {

	min_index = 0;
	max_index = 0;

	// min - max swaped
	for (int i = 0; i < I_CHANNELS - 1; ++i) {
		if (I_measured[i + 1] > I_measured[max_index])
			max_index = i + 1;
		if (I_measured[i + 1] < I_measured[min_index])
			min_index = i + 1;
	}

	if (error > 0) {
		if ((ticks[min_index] - 1) <= 0) {
			ticks[min_index] = 0;
		} else {
			ticks[min_index] = ticks[min_index] - 1;
		}
	}
	if (error < 0) {
		if ((ticks[max_index] + 1) >= MAX_TICKS) {
			ticks[max_index] = MAX_TICKS;
		} else {
			ticks[max_index] = ticks[max_index] + 1;
		}
	}
}

void pwm_update() {

	switch (esettings.mode) {
		case E_mode_I:
		case E_mode_U:
			error = (esettings.Inom / 4 - eadc.I_avg) / DIVISION_FACTOR;
		break;
		case E_mode_P:
			error = ((esettings.Inom / 4) * esettings.Unom - eadc.I_avg * eadc.U1) / DIVISION_FACTOR;
		break;
	}

	//error = (esettings.Inom / 4 - eadc.I_avg) / DIVISION_FACTOR;

	I_measured[0] = eadc.I1;
	I_measured[1] = eadc.I2;
	I_measured[2] = eadc.I3;
	I_measured[3] = eadc.I4;

	balanced = 1;

	float I_avg_abs = 0.0;

	for (int i = 0; i < I_CHANNELS; ++i) {
		I_avg_abs = I_avg_abs + abs(I_measured[i]);
	}
	I_avg_abs = I_avg_abs / 4;

//	//for (int i = 0; i < I_CHANNELS; ++i) {
//		if (I_avg_abs > I_BALANCE_MIN) {
//			;
//		} else {
//			//if (abs(I_measured[i]) < I_BALANCE_MIN) {
//				balanced = 0;
//			//	break;
//			//}
//		}
//	//}
	if (I_avg_abs < I_BALANCE_MIN) {
		balanced = 0;
	}

	if (balanced == 1) {
		pwm_balance_min_max_ticks();
	} else {
		for (int i = 0; i < I_CHANNELS; ++i) {
			if ((ticks[i] - 1) > 0) {
				ticks[i] = ticks[i] - 1;
			}
		}
	}

	//set pwm_ticks directly
//		for (int i = 0; i < I_CHANNELS; ++i) {
//			ticks[i] = (uint16_t)(esettings.DAC & 0xFFFF);
//		}

	pwm_set_ticks(ticks);
}
