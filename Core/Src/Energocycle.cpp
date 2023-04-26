#include "Energocycle.h"

#ifdef _UART_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
	extern UART_HandleTypeDef huart2;
#endif

Energocycle_settings_s esettings;
Energocycle_status_s estatus;
EC_FLASH_settings eflash_settings;
Energocycle_ADC_s eadc;
Energocycle_DEBUG_s edebug;
Energocycle_DEBUG_OUT_s edebug_out;

uint16_t test_ticks[I_CHANNELS];

float T_stored = 0.0;
unsigned int CNT_ENERGOCYCLE = 1;
uint8_t t_error_cnt = 0;

extern uint8_t MAX6675_started;

extern uint8_t LM92_started;
extern float LM92_temp;

extern uint8_t adc_started;

extern uint16_t ticks[I_CHANNELS];

float t_measure_max = 0.0;
float t_error = 0.0;

void energocycle_protection() {

#ifdef MAX6675_PROTECTION
#ifdef PWM_TICKS_DEBUG
	if (edebug.MAX6675) {
#endif
		if (MAX6675_started) {
			if (estatus.T < MAX6675_T_min) {
				if (t_error_cnt < 3)
					t_error_cnt++;
			} else if (estatus.T > MAX6675_T_max) {
				if (t_error_cnt < 3)
					t_error_cnt++;
			} else {
				if (t_error_cnt > 0)
					t_error_cnt--;
			}
		}

		if (t_error_cnt >= 3) {
			if (estatus.T < MAX6675_T_min) {
				estatus.state = E_state_to_error;
				estatus.error = E_T_low;
			} else if (estatus.T > MAX6675_T_max) {
				estatus.state = E_state_to_error;
				estatus.error = E_T_lost;
			}
		}
#ifdef PWM_TICKS_DEBUG
	}
#endif
#else
	estatus.T = 30.0;
#endif

#ifdef LM92_PROTECTION
#ifdef PWM_TICKS_DEBUG
	if (edebug.LM92) {
#endif
		if (LM92_started) {
			if (LM92_temp > LM92_T_max) {
				estatus.state = E_state_to_error;
				estatus.error = E_overheat;
			} else if (LM92_temp <= 0.0) {
				estatus.state = E_state_to_error;
				estatus.error = E_DC_T_lost;
			}
		}

#ifdef PWM_TICKS_DEBUG
		if (!edebug.PWM_ticks_enable) {
#endif
//			if ((estatus.state == E_state_calibration) || (estatus.state == E_state_to_calibration)) {
//				if ((LM92_temp > LM92_TEMP_FAN_CALIB) || (estatus.I > I_FAN_CALIB)) {
//					FAN_DC_EN();
//				}  else if ((LM92_temp < (LM92_TEMP_FAN_CALIB - 5)) && (estatus.I < (I_FAN_CALIB - 5))) {
//					FAN_DC_DIS();
//				}
//			} else {
				if ((LM92_temp > LM92_TEMP_FAN) || (estatus.I > I_FAN)) {
					FAN_DC_EN();
				}  else if ((LM92_temp < (LM92_TEMP_FAN - 5)) && (estatus.I < (I_FAN - 5))) {
					FAN_DC_DIS();
				}
//			}
#ifdef PWM_TICKS_DEBUG
		}
	}
#endif
#else
	//FAN_DC_EN();
#endif

//	if ((estatus.state == E_state_to_heat) || (estatus.state == E_state_heat) ||
//		(estatus.state == E_state_to_test) || (estatus.state == E_state_test) ||
//		(estatus.state == E_state_to_calibration) || (estatus.state == E_state_calibration)) {
		if ((estatus.error != E_I_overcurrent)
				&& (HAL_GPIO_ReadPin(OVERCURRENT_GPIO_Port, OVERCURRENT_Pin) == GPIO_PIN_RESET)
				//&& (HAL_GPIO_ReadPin(GATE_GPIO_Port, GATE_Pin) == GPIO_PIN_SET)
				) {
			energocycle_abort();

			estatus.state = E_state_to_error;
			estatus.error = E_I_overcurrent;
		}
//	}
}

void energocycle_heat_protection() {

//	if (CNT_ENERGOCYCLE > ADC_START_MEASUREMENTS) { //adc_started
//		if (estatus.U_in < U_in_min) {
//			estatus.state = E_state_to_error;
//			estatus.error = E_undervoltage;
//		}
//		if (estatus.U < esettings.Umin) {
//			estatus.state = E_state_to_error;
//			estatus.error = E_U_min;
//		}
//		if (estatus.I < esettings.Imin) {
//			estatus.state = E_state_to_error;
//			estatus.error = E_I_min;
//		}
//	}

	if ((eadc.started < 10) && (estatus.U > esettings.Umin) && (estatus.I > esettings.Imin)) {
		eadc.started = eadc.started + 1;
	}

	if (CNT_ENERGOCYCLE > ADC_START_MEASUREMENTS) { //if (eadc.started >= 10) {
		if (estatus.I < esettings.Imin) {
			estatus.state = E_state_to_error;
			estatus.error = E_I_min;
			energocycle_log();
		}
		if (estatus.I > esettings.Imax) {
			estatus.state = E_state_to_error;
			estatus.error = E_I_max;
			energocycle_log();
		}
		if (estatus.U < esettings.Umin) {
			estatus.state = E_state_to_error;
			estatus.error = E_U_min;
			energocycle_log();
		}
		if (estatus.U > esettings.Umax) {
			estatus.state = E_state_to_error;
			estatus.error = E_U_max;
			energocycle_log();
		}
	}

#ifdef MAX6675_PROTECTION
#ifdef PWM_TICKS_DEBUG
	if (edebug.MAX6675) {
#endif
		if ((CNT_ENERGOCYCLE % (int)(T_MEASURE_COMPARE_RATE / T_MEASURE_RATE)) == 0) {
			if (estatus.T < (T_stored + T_MEASURE_DELTA)) {
				estatus.state = E_state_to_error;
				estatus.error = E_T_measure;
			}
			T_stored = estatus.T;
		}
#ifdef PWM_TICKS_DEBUG
	}
#endif
#endif
	CNT_ENERGOCYCLE++;
}

void energocycle_cycle() {

	energocycle_protection();

#ifdef PWM_TICKS_DEBUG
	if (!edebug.PWM_ticks_enable) {
#endif
	switch (estatus.state) {
		case E_state_idle:

			break;
		case E_state_to_heat:
			pwm_update();
			pwm_start();
			HAL_Delay(1);

			GATE_EN();
			LED_DC_ON();
			FAN_DIS();

			CNT_ENERGOCYCLE = 1;
			T_stored = estatus.T;
			estatus.state = E_state_heat;
			break;
		case E_state_heat:
			pwm_update();

#ifdef TEMP_INERTIA_COMPENSATION
			if ((estatus.T >= estatus.T_OFF) && (estatus.T < MAX6675_T_max)) {
				estatus.state = E_state_to_cool;
			}
#else
			if ((estatus.T >= esettings.Tmax) && (estatus.T < MAX6675_T_max)) {
				estatus.state = E_state_to_cool;
			}
#endif

			energocycle_heat_protection();
			break;
		case E_state_to_cool:
			pwm_stop();
			GATE_DIS();
			LED_DC_OFF();
			eadc.started = 0;

			FAN_EN();

			estatus.state = E_state_cool;
			break;
		case E_state_cool:
			if ((esettings.Tmin) && (estatus.T < esettings.Tmin)) {
				if (estatus.energocycle_started) {
					if (estatus.energocycle_stoped) {

					} else {
						estatus.Cycles = estatus.Cycles + 1;
					}
				}

				FAN_DIS();

				estatus.state = E_state_to_done;
			}
			break;
		case E_state_to_error:
			pwm_stop();
			GATE_DIS();
			//LED_DC_OFF();
			eadc.started = 0;

			LED_DC_BLINK();

			if ((esettings.Tmin) && (estatus.T > esettings.Tmin)) {
				FAN_EN();
			} else {
				FAN_DIS();
			}

			estatus.state = E_state_error;
			break;
		case E_state_error:
			LED_DC_BLINK();

			if (estatus.T < esettings.Tmin) {
				FAN_DIS();
			}
			break;
		case E_state_to_done:
			if (estatus.energocycle_started) {
				if (estatus.energocycle_stoped) {
					estatus.state = E_state_ostanov;
				} else {
					if ((esettings.Cycles) && (estatus.Cycles < esettings.Cycles)) {
						estatus.state = E_state_to_heat;
					} else if (esettings.Cycles == 0) {
						estatus.state = E_state_idle;
					} else {
						estatus.Cycles = 0;
						estatus.energocycle_started = 0;
						estatus.energocycle_stoped = 0;
						estatus.state = E_state_done;
					}
				}
			} else {
				estatus.state = E_state_idle;
			}
			break;
		case E_state_done:

			break;
		case E_state_to_ostanov:
			pwm_stop();
			GATE_DIS();
			LED_DC_OFF();
			eadc.started = 0;

			if (estatus.energocycle_started) {
				estatus.energocycle_stoped = 1;
			}

			if ((esettings.Tmin) && (estatus.T > esettings.Tmin)) {
				FAN_EN();
			} else {
				FAN_DIS();
                if (estatus.energocycle_started) {
                    estatus.state = E_state_ostanov;
                } else {
                    estatus.state = E_state_idle;
                }
			}
			break;
		case E_state_ostanov:

			break;
		case E_state_to_test:
			pwm_update();
			pwm_start();
			HAL_Delay(1);

			GATE_EN();
			LED_DC_ON();
			FAN_DIS();

			CNT_ENERGOCYCLE = 1;
			T_stored = estatus.T;
			estatus.state = E_state_test;
			break;
		case E_state_test:
			pwm_update();

			if ((estatus.T >= esettings.Tmax) && (estatus.T < MAX6675_T_max)) {
				estatus.state = E_state_to_cool; //estatus.state = E_state_to_idle;
			}

			energocycle_heat_protection();
			break;
		case E_state_to_calibration:
			pwm_update();
			pwm_start();
			HAL_Delay(1);

			GATE_EN();
			LED_DC_ON();
			FAN_DIS();

			estatus.state = E_state_calibration;

			break;
		case E_state_calibration:
			pwm_update();
			break;
		case E_state_reset:
			LED_DC_OFF();

			estatus.error = E_no_error;
			estatus.state = E_state_to_ostanov; //E_state_to_done
			break;
		case E_state_to_idle:
			pwm_stop();
			GATE_DIS();
			LED_DC_OFF();
			eadc.started = 0;

			if ((esettings.Tmin) && (estatus.T > esettings.Tmin)) {
				FAN_EN();
			} else {
				FAN_DIS();
				estatus.state = E_state_idle;
			}

			break;
		case E_state_init:

			break;
		default:
			estatus.state = E_state_to_error;
			estatus.error = E_error_undefined;
			break;
	}

#ifdef TEMP_INERTIA_COMPENSATION
	energocycle_temp_off_calc();
#endif
#ifdef PWM_TICKS_DEBUG
    }
#endif

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
	//LOG("state %u \n", estatus.state);
#endif
}

void energocycle_debug() {

	if (edebug.PWM_ticks_enable) {

		test_ticks[0] = edebug.PWM_ticks_CH1;
		test_ticks[1] = edebug.PWM_ticks_CH2;
		test_ticks[2] = edebug.PWM_ticks_CH3;
		test_ticks[3] = edebug.PWM_ticks_CH4;

		pwm_set_ticks(test_ticks);
		pwm_start();
		HAL_Delay(1);

		if (edebug.FAN) {
			FAN_EN();
		} else {
			FAN_DIS();
		}
		if (edebug.FAN_DC) {
			FAN_DC_EN();
		} else {
			FAN_DC_DIS();
		}
		if (edebug.LED_DC) {
			LED_DC_ON();
		} else {
			LED_DC_OFF();
		}
		if (edebug.GATE) {
			GATE_EN();
		} else {
			GATE_DIS();
		}
	} else {
		test_ticks[0] = MAX_TICKS;
		test_ticks[1] = MAX_TICKS;
		test_ticks[2] = MAX_TICKS;
		test_ticks[3] = MAX_TICKS;

		pwm_set_ticks(test_ticks);
	}
}

void energocycle_start() {
    if ((estatus.state == E_state_idle) ||
    		(estatus.state == E_state_ostanov) ||
    		(estatus.state == E_state_done) ||
			(estatus.state == E_state_init)) {
        estatus.state = E_state_to_heat;
        estatus.Cycles = esettings.Cycle;

        estatus.T_OFF = esettings.Tmax;

        estatus.energocycle_started = 1;
        estatus.energocycle_stoped = 0;
    }
}

void energocycle_log() {
#ifdef _UART_DEBUG
	LOG("ERROR HANDLER\r\n");
	LOG("I1=%s%u.%02u I2=%s%u.%02u I3=%s%u.%02u I4=%s%u.%02u\r\n", FLOAT_PRINTF_SIGNED(eadc.I1), FLOAT_PRINTF_SIGNED(eadc.I2), FLOAT_PRINTF_SIGNED(eadc.I3), FLOAT_PRINTF_SIGNED(eadc.I4));
	LOG("Iavg=%s%u.%02u I=%s%u.%02u U1=%u.%02u U2=%u.%02u\r\n", FLOAT_PRINTF_SIGNED(eadc.I_avg), FLOAT_PRINTF_SIGNED(estatus.I), FLOAT_PRINTF(eadc.U1), FLOAT_PRINTF(eadc.U2));
	LOG("ticks: ");
	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
		LOG("%u; ", *(ticks + i));
	}
	LOG("\r\nGATE=%u OVERCUR=%u\r\n", HAL_GPIO_ReadPin(OVERCURRENT_GPIO_Port, OVERCURRENT_Pin), HAL_GPIO_ReadPin(GATE_GPIO_Port, GATE_Pin));
	LOG("ERROR HANDLER END\r\n");
#endif
}

void energocycle_abort() {
	GATE_DIS();

#ifdef PWM_TICKS_DEBUG
	edebug_out.overcurrent = 1;
#endif
}

void energocycle_temp_off_calc() {
	if (estatus.state == E_state_to_heat) {
		t_measure_max = estatus.T_OFF - 20;
	}
	if (estatus.T_OFF > 0) {
		if (t_measure_max < estatus.T) {
			t_measure_max = estatus.T;
		}
	}

	if ((estatus.state == E_state_to_error) || (estatus.state == E_state_error)) {
		t_measure_max = (esettings.Tmax + 0.0); // 0.25
	}
	if (estatus.state == E_state_to_done) {

		if ((t_measure_max == 0) || (estatus.T_OFF == 0)) {
			estatus.T_OFF = esettings.Tmax;
		} else {
			t_error = t_measure_max - (esettings.Tmax + 0.0); //0.25

			if (abs(t_error) > (esettings.Tmax / 5)) {
				estatus.T_OFF = estatus.T_OFF; //not valid error measure
			} else {
				estatus.T_OFF = estatus.T_OFF - t_error / 4; //2
			}

			if (abs(estatus.T_OFF - esettings.Tmax) > (esettings.Tmax / 5)) { //10
				estatus.T_OFF = esettings.Tmax;
			}
		}

//#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
//		LOG("TEMP_INERTIA_COMPENSATION "
//				"T_OFF=%u.%02u "
//				"t_measure_max=%u.%02u "
//				"t_error=%u.%02u\n",
//				FLOAT_PRINTF(estatus.T_OFF),
//				FLOAT_PRINTF(t_measure_max),
//				FLOAT_PRINTF(t_error));
//#endif

	}
}

void energocycle_write_flash() {
	WWDG->CR |= 0x7F; // Обновить значение таймера WWDG //
	energocycle_abort();

#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
	LOG("energocycle_write_flash\r\n");
#endif

	WWDG->CR |= 0x7F;

	eflash_settings.wwdg_fires = 0x09876543;
	eflash_settings.state = estatus.state;
	eflash_settings.energocycle_started = estatus.energocycle_started;
	eflash_settings.energocycle_stoped = estatus.energocycle_stoped;
	eflash_settings.error = estatus.error;
	eflash_settings.ewwdg_settings.Cycle = estatus.Cycles; //!!!
	eflash_settings.ewwdg_settings.Cycles = esettings.Cycles;
	eflash_settings.ewwdg_settings.DAC = esettings.DAC;
	eflash_settings.ewwdg_settings.Imax = esettings.Imax;
	eflash_settings.ewwdg_settings.Imin = esettings.Imin;
	eflash_settings.ewwdg_settings.Inom = esettings.Inom;
	eflash_settings.ewwdg_settings.Tmax = esettings.Tmax;
	eflash_settings.ewwdg_settings.Tmin = esettings.Tmin;
	eflash_settings.ewwdg_settings.Umax = esettings.Umax;
	eflash_settings.ewwdg_settings.Umin = esettings.Umin;
	eflash_settings.ewwdg_settings.Unom = esettings.Unom;
	eflash_settings.ewwdg_settings.mode = esettings.mode;
	eflash_settings.ewwdg_settings.res = esettings.res;
	eflash_settings.CNT_WWDG_RESETS = eflash_settings.CNT_WWDG_RESETS + 1;

	WWDG->CR |= 0x7F;
	FLASH_WriteSettings();
	WWDG->CR |= 0x7F;

#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
	LOG("energocycle_write_flash END\r\n");
	LOG("estatus.Cycles=%u\r\n", estatus.Cycles);
#endif

	WWDG->CR |= 0x7F;
}

void  energocycle_write_flash_C() {
	energocycle_write_flash();
};
