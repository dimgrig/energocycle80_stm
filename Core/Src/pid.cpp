#include "pid.h"

arm_pid_instance_f32 I_PID[I_CHANNELS];

extern Energocycle_settings_s esettings;
extern EC_FLASH_settings eflash_settings;
extern Energocycle_ADC_s eadc;

//extern "C" void arm_pid_init_f32(arm_pid_instance_f32 * S, int32_t resetStateFlag);
//extern "C" float32_t arm_pid_f32(arm_pid_instance_f32 * S, float32_t in);

void arm_pid_init_f32(
  arm_pid_instance_f32 * S,
  int32_t resetStateFlag)
{
  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float32_t) 2.0f * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if (resetStateFlag)
  {
    /* Reset state to zero, The size will be always 3 samples */
    memset(S->state, 0, 3U * sizeof(float32_t));
  }

}

void pid_init(void) {
	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
		I_PID[i].Kp = eflash_settings.epid_settings.I_PID_settings.Kp;
		I_PID[i].Ki = eflash_settings.epid_settings.I_PID_settings.Ki;
		I_PID[i].Kd = eflash_settings.epid_settings.I_PID_settings.Kd;
		arm_pid_init_f32(&I_PID[i], 1);
#if defined SEGGER_DEBUG || defined OCD_DEBUG
//		LOG("A0 %s%u.%02u - A1 %s%u.%02u - A2 %s%u.%02u \n",
//				FLOAT_PRINTF_SIGNED(I_PID[i].A0),
//				FLOAT_PRINTF_SIGNED(I_PID[i].A1),
//				FLOAT_PRINTF_SIGNED(I_PID[i].A2));
#endif
	}
}

void pid_I_iter(float* pid_outputs) {

	float I_measure[I_CHANNELS] = {
		eadc.I1,
		eadc.I2,
		eadc.I3,
		eadc.I4
	};

	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
		float pid_error = esettings.Inom / I_CHANNELS - I_measure[i];
		*(pid_outputs + i) = arm_pid_f32(&I_PID[i], pid_error);
	}
	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
		if (*(pid_outputs + i) > esettings.Imax / I_CHANNELS) {
			*(pid_outputs + i) = esettings.Imax / I_CHANNELS;
		} else if (*(pid_outputs + i) < 0) {
			*(pid_outputs + i) = 0;
		}
	}
}

void pig_get_dutys(float* pid_outputs, float* dutys) {

	float I_measure[I_CHANNELS] = {
		eadc.I1,
		eadc.I2,
		eadc.I3,
		eadc.I4
	};

	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
		*(dutys + i) = map(0, esettings.Imax / I_CHANNELS, 0, MAX_DUTY, *(pid_outputs + i));
	}

#if defined SEGGER_DEBUG || defined OCD_DEBUG
	LOG("PID target %u.%02u;\n", FLOAT_PRINTF(esettings.Inom / I_CHANNELS));
	for (uint8_t i = 0; i < I_CHANNELS; ++i) {
		LOG("%u - I %s%u.%02u - pid_output %s%u.%02u - duty %u.%02u\n",
				i, FLOAT_PRINTF_SIGNED(I_measure[i]), FLOAT_PRINTF_SIGNED(*(pid_outputs + i)), FLOAT_PRINTF(*(dutys + i)));
	}
#endif

}

void pid_I_update() {
	float pid_outputs[I_CHANNELS];
	float dutys[I_CHANNELS];

	pid_I_iter(pid_outputs);
	pig_get_dutys(pid_outputs, dutys);

	pwm_set_dutys(dutys);
}
