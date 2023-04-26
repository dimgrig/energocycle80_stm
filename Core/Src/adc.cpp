#include "adc.h"

#ifdef _UART_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
	extern UART_HandleTypeDef huart2;
#endif

#define ADC_CHANNELS 6
#define NBR_OF_CONVERSIONS 100

uint16_t rawValues[ADC_CHANNELS * NBR_OF_CONVERSIONS];
uint32_t adcValues[6] = {0,0,0,0,0,0};
//uint8_t adc_started = 0;
float U_zero[4] = {0.0, 0.0, 0.0, 0.0};
uint8_t adc_started = 0;

extern ADC_HandleTypeDef hadc1;
extern Energocycle_status_s estatus;
extern EC_FLASH_settings eflash_settings;
extern Energocycle_ADC_s eadc;
extern Energocycle_DEBUG_OUT_s edebug_out;

void adc_init() {
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(200);
	adc_start();
}

void adc_start() {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, ADC_CHANNELS * NBR_OF_CONVERSIONS);
}

uint8_t adc_get_zeroes() {
	float U_zero_new[4] = {0.0, 0.0, 0.0, 0.0};
	U_zero_new[0] = (((float)adcValues[5]*U_ref/4096) - 0);
	U_zero_new[1] = (((float)adcValues[4]*U_ref/4096) - 0);
	U_zero_new[2] = (((float)adcValues[3]*U_ref/4096) - 0);
	U_zero_new[3] = (((float)adcValues[2]*U_ref/4096) - 0);
	eadc.I1 = 0;
	eadc.I2 = 0;
	eadc.I3 = 0;
	eadc.I4 = 0;

	uint8_t res = 0;
	for (uint8_t i = 0; i < 4; ++i) {
		if ((U_zero_new[i] > (U_ref/2 + U_ref*I_MEASURE_DELTA/100)) || (U_zero_new[i] < (U_ref/2 - U_ref*I_MEASURE_DELTA/100))) {
			res = 1;
			break;
		}
	}

	if (res == 0) {
		for (uint8_t i = 0; i < 4; ++i) {
			U_zero[i] = U_zero_new[i];
		}
	}
	return res;
}

void adc_get_values() {
	for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
		adcValues[i] = 0.0;
	}

	for (uint16_t j = 0; j < ADC_CHANNELS * NBR_OF_CONVERSIONS; j = j + ADC_CHANNELS) {
		for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
			adcValues[i] = adcValues[i] + rawValues[j + i];
		}
	}
	for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
		adcValues[i] = adcValues[i] / NBR_OF_CONVERSIONS;
	}

	eadc.U1 = (eflash_settings.ecalib_settings.U_CALIB_settings.K*(adcValues[0]*U_ref/4096) + eflash_settings.ecalib_settings.U_CALIB_settings.B)*U_SCALE_FACTOR;
	eadc.U2 = (adcValues[1]*U_ref/4096)*U_IN_SCALE_FACTOR;

	eadc.I1 = eflash_settings.ecalib_settings.I_CALIB_settings.K*(((float)adcValues[5]*U_ref/4096) - U_zero[0]) / GAIN_VA + eflash_settings.ecalib_settings.I_CALIB_settings.B;
	eadc.I2 = eflash_settings.ecalib_settings.I_CALIB_settings.K*(((float)adcValues[4]*U_ref/4096) - U_zero[1]) / GAIN_VA + eflash_settings.ecalib_settings.I_CALIB_settings.B;
	eadc.I3 = eflash_settings.ecalib_settings.I_CALIB_settings.K*(((float)adcValues[3]*U_ref/4096) - U_zero[2]) / GAIN_VA + eflash_settings.ecalib_settings.I_CALIB_settings.B;
	eadc.I4 = eflash_settings.ecalib_settings.I_CALIB_settings.K*(((float)adcValues[2]*U_ref/4096) - U_zero[3]) / GAIN_VA + eflash_settings.ecalib_settings.I_CALIB_settings.B;

	eadc.I_avg = (eadc.I1 + eadc.I2 + eadc.I3 + eadc.I4) / 4;

	estatus.U = eadc.U1;
	estatus.U_in = eadc.U2;
	estatus.I = eadc.I1 + eadc.I2 + eadc.I3 + eadc.I4;

#ifdef PWM_TICKS_DEBUG
	edebug_out.CH1 = eadc.I1;
	edebug_out.CH2 = eadc.I2;
	edebug_out.CH3 = eadc.I3;
	edebug_out.CH4 = eadc.I4;
#endif

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
	if ((estatus.state == E_state_heat) || (estatus.state == E_state_to_error) || (estatus.state == E_state_error)) {
		LOG("I1=%s%u.%02u I2=%s%u.%02u I3=%s%u.%02u I4=%s%u.%02u \r\n", //Iavg=%s%u.%02u I=%s%u.%02u%s U1=%s%u.%02u U2=%s%u.%02u
				FLOAT_PRINTF_SIGNED(eadc.I1), FLOAT_PRINTF_SIGNED(eadc.I2), FLOAT_PRINTF_SIGNED(eadc.I3), FLOAT_PRINTF_SIGNED(eadc.I4)//,
				//FLOAT_PRINTF_SIGNED(eadc.I_avg), FLOAT_PRINTF_SIGNED(estatus.I), FLOAT_PRINTF_SIGNED(eadc.U1), FLOAT_PRINTF_SIGNED(eadc.U2)
				);
	}
	//LOG("U=%u.%02u; I=%u.%02u; u=%u.%02u; ", FLOAT_PRINTF(estatus.U), FLOAT_PRINTF(estatus.I));
	//LOG("\r\n");
#endif
}

