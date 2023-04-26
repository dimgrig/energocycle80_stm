#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <Energocycle.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "SEGGER_RTT.h"

void adc_init();
void adc_start();
uint8_t adc_get_zeroes();
void adc_get_values();

#endif /* INC_ADC_H_ */
