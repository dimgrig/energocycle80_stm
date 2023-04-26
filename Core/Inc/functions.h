#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "main.h"

#define ABS(x) (((x)) > 0 ? ((x)) : -((x)))
float map(float st1, float fn1, float st2, float fn2, float value);
float expRunningAverageAdaptive(float newVal, float filVal);

float temp_filter(uint8_t N, float value, float* temps_real, float* temps_avg, float expFilVal,
        int en_median, int en_avg, int en_exp);

#endif /* INC_FUNCTIONS_H_ */
