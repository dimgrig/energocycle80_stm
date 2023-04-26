#include "functions.h"

float map(float st1, float fn1, float st2, float fn2, float value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}

// бегущее среднее с адаптивным коэффициентом
float expRunningAverageAdaptive(float newVal, float filVal) {
    float k;
    // резкость фильтра зависит от модуля разности значений
    if (ABS((newVal - filVal)) > 10) k = 0.9;
    else k = 0.5;

    filVal += (newVal - filVal) * k;
    return filVal;
}

float temp_filter(uint8_t N, float value, float* temps_real, float* temps_avg, float expFilVal,
        int en_median, int en_avg, int en_exp) {

    float res = 0.0;
    float median = 0.0;

    int zeros = 0;
    for (uint8_t i = 0; i < N; ++i) {
        if (*(temps_real + i) == 0.0) {
            zeros++;
        }
    }
    if (zeros == N) {
        for (uint8_t i = 0; i < N; ++i) {
            *(temps_real +i) = value;
            *(temps_avg + i) = value;
        }
    }


    res = value;
    median = value;

    if (en_median) {
    	for (uint8_t i = 0; i < (N - 1); ++i) {
    		 *(temps_real + i) = *(temps_real + i + 1);
    	}
    	*(temps_real + N - 1) = res;

        if (N >= 3) {
            float a = temps_real[N - 3];
            float b = temps_real[N - 2];
            float c = temps_real[N - 1];
            if ((a <= b) && (a <= c)) median = (b <= c) ? b : c;
            else {
                if ((b <= a) && (b <= c)) median = (a <= c) ? a : c;
                else median = (a <= b) ? a : b;
            }
        }

        res = median;
    }

    if (en_avg) {
    	for (uint8_t i = 0; i < (N - 1); ++i) {
    		 *(temps_avg + i) = *(temps_avg + i + 1);
    	}
    	*(temps_avg + N - 1) = res;

        float avg = 0.0;
        for (uint8_t i = 0; i < N; ++i) {
            avg += *(temps_avg + i);
        }
        res = avg / N;
    }

    if (en_exp) {
        res = expRunningAverageAdaptive(res, expFilVal);
    }

    return res;
}
