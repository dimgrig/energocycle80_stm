#ifndef LM92_LM92_H_
#define LM92_LM92_H_

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "main.h"
#include "dwt.h"
#include "functions.h"
#include "gpio.h"

#define TEPMERATURE_COEFFICIENT 1.48
#define TEPMERATURE_OF_ENVIROMENT 25

void LM92_init();
float LM92_get_temp();

void I2C_SetRegs(uint8_t DEV_ADDR);

//#ifdef __cplusplus
//}
//#endif

#endif /* LM92_LM92_H_ */
