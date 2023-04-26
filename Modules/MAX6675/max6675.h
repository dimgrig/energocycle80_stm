#ifndef MAX6675_MAX6675_H_
#define MAX6675_MAX6675_H_

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "main.h"
#include "dwt.h"
#include "Energocycle.h"
#include "functions.h"

#define MAX6675_SS_SELECT() HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_RESET)
#define MAX6675_SS_DESELECT() HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_SET)
#define MAX6675_CLK_HIGH() HAL_GPIO_WritePin(MAX6675_CLK_GPIO_Port, MAX6675_CLK_Pin, GPIO_PIN_SET)
#define MAX6675_CLK_LOW() HAL_GPIO_WritePin(MAX6675_CLK_GPIO_Port, MAX6675_CLK_Pin, GPIO_PIN_RESET)
//#define MAX6675_SI_HIGH() HAL_GPIO_WritePin(MAX6675_SI_GPIO_Port, MAX6675_SI_Pin, GPIO_PIN_SET)
//#define MAX6675_SI_LOW() HAL_GPIO_WritePin(MAX6675_SI_GPIO_Port, MAX6675_SI_Pin, GPIO_PIN_RESET)

void MAX6675_init();
float MAX6675_get_temp();
float MAX6675_get_temp_slow();

//#ifdef __cplusplus
//}
//#endif

#endif /* MAX6675_MAX6675_H_ */


