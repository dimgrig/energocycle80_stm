#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "main.h"

static uint8_t blink_delay = 0;

#define GATE_EN() \
	if (HAL_GPIO_ReadPin(OVERCURRENT_GPIO_Port, OVERCURRENT_Pin) == GPIO_PIN_SET) { \
		HAL_GPIO_WritePin(GATE_GPIO_Port, GATE_Pin, GPIO_PIN_SET); \
	} \

#define GATE_DIS() HAL_GPIO_WritePin(GATE_GPIO_Port, GATE_Pin, GPIO_PIN_RESET)
#define FAN_EN() HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET)
#define FAN_DIS() HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET)
#define FAN_DC_EN() HAL_GPIO_WritePin(FAN_DC_GPIO_Port, FAN_DC_Pin, GPIO_PIN_SET)
#define FAN_DC_DIS() HAL_GPIO_WritePin(FAN_DC_GPIO_Port, FAN_DC_Pin, GPIO_PIN_RESET)
#define LED_DC_ON() HAL_GPIO_WritePin(LED_DC_GPIO_Port, LED_DC_Pin, GPIO_PIN_SET)
#define LED_DC_OFF() HAL_GPIO_WritePin(LED_DC_GPIO_Port, LED_DC_Pin, GPIO_PIN_RESET)
#define LED_DC_BLINK() \
	if (blink_delay > 20) { \
		blink_delay = 0; \
		HAL_GPIO_TogglePin(LED_DC_GPIO_Port, LED_DC_Pin); \
	} else { \
		blink_delay++; \
	} \


#endif /* INC_GPIO_H_ */
