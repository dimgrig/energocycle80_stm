#include "lm92.h"

extern I2C_HandleTypeDef LM92_hi2c;

#define LM92_N 5
uint8_t LM92_started = 0;
float LM92_temp = 0.0;
float LM92_temps_real[LM92_N];
float LM92_temps_avg[LM92_N];
float LM92_expFilVal = 0.0;

uint8_t Temperature_REG 	= 0x00;
uint8_t Configuration_REG 	= 0x01;
uint8_t THYST_REG 			= 0x02;
uint8_t T_CRIT_REG 			= 0x03;
uint8_t TLOW_REG 			= 0x04;
uint8_t THIGH_REG 			= 0x05;
uint8_t ManufacturerID_REG 	= 0x07;



void LM92_init() {
	for(uint8_t i = 0; i < LM92_N; ++i) {
		LM92_temps_real[i] = 0.0;
		LM92_temps_avg[i] = 0.0;
	}
	FAN_DC_DIS();

	//I2C_SetRegs(LM92_ADDR);
}

void I2C_SetReg(uint8_t DEV_ADDR, uint8_t REG, float temp) {

	uint8_t aRxTxBuffer[4] = {0x00, 0x00, 0x00, 0x00};

	aRxTxBuffer[0] = REG;
	uint16_t temp_ = ((uint16_t)(temp / 0.0625)) << 3;
	aRxTxBuffer[1] = (uint8_t)((temp_ >> 8) & 0xFF);
	aRxTxBuffer[2] = (uint8_t)(temp_ & 0xFF);
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(&LM92_hi2c, (uint16_t)(DEV_ADDR | 0x01), (uint8_t *)&aRxTxBuffer, 3, (uint32_t)100);
	HAL_Delay(10);
	res = HAL_I2C_Master_Receive(&LM92_hi2c, (uint16_t)(DEV_ADDR | 0x00), (uint8_t *)&aRxTxBuffer, 2, (uint32_t)100);
	volatile float T_SETTED = (((aRxTxBuffer[0] << 5) | (aRxTxBuffer[1] >> 3)) * 0.0625);
	T_SETTED = T_SETTED + 1;
}

void I2C_SetRegs(uint8_t DEV_ADDR) {

	I2C_SetReg(DEV_ADDR, T_CRIT_REG, LM92_T_CRIT);
	I2C_SetReg(DEV_ADDR, TLOW_REG, LM92_TLOW);
	I2C_SetReg(DEV_ADDR, THIGH_REG, LM92_THIGH);

	//reset pointer
    uint8_t aRxTxBuffer[4] = {0x00, 0x00, 0x00, 0x00};
	aRxTxBuffer[0] = Temperature_REG;
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(&LM92_hi2c, (uint16_t)(DEV_ADDR | 0x01), (uint8_t *)&aRxTxBuffer, 1, (uint32_t)100);
}

void I2C_ReadTemperature(uint8_t DEV_ADDR) {

  uint8_t trys = 1; //6
  uint8_t aRxBuffer[2] = {0x00, 0x00};

  HAL_StatusTypeDef res = HAL_I2C_Master_Receive(&LM92_hi2c, (uint16_t)DEV_ADDR, (uint8_t *)&aRxBuffer, 2, (uint32_t)100);
//  while (res != HAL_OK) {
//    res = HAL_I2C_Master_Receive(&LM92_hi2c, (uint16_t)DEV_ADDR, (uint8_t *)&aRxBuffer, 8, (uint32_t)100); //size 8???
//    trys--;
//    if (trys <= 3 && trys > 0) {
//      if (HAL_I2C_DeInit(&LM92_hi2c) != HAL_OK) {
//        __NOP();
//      }
//      if (HAL_I2C_Init(&LM92_hi2c) != HAL_OK) {
//        __NOP();
//      }
//    } else if (trys <= 0) {
//      memset(aRxBuffer, 0, sizeof aRxBuffer);
//      break;
//    }
//  }

  LM92_temp = (((aRxBuffer[0] << 5) | (aRxBuffer[1] >> 3)) * 0.0625); // - TEPMERATURE_OF_ENVIROMENT) * TEPMERATURE_COEFFICIENT + TEPMERATURE_OF_ENVIROMENT
  LM92_temp = temp_filter(LM92_N, LM92_temp, LM92_temps_real, LM92_temps_avg, LM92_expFilVal, 1, 1, 0);
}

float LM92_get_temp() {

	I2C_ReadTemperature(LM92_ADDR);
	LM92_started = 1;

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
	//LOG("%u.%02u C\r\n", FLOAT_PRINTF(LM92_temp));
#endif

	return LM92_temp;
}

