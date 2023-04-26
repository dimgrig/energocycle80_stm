#include "max6675.h"

extern SPI_HandleTypeDef MAX6675_hspi;
extern EC_FLASH_settings eflash_settings;
extern Energocycle_status_s estatus;

#define MAX6675_N 4
uint8_t MAX6675_started = 0;
float MAX6675_temps_real[MAX6675_N];
float MAX6675_temps_avg[MAX6675_N];
float MAX6675_expFilVal = 0.0;

uint8_t MAX6675_DATA_PORT() {
  return HAL_GPIO_ReadPin(MAX6675_SO_GPIO_Port, MAX6675_SO_Pin);
}

void MAX6675_init() {
	FAN_DIS();
	for(uint8_t i = 0; i < MAX6675_N; ++i) {
		MAX6675_temps_real[i] = 0.0;
		MAX6675_temps_avg[i] = 0.0;
	}
	MAX6675_SS_DESELECT();
	HAL_Delay(300); //need for MAX6675 ready + 250ms between get_temp calls!!!
}

float MAX6675_get_temp() {
	uint8_t sendbytes[2] = {0, 0};
	uint8_t receivedbytes[2] = {0, 0};

	MAX6675_SS_SELECT();
	HAL_SPI_TransmitReceive(&MAX6675_hspi, (uint8_t*) &sendbytes,
				(uint8_t*) &receivedbytes, 2, 0x1000);
	MAX6675_SS_DESELECT();

	float temp = eflash_settings.ecalib_settings.T_CALIB_settings.K*((float)(((receivedbytes[0] << 8) | receivedbytes[1]) >> 3) * 0.25) +
			eflash_settings.ecalib_settings.T_CALIB_settings.B;

	MAX6675_started = 1;
	return temp;
}

float MAX6675_get_temp_slow() {
	//uint32_t st,en;
	//st=DWT_Get();

	uint8_t receivedbytes[2] = {0x00, 0x00};

    uint8_t bit_count;
    uint32_t portdata;

//    MAX6675_SS_DESELECT();
//    DWT_Delay(200);
//    MAX6675_CLK_LOW();
//    DWT_Delay(200);
	MAX6675_SS_SELECT();
	DWT_Delay(100);

	for (bit_count = 0; bit_count < 16; bit_count++)
	{
		DWT_Delay(100);
		MAX6675_CLK_HIGH();
		DWT_Delay(100);

		// read the port data
		portdata = MAX6675_DATA_PORT();
		if (bit_count < 8) {
			receivedbytes[0] = (receivedbytes[0] << 1);
			receivedbytes[0] = receivedbytes[0] | portdata;
		} else {
			receivedbytes[1] = (receivedbytes[1] << 1);
			receivedbytes[1] = receivedbytes[1] | portdata;
		}


		// falling edge on clock port

		DWT_Delay(100);
		MAX6675_CLK_LOW();
		DWT_Delay(100);

		if (bit_count == 7) {
			DWT_Delay(200);
		}
	}

	DWT_Delay(10);
	MAX6675_SS_DESELECT();
	MAX6675_CLK_LOW();

	//HAL_SPI_TransmitReceive(&MAX6675_hspi, (uint8_t*) &sendbytes,
	//			(uint8_t*) &receivedbytes, 2, 0x1000);

	float temp = 0.0;
	if (((receivedbytes[1] >> 2) & 0x01) == 0) {
		temp = eflash_settings.ecalib_settings.T_CALIB_settings.K*((float)((((receivedbytes[0] & 0xFF) << 8) | (receivedbytes[1] & 0xFF)) >> 3) * 0.25) +
				eflash_settings.ecalib_settings.T_CALIB_settings.B;
		if ((estatus.state == E_state_to_calibration) || (estatus.state == E_state_calibration)){
			for(uint8_t i = 0; i < MAX6675_N; ++i) {
				MAX6675_temps_real[i] = 0.0;
				MAX6675_temps_avg[i] = 0.0;
			}
		} else {
			temp = temp_filter(MAX6675_N, temp, MAX6675_temps_real, MAX6675_temps_avg, MAX6675_expFilVal, 1, 0, 0);
		}
	} else {
		temp = MAX6675_temps_real[MAX6675_N-1];
		estatus.state = E_state_to_error;
		estatus.error = E_T_lost;
	}


#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
//	en=DWT_Get();
//	LOG("%u.%02u C\r\n", FLOAT_PRINTF(temp));
//	LOG("%d ms\r\n",((en-st)/72000));
#endif

	MAX6675_started = 1;

	return temp;
}

