#include <LinxCustomCmdHandler.h>

extern Energocycle_settings_s esettings;
extern Energocycle_status_s estatus;
extern EC_FLASH_settings eflash_settings;
extern Energocycle_DEBUG_s edebug;
extern Energocycle_DEBUG_OUT_s edebug_out;
//extern Energocycle_ADC_s eadc;
//extern EC_CHANNELS_OUT channel_settings;

#ifdef _UART_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
	extern UART_HandleTypeDef huart2;
#endif


const unsigned char CUSTOM_CMD_DATA_LENGTH[10] {
    0x01,
    0x01,
    0x28,
    0x24,
    0x18,
	0x04,
	0x04,
	0x04,
	0x01,
	0x10
};

const unsigned char CUSTOM_RSP_DATA_LENGTH[10] {
    0x00,
    0x19,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x21,
	0x21
};

LinxCustomCmdHandler::LinxCustomCmdHandler() {

}

LinxCustomCmdHandler::~LinxCustomCmdHandler() {

}

void LinxCustomCmdHandler::float_to_uca(float value, unsigned char *data) {
    unsigned char *ptr = (unsigned char *) &value;
    for (unsigned char i = 0; i < 4; ++i) {
        *data = *ptr;
        data++;
        ptr++;
    }
}

float LinxCustomCmdHandler::uca_to_float(const unsigned char *data) {
    float result = 0.0;
    unsigned char *ptr = (unsigned char *) &result;
    for (unsigned char i = 0; i < 4; ++i) {
        *ptr = *data;
        data++;
        ptr++;
    }
    return result;
}

void LinxCustomCmdHandler::float_value_unpack(unsigned char *data, float *value) {
	*value = uca_to_float(data + 0);
}



void LinxCustomCmdHandler::settings_unpack(unsigned char *data, Energocycle_settings_s *es) {
    es->mode = (Energocycle_mode) *(data + 0);
    es->DAC = ((*(data + 1)) << 8) | *(data + 2);
    es->res = *(data + 3);

    es->Unom = uca_to_float(data + 4);
    es->Umax = uca_to_float(data + 8);
    es->Umin = uca_to_float(data + 12);
    es->Inom = uca_to_float(data + 16);
    es->Imax = uca_to_float(data + 20);
    es->Imin = uca_to_float(data + 24);
    es->Tmax = uca_to_float(data + 28);
    es->Tmin = uca_to_float(data + 32);

    es->Cycle = ((*(data + 36)) << 8) | *(data + 37);
    es->Cycles = ((*(data + 38)) << 8) | *(data + 39);
}

void LinxCustomCmdHandler::pid_settings_unpack(unsigned char *data, EC_PID_settings *settings) {
	settings->U_PID_settings.Kp = uca_to_float(data + 0);
	settings->U_PID_settings.Ki = uca_to_float(data + 4);
	settings->U_PID_settings.Kd = uca_to_float(data + 8);

	settings->I_PID_settings.Kp = uca_to_float(data + 12);
	settings->I_PID_settings.Ki = uca_to_float(data + 16);
	settings->I_PID_settings.Kd = uca_to_float(data + 20);

	settings->T_PID_settings.Kp = uca_to_float(data + 24);
	settings->T_PID_settings.Ki = uca_to_float(data + 28);
	settings->T_PID_settings.Kd = uca_to_float(data + 32);
}

void LinxCustomCmdHandler::calib_settings_unpack(unsigned char *data, EC_CALIB_settings *settings) {
	settings->U_CALIB_settings.K = uca_to_float(data + 0);
	settings->U_CALIB_settings.B = uca_to_float(data + 4);

	settings->I_CALIB_settings.K = uca_to_float(data + 8);
	settings->I_CALIB_settings.B = uca_to_float(data + 12);

	settings->T_CALIB_settings.K = uca_to_float(data + 16);
	settings->T_CALIB_settings.B = uca_to_float(data + 20);
}

void LinxCustomCmdHandler::float_unpack(unsigned char *data, float *value) {
	*value = uca_to_float(data + 0);
}

void LinxCustomCmdHandler::debug_unpack(unsigned char *data, Energocycle_DEBUG_s *settings) {
	settings->PWM_ticks_enable = 	((*(data + 0)) >> 0) & 0x01;
	settings->GATE = 				((*(data + 0)) >> 1) & 0x01;
	settings->FAN = 				((*(data + 0)) >> 2) & 0x01;
	settings->FAN_DC = 				((*(data + 0)) >> 3) & 0x01;
	settings->LED_DC = 				((*(data + 0)) >> 4) & 0x01;
	settings->MAX6675 = 			((*(data + 0)) >> 5) & 0x01;
	settings->LM92 = 				((*(data + 0)) >> 6) & 0x01;

	settings->PWM_ticks_CH1 = ((*(data + 1)) << 8) | *(data + 2);
	settings->PWM_ticks_CH2 = ((*(data + 3)) << 8) | *(data + 4);
	settings->PWM_ticks_CH3 = ((*(data + 5)) << 8) | *(data + 6);
	settings->PWM_ticks_CH4 = ((*(data + 7)) << 8) | *(data + 8);
}

void LinxCustomCmdHandler::status_packetize(Energocycle_status_s *es, unsigned char *data) {
    //*(data + 0) = es->status;
    *(data + 0) = es->state;
    *(data + 1) = es->error;
    *(data + 2) = es->res;

    float_to_uca(es->U, (data + 3));
    float_to_uca(es->I, (data + 7));
    float_to_uca(es->T, (data + 11));
    float_to_uca(es->T_DC, (data + 15));

    *(data + 19) = (es->Cycles & 0xFF00) >> 8;
    *(data + 20) = es->Cycles & 0x00FF;

    *(data + 21) = (es->TCP_RESETS & 0xFF00) >> 8;
    *(data + 22) = es->TCP_RESETS & 0x00FF;

    *(data + 23) = (es->WWDG_RESETS & 0xFF00) >> 8;
    *(data + 24) = es->WWDG_RESETS & 0x00FF;
}

void LinxCustomCmdHandler::debug_packetize(Energocycle_DEBUG_OUT_s *settings, unsigned char *data) {
    //*(data + 0) = es->status;
    float_to_uca(settings->CH1, (data + 0));
    float_to_uca(settings->CH2, (data + 4));
    float_to_uca(settings->CH3, (data + 8));
    float_to_uca(settings->CH4, (data + 12));
    *(data + 16) = (settings->CCR1 & 0xFF00) >> 8;
    *(data + 17) = settings->CCR1 & 0x00FF;
    *(data + 18) = (settings->CCR2 & 0xFF00) >> 8;
    *(data + 19) = settings->CCR2 & 0x00FF;
    *(data + 20) = (settings->CCR3 & 0xFF00) >> 8;
    *(data + 21) = settings->CCR3 & 0x00FF;
    *(data + 22) = (settings->CCR4 & 0xFF00) >> 8;
    *(data + 23) = settings->CCR4 & 0x00FF;
	*(data + 24) = 0;
	*(data + 25) = 0;
	*(data + 26) = 0;
	*(data + 27) = 0;
	*(data + 28) = 0;
	*(data + 29) = 0;
	*(data + 30) = 0;
	*(data + 31) = 0;
    *(data + 32) = ((settings->overcurrent & 0x01) << 0);
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_CONTROL_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	//LINX_CUSTOM_CMD cmd = CUSTOM_CMD_CONTROL;
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
	LOG("CUSTOM_CMD_CONTROL");
#endif

	switch (*data_rx) {
	case 1:
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG(" - START\r\n");
#endif
		energocycle_start();

		break;
	case 2:
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG(" - RESET\r\n");
#endif
        if ((estatus.state == E_state_error)) // || (estatus.state == E_state_to_error) || (estatus.state == E_state_ostanov) || (estatus.state == E_state_to_ostanov))
        {
            estatus.state = E_state_reset;
        }
		break;
	default:
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG(" - OSTANOV\r\n");
#endif
        if ((estatus.state != E_state_done) &&
            (estatus.state != E_state_idle) &&
            (estatus.state != E_state_error)) {
            estatus.state = E_state_to_ostanov;
        }
	}
    return L_OK;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_STATUS_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_STATUS;

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
	//LOG("CUSTOM_CMD_STATUS\r\n");
#endif

    status_packetize(&estatus, data_tx);
    *data_tx_size = CUSTOM_RSP_DATA_LENGTH[cmd - CMD_OFFSET];

    return L_OK;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_SET_PARAMS_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_SET_PARAMS;

	LinxStatus status;
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		settings_unpack(data_rx, &esettings);
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_SET_PARAMS \r\n"
				"mode=%u; Cycles=%u; DAC=%u;\r\n"
				"Unom=%u.%02u; Umax=%u.%02u; Umin=%u.%02u; \r\n"
				"Inom=%u.%02u; Imax=%u.%02u; Imin=%u.%02u; \r\n"
				"Tmax=%u.%02u; Tmin=%u.%02u\r\n",
				esettings.mode,	esettings.Cycles, esettings.DAC,
				FLOAT_PRINTF(esettings.Unom),
				FLOAT_PRINTF(esettings.Umax),
				FLOAT_PRINTF(esettings.Umin),
				FLOAT_PRINTF(esettings.Inom),
				FLOAT_PRINTF(esettings.Imax),
				FLOAT_PRINTF(esettings.Imin),
				FLOAT_PRINTF(esettings.Tmax),
				FLOAT_PRINTF(esettings.Tmin));
#endif


		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_PID_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_PID;

	LinxStatus status;
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		pid_settings_unpack(data_rx, &eflash_settings.epid_settings);
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_PID \r\n"
				"T Kp=%u.%02u; T Ki=%u.%02u; T Kd=%u.%02u; \r\n"
				"U Kp=%u.%02u; U Ki=%u.%02u; U Kd=%u.%02u; \r\n"
				"I Kp=%u.%02u; I Ki=%u.%02u; I Kd=%u.%02u; \r\n",
				FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Kp),
				FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Ki),
				FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Kd),
				FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Kp),
				FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Ki),
				FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Kd),
				FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Kp),
				FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Ki),
				FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Kd));
#endif
		FLASH_WriteSettings();

		//pid_init();

		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_CALIBRATION_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_CALIBRATION;

	LinxStatus status;
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		calib_settings_unpack(data_rx, &eflash_settings.ecalib_settings);
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_CALIBRATION \r\n"
		  		"T K=%u.%02u; T B=%u.%02u \r\n"
		  		"U K=%u.%02u; U B=%u.%02u; \r\n"
		  		"I K=%u.%02u; I B=%u.%02u; \r\n",
		  		FLOAT_PRINTF(eflash_settings.ecalib_settings.T_CALIB_settings.K),
		  		FLOAT_PRINTF(eflash_settings.ecalib_settings.T_CALIB_settings.B),
		  		FLOAT_PRINTF(eflash_settings.ecalib_settings.U_CALIB_settings.K),
		  		FLOAT_PRINTF(eflash_settings.ecalib_settings.U_CALIB_settings.B),
		  		FLOAT_PRINTF(eflash_settings.ecalib_settings.I_CALIB_settings.K),
		  		FLOAT_PRINTF(eflash_settings.ecalib_settings.I_CALIB_settings.B));
#endif
		FLASH_WriteSettings();
		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_SET_U_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_SET_U;

	LinxStatus status;

	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		float value = 0;
		float_value_unpack(data_rx, &value);
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_SET_U U=%u\r\n", FLOAT_PRINTF(value));
#endif
		esettings.Unom = value;
		esettings.Inom = 0;
        if ((estatus.state != E_state_error)) // || (estatus.state == E_state_to_error) || (estatus.state == E_state_ostanov) || (estatus.state == E_state_to_ostanov))
        {
        	estatus.state = E_state_to_calibration;
        }

		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_SET_I_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_SET_I;

	LinxStatus status;

	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		float value = 0;
		float_value_unpack(data_rx, &value);
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_SET_I I=%u\r\n", FLOAT_PRINTF(value));
#endif
		esettings.Unom = 0;
		esettings.Inom = value;
        if ((estatus.state != E_state_error)) // || (estatus.state == E_state_to_error) || (estatus.state == E_state_ostanov) || (estatus.state == E_state_to_ostanov))
        {
        	estatus.state = E_state_to_calibration;
        }

		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_TEST_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_TEST;

	LinxStatus status;

	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		float value = 0;
		float_value_unpack(data_rx, &value);
#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_TEST I=%u\r\n", FLOAT_PRINTF(value));
#endif
		//esettings.Unom = 0;
		//esettings.Inom = value;
		if ((estatus.state != E_state_error) && (estatus.state != E_state_to_error)) {
			estatus.state = E_state_to_test;
		}

		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_DEBUG_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_DEBUG;

	LinxStatus status;

#ifdef PWM_TICKS_DEBUG
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_DEBUG \r\n");
#endif

		debug_packetize(&edebug_out, data_tx);
	    *data_tx_size = CUSTOM_RSP_DATA_LENGTH[cmd - CMD_OFFSET];

		status = L_OK;
	}
#endif

    return L_OK;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_SET_PWM_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_SET_PWM;

	LinxStatus status;

#ifdef PWM_TICKS_DEBUG
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		debug_unpack(data_rx, &edebug);

#if defined SEGGER_DEBUG || defined OCD_DEBUG || defined _UART_DEBUG
		LOG("CUSTOM_CMD_SET_PWM \r\n"
			"en=%u; GATE=%u; FAN=%u; FAN_DC=%u; LED_DC=%u; MAX6675=%u; LM92=%u; \r\n"
			"CH1=%u; CH2=%u; CH3=%u; CH4=%u; \r\n",
			edebug.PWM_ticks_enable, edebug.GATE, edebug.FAN, edebug.FAN_DC, edebug.LED_DC, edebug.MAX6675, edebug.LM92,
			edebug.PWM_ticks_CH1,
			edebug.PWM_ticks_CH2,
			edebug.PWM_ticks_CH3,
			edebug.PWM_ticks_CH4);
#endif

		edebug_out.overcurrent = 0;

		energocycle_debug();

	    debug_packetize(&edebug_out, data_tx);
	    *data_tx_size = CUSTOM_RSP_DATA_LENGTH[cmd - CMD_OFFSET];

		status = L_OK;
	}
#endif

    return L_OK;
}
