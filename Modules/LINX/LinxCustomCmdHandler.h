#ifndef LINX_LINXCUSTOMCMDHANDLER_H_
#define LINX_LINXCUSTOMCMDHANDLER_H_

#include <Energocycle.h>
#include "SEGGER_RTT.h"

#include "config.h"
#include "Flash.h"
#include "LinxDevice.h"
#include "gpio.h"
#include "pid.h"
#include "pwm.h"

enum LINX_CUSTOM_CMD {
    CUSTOM_CMD_CONTROL = 0xFE01,
    CUSTOM_CMD_STATUS = 0xFE02,
    CUSTOM_CMD_SET_PARAMS = 0xFE03,
    CUSTOM_CMD_PID = 0xFE04,
    CUSTOM_CMD_CALIBRATION = 0xFE05,
    CUSTOM_CMD_SET_U = 0xFE06,
	CUSTOM_CMD_SET_I = 0xFE07,
    CUSTOM_CMD_TEST = 0xFE08,
    CUSTOM_CMD_DEBUG = 0xFE09,
	CUSTOM_CMD_SET_PWM = 0xFE0A
};

#define CMD_OFFSET CUSTOM_CMD_CONTROL
#define CALL_MEMBER_FN(object, ptr) (object->*ptr)

class LinxCustomCmdHandler {
public:
	LinxCustomCmdHandler();
	virtual ~LinxCustomCmdHandler();

	void float_to_uca(float value, unsigned char *data);
	float uca_to_float(const unsigned char *data);

	void float_value_unpack(unsigned char *data, float *value);
	void settings_unpack(unsigned char *data, Energocycle_settings_s *es);
	void pid_settings_unpack(unsigned char *data, EC_PID_settings *settings);
	void calib_settings_unpack(unsigned char *data, EC_CALIB_settings *settings);
	void float_unpack(unsigned char *data, float *value);
	void debug_unpack(unsigned char *data, Energocycle_DEBUG_s *settings);

	void status_packetize(Energocycle_status_s *es, unsigned char *data);
	void debug_packetize(Energocycle_DEBUG_OUT_s *settings, unsigned char *data);

	LinxStatus CUSTOM_CMD_CONTROL_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_STATUS_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_SET_PARAMS_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_PID_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_CALIBRATION_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_SET_U_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_SET_I_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_TEST_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_DEBUG_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_SET_PWM_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
};

#endif /* LINX_LINXCUSTOMCMDHANDLER_H_ */
