#ifndef ENERGOCYCLE80_ENERGOCYCLESETTINGS_H
#define ENERGOCYCLE80_ENERGOCYCLESETTINGS_H

#include "main.h"
#include "gpio.h"
#include "pwm.h"
#include "lm92.h"
#include "LinxDevice.h"
#include "dwt.h"
#include "Flash.h"

struct PID_settings_s {
    PID_settings_s() :
        Kp(0),
        Ki(0),
        Kd(0)
    {};
    PID_settings_s(float p, float i, float d) :
        Kp(p),
        Ki(i),
        Kd(d)
    {};
    PID_settings_s(PID_settings_s &settings) :
            Kp(settings.Kp),
            Ki(settings.Ki),
            Kd(settings.Kd)
    {};
    float Kp;
    float Ki;
    float Kd;
};

struct EC_PID_settings {
	EC_PID_settings() :
            T_PID_settings(),
            U_PID_settings(),
            I_PID_settings()
    {};
    EC_PID_settings(PID_settings_s t, PID_settings_s u, PID_settings_s i) :
            T_PID_settings(t),
            U_PID_settings(u),
            I_PID_settings(i)
    {};
    PID_settings_s T_PID_settings;
    PID_settings_s U_PID_settings;
    PID_settings_s I_PID_settings;
};

struct CALIB_settings_s {
	CALIB_settings_s() :
        K(1.0),
        B(0)
    {};
	CALIB_settings_s(float k, float b) :
        K(k),
        B(b)
    {};
    CALIB_settings_s(CALIB_settings_s &settings) :
            K(settings.K),
            B(settings.B)
    {};
    float K;
    float B;
};

struct EC_CALIB_settings {
	EC_CALIB_settings() :
		T_CALIB_settings(),
		U_CALIB_settings(),
		I_CALIB_settings()
    {};
	EC_CALIB_settings(CALIB_settings_s t, CALIB_settings_s u, CALIB_settings_s i) :
		T_CALIB_settings(t),
		U_CALIB_settings(u),
		I_CALIB_settings(i)
    {};

	CALIB_settings_s T_CALIB_settings;
	CALIB_settings_s U_CALIB_settings;
	CALIB_settings_s I_CALIB_settings;
};

enum Energocycle_mode {
    E_mode_U,
    E_mode_I,
    E_mode_P
};

enum Energocycle_state {
    E_state_idle,
    E_state_to_heat,
    E_state_heat,
    E_state_to_cool,
    E_state_cool,
    E_state_to_error,
    E_state_error,
    E_state_to_done,
    E_state_done,
    E_state_to_ostanov,
    E_state_ostanov,
	E_state_to_test,
	E_state_test,
	E_state_to_calibration,
	E_state_calibration,
    E_state_reset,
    E_state_to_idle,
	E_state_init
};

enum Energocycle_error {
    E_no_error,
    E_U_max,
    E_U_min,
    E_I_max,
    E_I_min,
	E_I_overcurrent,
	E_undervoltage,
	E_overheat,
	E_DC_T_lost,
	E_T_low,
	E_T_lost,
	E_WD,
	E_I_measure,
	E_T_measure,
	E_error_undefined
};

struct Energocycle_settings_s {
    Energocycle_settings_s() :
            mode(E_mode_U),
			DAC(MAX_TICKS),
            res(0),
            Unom(0),
            Umax(0),
            Umin(0),
            Inom(0),
            Imax(0),
            Imin(0),
            Tmax(0),
            Tmin(0),
            Cycle(0),
            Cycles(0)
    {};
    Energocycle_settings_s(Energocycle_mode m, float unom, float umax, float umin,
                           float inom, float imax, float imin,
                           float tmax, float tmin, unsigned int c, unsigned int cc) :
            mode(m),
			DAC(MAX_TICKS),
            res(0),
            Unom(unom),
            Umax(umax),
            Umin(umin),
            Inom(inom),
            Imax(imax),
            Imin(imin),
            Tmax(tmax),
            Tmin(tmin),
            Cycle(c),
    		Cycles(cc)
    {};
    Energocycle_mode mode;
    unsigned int DAC;
    unsigned int res;

    float Unom;
    float Umax;
    float Umin;
    float Inom;
    float Imax;
    float Imin;
    float Tmax;
    float Tmin;

    unsigned int Cycle;
    unsigned int Cycles;
};

struct Energocycle_status_s {
    Energocycle_status_s() :
            status(L_OK),
            state(E_state_init),
            error(E_no_error),
            res(0),
            U(0),
			U_in(0),
            I(0),
            T(0),
			T_DC(0),
			T_OFF(0),
            Cycles(0),
			energocycle_started(0),
			energocycle_stoped(0),
            TCP_RESETS(0),
            WWDG_RESETS(0)
    {};
    Energocycle_status_s(LinxStatus l_s, Energocycle_state s, Energocycle_error e,
                           float u, float u_in, float i, float t, float t_dc, float t_off,
						   unsigned int c, unsigned int tcp, unsigned int wwdg) :
            status(l_s),
            state(s),
            error(e),
            res(0),
            U(u),
			U_in(u_in),
            I(i),
            T(t),
			T_DC(t_dc),
			T_OFF(t_off),
            Cycles(c),
			energocycle_started(0),
			energocycle_stoped(0),
            TCP_RESETS(tcp),
            WWDG_RESETS(wwdg)
    {};

    LinxStatus status;
    Energocycle_state state;
    Energocycle_error error;
    unsigned char res;

    float U;
    float U_in;
    float I;
    float T;
    float T_DC;
    float T_OFF;
    unsigned int Cycles;
    unsigned int energocycle_started;
    unsigned int energocycle_stoped;
    unsigned int TCP_RESETS;
    unsigned int WWDG_RESETS;
};

struct Energocycle_ADC_s {
	Energocycle_ADC_s() :
        U1(0),
		U2(0),
		I1(0),
		I2(0),
		I3(0),
		I4(0),
		I_avg(0),
		started(0)
    {};
	Energocycle_ADC_s(float u1, float u2, float i1, float i2, float i3, float i4) :
            U1(u1),
			U2(u2),
			I1(i1),
			I2(i2),
			I3(i3),
			I4(i4),
			I_avg((i1 + i2 + i3 + i4)/4),
			started(0)
    {};


    float U1;
    float U2;
    float I1;
    float I2;
    float I3;
    float I4;
    float I_avg;
    uint8_t started;
};

struct Energocycle_DEBUG_s {
	Energocycle_DEBUG_s() :
		PWM_ticks_enable(0),
		GATE(0),
		PWM_ticks_CH1(MAX_TICKS),
		PWM_ticks_CH2(MAX_TICKS),
		PWM_ticks_CH3(MAX_TICKS),
		PWM_ticks_CH4(MAX_TICKS),
		FAN(0),
		FAN_DC(0),
		LED_DC(0),
		MAX6675(1),
		LM92(1)
    {};

	unsigned int PWM_ticks_enable;
	unsigned int GATE;
	unsigned int PWM_ticks_CH1;
	unsigned int PWM_ticks_CH2;
	unsigned int PWM_ticks_CH3;
	unsigned int PWM_ticks_CH4;
	unsigned int FAN;
	unsigned int FAN_DC;
	unsigned int LED_DC;
	unsigned int MAX6675;
    unsigned int LM92;
};

struct Energocycle_DEBUG_OUT_s {
    Energocycle_DEBUG_OUT_s() :
            CH1(0),
            CH2(0),
            CH3(0),
            CH4(0),
            CCR1(0),
            CCR2(0),
            CCR3(0),
            CCR4(0),
            overcurrent(0)
    {};

    float CH1;
    float CH2;
    float CH3;
    float CH4;
    unsigned int CCR1;
    unsigned int CCR2;
    unsigned int CCR3;
    unsigned int CCR4;

    unsigned int overcurrent;
};


struct EC_FLASH_settings{
	EC_FLASH_settings():
		init(0),
		epid_settings(),
		ecalib_settings(),
		wwdg_fires(0),
		ewwdg_settings()
	{}

	long unsigned int init;
	EC_PID_settings epid_settings;
	EC_CALIB_settings ecalib_settings;

	long unsigned int wwdg_fires;
	unsigned int CNT_WWDG_RESETS;
	Energocycle_state state;
	unsigned int energocycle_started;
	unsigned int energocycle_stoped;
    Energocycle_error error;
	Energocycle_settings_s ewwdg_settings;
	// !!! Full size (bytes) must be a multiple of 4 !!!
};

void energocycle_cycle();
void energocycle_debug();
void energocycle_start();
void energocycle_log();
void energocycle_abort();
void energocycle_temp_off_calc();
void energocycle_write_flash();

extern "C" void energocycle_write_flash_C();

#endif //ENERGOCYCLE80_ENERGOCYCLESETTINGS_H
