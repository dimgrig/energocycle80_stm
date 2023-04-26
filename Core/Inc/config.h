#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

//version 20220420 tested

#define CHANNEL 2
#define RPI 0

#if (CHANNEL == 1)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA0}
	#if (RPI == 0)
	#define IP_ADDR {192,168,0,100}
	#else
	#define IP_ADDR {169,254,103,100}
	#endif
#elif (CHANNEL == 2)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA1}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,101}
	#else
	#define IP_ADDR {169,254,103,101}
	#endif
#elif (CHANNEL == 3)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA2}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,102}
	#else
	#define IP_ADDR {169,254,103,102}
	#endif
#elif (CHANNEL == 4)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA3}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,103}
	#else
	#define IP_ADDR {169,254,103,103}
	#endif
#elif (CHANNEL == 5)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA4}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,104}
	#else
	#define IP_ADDR {169,254,103,104}
	#endif
#elif (CHANNEL == 6)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA5}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,105}
	#else
	#define IP_ADDR {169,254,103,105}
	#endif
#elif (CHANNEL == 7)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA6}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,106}
	#else
	#define IP_ADDR {169,254,103,106}
	#endif
#elif (CHANNEL == 8)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA7}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,107}
	#else
	#define IP_ADDR {169,254,103,107}
	#endif
#elif (CHANNEL == 9)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA8}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,208}
	#else
	#define IP_ADDR {169,254,103,108}
	#endif
#elif (CHANNEL == 10)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xA9}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,109}
	#else
	#define IP_ADDR {169,254,103,109}
	#endif
#elif (CHANNEL == 11)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB0}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,110}
	#else
	#define IP_ADDR {169,254,103,110}
	#endif
#elif (CHANNEL == 12)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB1}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,111}
	#else
	#define IP_ADDR {169,254,103,111}
	#endif
#elif (CHANNEL == 13)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB2}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,112}
	#else
	#define IP_ADDR {169,254,103,112}
	#endif
#elif (CHANNEL == 14)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB3}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,113}
	#else
	#define IP_ADDR {169,254,103,113}
	#endif
#elif (CHANNEL == 15)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB4}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,114}
	#else
	#define IP_ADDR {169,254,103,114}
	#endif
#elif (CHANNEL == 16)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB5}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,115}
	#else
	#define IP_ADDR {169,254,103,115}
	#endif
#elif (CHANNEL == 17)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB6}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,116}
	#else
	#define IP_ADDR {169,254,103,116}
	#endif
#elif (CHANNEL == 18)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB7}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,117}
	#else
	#define IP_ADDR {169,254,103,117}
	#endif
#elif (CHANNEL == 19)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB8}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,118}
	#else
	#define IP_ADDR {169,254,103,118}
	#endif
#elif (CHANNEL == 20)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xB9}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,119}
	#else
	#define IP_ADDR {169,254,103,119}
	#endif
#elif (CHANNEL == 21)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xC0}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,120}
	#else
	#define IP_ADDR {169,254,103,120}
	#endif
#elif (CHANNEL == 22)
	#define MAC_ADDR {0x00, 0xAD, 0xBE, 0xEF, 0xFE, 0xC1}
	#if (RPI == 0)
	#define IP_ADDR {192,168,1,121}
	#else
	#define IP_ADDR {169,254,103,121}
	#endif
#endif

#if (RPI == 0)
#define IP_GATE {192,168,1,1} //1.63
#else
#define IP_GATE {169,254,103,1}
#endif


#define IP_MASK {255,255,255,0}
#define LOCAL_PORT_TCP 80
#define TCP_MESSAGE_BUFF_SIZE 128

#define I_CHANNELS 4
#define MAX_DUTY 100
#define PRESCALER 0 //16000
#define PERIOD 288 - 1 //2880 - 1
#define MAX_TICKS 288
#define I_BALANCE_MIN 1.0 //A

#define MAX6675_T_min 10
#define MAX6675_T_max 200

#define LM92_ADDR 0b10010000
#define LM92_T_max 75
#define LM92_T_CRIT 100.0
#define LM92_TLOW 5.0
#define LM92_THIGH 80.0

#define LM92_TEMP_FAN 40 //temp to on DC_FAN
#define I_FAN 40 //current to on DC_FAN
#define LM92_TEMP_FAN_CALIB 60 //temp to on DC_FAN
#define I_FAN_CALIB 100 //current to on DC_FAN

// 0.048 or 0.055 V/A
#define GAIN_VA 0.048
#define U_ref 3.327 //3.347 // 3.27 //3.61
#define U_IN_SCALE_FACTOR 4.19  // scale of DC input
#define U_SCALE_FACTOR 2.0  // scale of DC input

#define U_in_min 10.0
#define ADC_START_MEASUREMENTS 100 //20
#define I_MEASURE_DELTA 20.0 //percents from Uref/2

#define T_MEASURE_RATE 0.0125 //seconds
#define T_MEASURE_COMPARE_RATE 30 //seconds
#define T_MEASURE_DELTA 0.25 //degree

#define ENERGOCYCLE_START 3 //seconds

#define WWDG_PROTECTION 1
#define LM92_PROTECTION 1
#define MAX6675_PROTECTION 1

#define TEMP_INERTIA_COMPENSATION 1

//#define TEST_IRQ 1


//#define SEGGER_DEBUG 1
//#define OCD_DEBUG 1
//#define SYSVIEW_DEBUG 1
//#define uIP_DEBUG 1
//#define _UART_DEBUG 1
#define PWM_TICKS_DEBUG 1

#define FLOAT_PRINTF(x) (int)(x), (int)(((x) - (int)(x))*100)
#define FLOAT_SIGN(x) (x > 0) ? "+" : "-"
#define FLOAT_PRINTF_INT(x) (x > 0) ? (int)(x) : (int)(-x)
#define FLOAT_PRINTF_FRAC(x) (x > 0) ? (int)(((x) - (int)(x))*100) : (int)(((-x) - (int)(-x))*100)
#define FLOAT_PRINTF_SIGNED(x) FLOAT_SIGN(x), FLOAT_PRINTF_INT(x), FLOAT_PRINTF_FRAC(x)

#ifdef OCD_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
#endif

#ifdef OCD_DEBUG
	#define LOG(...) printf(__VA_ARGS__)
#else
	#ifdef SEGGER_DEBUG
		#define LOG(...) SEGGER_RTT_printf(0, __VA_ARGS__);
	#else
		#ifdef _UART_DEBUG
			#define LOG(...) printf(__VA_ARGS__)
		#else
			#define LOG(...)
		#endif
	#endif
#endif

#endif /* INC_CONFIG_H_ */
