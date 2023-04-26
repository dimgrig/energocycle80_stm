#ifndef INC_DWT_H_
#define INC_DWT_H_

#include "stm32f1xx_hal.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void DWT_Init(void);
uint32_t DWT_Get(void);
__inline uint8_t DWT_Compare(int32_t tp);
void DWT_Delay(uint32_t us);
void DWT_Test();

#ifdef __cplusplus
}
#endif

#endif /* INC_DWT_H_ */
