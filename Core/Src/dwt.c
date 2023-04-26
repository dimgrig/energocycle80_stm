#include "dwt.h"

#ifdef _UART_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
	extern UART_HandleTypeDef huart2;
#endif

extern uint32_t SystemCoreClock;

void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}

__inline
uint8_t DWT_Compare(int32_t tp)
{
  return (((int32_t)DWT_Get() - tp) < 0);
}

void DWT_Delay(uint32_t us) // microseconds
{
  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}

void DWT_Test(){
	uint32_t st,en;
	st=DWT_Get();
	HAL_Delay(27);
	en=DWT_Get();
	LOG("%d ms\r\n",((en-st)/72000));
}
