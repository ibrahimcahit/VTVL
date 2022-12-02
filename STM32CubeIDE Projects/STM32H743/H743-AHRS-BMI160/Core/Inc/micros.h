#ifndef INC_MICROS_H_
#define INC_MICROS_H_

#include "main.h"

__STATIC_INLINE void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void delay_us(uint32_t us)
{
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while(DWT->CYCCNT < us_count_tic);
}

__STATIC_INLINE uint64_t micros(void){
	return  DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

#endif /* INC_MICROS_H_ */
