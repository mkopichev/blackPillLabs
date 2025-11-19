#include "../Inc/common.h"

uint8_t clockSourceChoosen;

void cpuFreqInit(uint8_t clockSource) {

	// setup clock
	switch (clockSource) {

	case HSI_SOURCE:

		RCC->CR = (1 << RCC_CR_HSION_Pos);
		while (!(RCC->CR & (1 << RCC_CR_HSIRDY_Pos))) {

			__NOP();
		}
		RCC->CFGR = RCC_CFGR_SW_HSI;
		clockSourceChoosen = HSI_SOURCE;
		break;

	case HSE_SOURCE:

		RCC->CR = (1 << RCC_CR_HSEON_Pos);
		while (!(RCC->CR & (1 << RCC_CR_HSERDY_Pos))) {

			__NOP();
		}
		RCC->CFGR = RCC_CFGR_SW_HSE;
		clockSourceChoosen = HSE_SOURCE;
		break;

	case PLL_SOURCE:

		RCC->CR = (1 << RCC_CR_HSEON_Pos);
		while (!(RCC->CR & (1 << RCC_CR_HSERDY_Pos))) {

			__NOP();
		}
		RCC->CR &= ~(1 << RCC_CR_PLLON_Pos);
		RCC->PLLCFGR = (1 << RCC_PLLCFGR_PLLSRC_Pos)
				| (1 << RCC_PLLCFGR_PLLP_Pos) | (336UL << RCC_PLLCFGR_PLLN_Pos)
				| (25UL << RCC_PLLCFGR_PLLM_Pos);
		RCC->CR |= (1 << RCC_CR_PLLON_Pos);
		while (!(RCC->CR & (1 << RCC_CR_PLLRDY_Pos))) {

			__NOP();
		}
		do {

			FLASH->ACR |= 2;
		} while ((FLASH->ACR & FLASH_ACR_LATENCY) != 2);
		RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_SW_PLL;
		clockSourceChoosen = PLL_SOURCE;
		break;

	}
}

void sysTickInit(void) {

	// configure SysTick
	SysTick->CTRL |= (1 << SysTick_CTRL_CLKSOURCE_Pos) // AHB
	| (1 << SysTick_CTRL_ENABLE_Pos);
	switch (clockSourceChoosen) {
	case HSI_SOURCE:
		SysTick->LOAD = 16000 - 1;
		break;
	case HSE_SOURCE:
		SysTick->LOAD = 25000 - 1;
		break;
	case PLL_SOURCE:
		SysTick->LOAD = 84000 - 1;
		break;
	}
}

void delayMs(uint32_t msTime) {

	SysTick->VAL = 0;
	for (uint32_t i = 0; i < msTime; i++) {

		while (!(SysTick->CTRL & (1 << SysTick_CTRL_COUNTFLAG_Pos))) {

			__NOP();
		}
	}
}

uint8_t digitExtractor(uint16_t value, uint8_t digit) {

	uint8_t tmp = 5, res;
	while (tmp) {

		res = value % 10;
		if (tmp == digit) {

			break;
		}
		tmp--;
		value /= 10;
	}
	return res;
}
