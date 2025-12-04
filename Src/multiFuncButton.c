#include "../Inc/multiFuncButton.h"

extern uint8_t clockSourceChoosen;
bool b12ButtonToggle = false;
uint8_t b12ButtonPressCounter = 1;
bool multiFuncButtonInitDone = false;

void multiFuncButtonDeinit(void) {

	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN);
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD12);
	RCC->APB2ENR &= ~(RCC_APB2ENR_SYSCFGEN);
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);
	EXTI->IMR &= ~(EXTI_IMR_MR12);
	EXTI->FTSR &= ~(EXTI_FTSR_TR12);
	EXTI->PR = EXTI_PR_PR12;
	NVIC_DisableIRQ(EXTI15_10_IRQn);
	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM4EN);
	TIM4->PSC &= ~TIM_PSC_PSC;
	TIM4->ARR &= ~TIM_ARR_ARR;
	TIM4->CR1 &= ~(TIM_CR1_URS);
	TIM4->DIER &= ~(TIM_DIER_UIE);
	NVIC_DisableIRQ(TIM4_IRQn);

	multiFuncButtonInitDone = false;
}

void multiFuncButtonInit(void) {

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos);
	//configure button PB12
	GPIOB->MODER &= ~(3 << GPIO_MODER_MODE12_Pos); // pin PB12 - input
	GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD12_Pos); // pull-up on PB12
	RCC->APB2ENR |= (1 << RCC_APB2ENR_SYSCFGEN_Pos); // clocking for SYSCFG
	SYSCFG->EXTICR[3] = (1 << SYSCFG_EXTICR4_EXTI12_Pos); // choose PB
	EXTI->IMR |= (1 << EXTI_IMR_MR12_Pos); // choose 12-th
	EXTI->FTSR |= (1 << EXTI_FTSR_TR12_Pos); // falling trigger
	EXTI->PR = EXTI_PR_PR12; // clear pending
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	// configure timer 4 for debouncing
	RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM4EN_Pos);
	switch (clockSourceChoosen) {
	case HSI_SOURCE:
		TIM4->PSC = 16000 - 1; // 1 kHz freq
		break;
	case HSE_SOURCE:
		TIM4->PSC = 25000 - 1; // 1 kHz freq
		break;
	case PLL_SOURCE:
		TIM4->PSC = 42000 - 1; // 1 kHz freq
		break;
	}
	TIM4->ARR = 15; // 15 ms debouncing period
	TIM4->CR1 |= (1 << TIM_CR1_URS_Pos); // overflow generates IRQ
	TIM4->DIER |= (1 << TIM_DIER_UIE_Pos); // enable update interrupt
	NVIC_EnableIRQ(TIM4_IRQn); // enable timer 4 NVIC

	multiFuncButtonInitDone = true;
}

void EXTI15_10_IRQHandler(void) {

	EXTI->IMR &= ~(1 << EXTI_IMR_MR12_Pos); // запретить прерывание на выводе 12
	EXTI->PR = (1 << EXTI_PR_PR12_Pos); // сбросить флаг Pending события
	TIM4->CR1 |= (1 << TIM_CR1_CEN_Pos); // start timer 4 counting
}

void TIM4_IRQHandler(void) {

	TIM4->CR1 &= ~(1 << TIM_CR1_CEN_Pos); // stop timer 4
	TIM4->SR &= ~(1 << TIM_SR_UIF_Pos); // clear timer 4 interrupt flag
	if (!(GPIOB->IDR & (1 << GPIO_IDR_ID12_Pos))) { // check voltage on PB12

		b12ButtonToggle = !b12ButtonToggle;

		if (++b12ButtonPressCounter > 7) {

			b12ButtonPressCounter = 1;
		}
	}
	EXTI->IMR |= (1 << EXTI_IMR_MR12_Pos); // разрешить прерывание на выводе 12
}
