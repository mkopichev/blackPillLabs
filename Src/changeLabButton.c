#include "../Inc/changeLabButton.h"

extern uint8_t clockSourceChoosen;
uint8_t changeLabButtonPressCounter = 1;
bool changeLabButtonInitDone = false, changeLabButtonPressed = false;

void changeLabButtonDeinit(void) {

	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);
	RCC->APB2ENR &= ~(RCC_APB2ENR_SYSCFGEN);
	SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0);
	EXTI->IMR &= ~(EXTI_IMR_MR0);
	EXTI->FTSR &= ~(EXTI_FTSR_TR0);
	EXTI->PR = EXTI_PR_PR0;
	NVIC_DisableIRQ(EXTI0_IRQn);
	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN);
	TIM2->PSC &= ~TIM_PSC_PSC;
	TIM2->ARR &= ~TIM_ARR_ARR;
	TIM2->CR1 &= ~(TIM_CR1_URS);
	TIM2->DIER &= ~(TIM_DIER_UIE);
	NVIC_DisableIRQ(TIM2_IRQn);

	changeLabButtonInitDone = false;
}

void changeLabButtonInit(void) {

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);
	//configure external interrupt
	GPIOA->MODER &= ~(3 << GPIO_MODER_MODE0_Pos); // pin PA0 - input
	GPIOA->PUPDR |= (1 << GPIO_PUPDR_PUPD0_Pos); // pull-up on PA0
	RCC->APB2ENR |= (1 << RCC_APB2ENR_SYSCFGEN_Pos); // clocking for SYSCFG
	SYSCFG->EXTICR[0] |= (0 << SYSCFG_EXTICR1_EXTI0_Pos); // choose PA
	EXTI->IMR |= (1 << EXTI_IMR_MR0_Pos); // choose 0 pin
	EXTI->FTSR |= (1 << EXTI_FTSR_TR0_Pos); // falling trigger
	EXTI->PR = EXTI_PR_PR0; // clear pending
	NVIC_EnableIRQ(EXTI0_IRQn);
	// configure timer 2 for debouncing
	RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM2EN_Pos);
	switch (clockSourceChoosen) {
	case HSI_SOURCE:
		TIM2->PSC = 16000 - 1; // 1 kHz freq
		break;
	case HSE_SOURCE:
		TIM2->PSC = 25000 - 1; // 1 kHz freq
		break;
	case PLL_SOURCE:
		TIM2->PSC = 42000 - 1; // 1 kHz freq
		break;
	}
	TIM2->ARR = 15; // 15 ms debouncing period
	TIM2->CR1 |= (1 << TIM_CR1_URS_Pos); // overflow generates IRQ
	TIM2->DIER |= (1 << TIM_DIER_UIE_Pos); // enable update interrupt
	NVIC_EnableIRQ(TIM2_IRQn); // enable timer 2 NVIC

	changeLabButtonInitDone = true;
}

void EXTI0_IRQHandler(void) {

	EXTI->IMR &= ~(1 << EXTI_IMR_MR0_Pos); // запретить прерывание на выводе 0
	EXTI->PR = (1 << EXTI_PR_PR0_Pos); // сбросить флаг Pending события
	TIM2->CR1 |= (1 << TIM_CR1_CEN_Pos); // start timer 2 counting
}

void TIM2_IRQHandler(void) {

	TIM2->CR1 &= ~(1 << TIM_CR1_CEN_Pos); // stop timer 2
	TIM2->SR &= ~(1 << TIM_SR_UIF_Pos); // clear timer 2 interrupt flag
	if (!(GPIOA->IDR & (1 << GPIO_IDR_ID0_Pos))) { // check voltage on PA0

		changeLabButtonPressed = true;
		if (++changeLabButtonPressCounter > NUM_OF_LABS) {

			changeLabButtonPressCounter = 1;
			NVIC_SystemReset();
		}
	}
	EXTI->IMR |= (1 << EXTI_IMR_MR0_Pos); // разрешить прерывание на выводе 0
}
