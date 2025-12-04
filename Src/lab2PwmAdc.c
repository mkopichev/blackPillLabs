#include "../Inc/lab2PwmAdc.h"

extern uint8_t clockSourceChoosen;
extern uint8_t b12ButtonPressCounter;
extern bool changeLabButtonPressed;
bool lab2InitDone = false;

void lab2GpioDeinit(void);
void lab2Tim3Deinit(void);
void lab2AdcPotDeinit(void);
void lab2GpioInit(void);
void lab2Tim3Init(void);
void lab2AdcPotInit(void);
uint16_t lab2PotReadRawValue(void);
uint16_t lab2PotGetAverageValue(void);

void lab2Deinit(void) {

	lab2GpioDeinit();
	multiFuncButtonDeinit();
	lab2Tim3Deinit();
	lab2AdcPotDeinit();

	lab2InitDone = false;
}

void lab2Init(void) {

	lab2GpioInit();
	multiFuncButtonInit();
	lab2Tim3Init();
	lab2AdcPotInit();

	b12ButtonPressCounter = 1;
	lab2InitDone = true;
}

void lab2Execute(void) {

	if (changeLabButtonPressed) {

		changeLabButtonPressed = false;
		return;
	}
	TIM3->CCR4 = (lab2PotGetAverageValue() >> 4);
	delayMs(1);
}

void lab2GpioDeinit(void) {

	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN);
	GPIOB->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE6 | GPIO_MODER_MODE5
			| GPIO_MODER_MODE4 | GPIO_MODER_MODE3 | GPIO_MODER_MODE2);
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL0);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL7 | GPIO_AFRL_AFSEL6);
}

void lab2Tim3Deinit(void) {

	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN);
	TIM3->PSC &= ~TIM_PSC_PSC;
	TIM3->ARR &= ~TIM_ARR_ARR;
	TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
	TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M);
	TIM3->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC4IE);
	NVIC_DisableIRQ(TIM3_IRQn);
	TIM3->CR1 &= ~(TIM_CR1_CEN);
}

void lab2AdcPotDeinit(void) {

	RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE1);
	ADC1->CR2 &= ~(ADC_CR2_ADON);
	while (ADC1->CR2 & ADC_CR2_ADON) {
		__NOP();
	}
	ADC1->CR2 &= ~(ADC_CR2_SWSTART);
	ADC1->SR &= ~(ADC_SR_EOC);
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
}

void lab2GpioInit(void) {

	//configure GPIO
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos)
			| (1 << RCC_AHB1ENR_GPIOAEN_Pos);
	GPIOB->MODER &= ~(3 << GPIO_MODER_MODE0_Pos);
	GPIOB->MODER |= (2 << GPIO_MODER_MODE0_Pos);
	GPIOA->MODER &= ~((3 << GPIO_MODER_MODE7_Pos) | (3 << GPIO_MODER_MODE6_Pos)
			| (3 << GPIO_MODER_MODE5_Pos) | (3 << GPIO_MODER_MODE4_Pos)
			| (3 << GPIO_MODER_MODE3_Pos) | (3 << GPIO_MODER_MODE2_Pos));
	GPIOA->MODER |= (2 << GPIO_MODER_MODE7_Pos) | (2 << GPIO_MODER_MODE6_Pos)
			| (1 << GPIO_MODER_MODE5_Pos) | (1 << GPIO_MODER_MODE4_Pos)
			| (1 << GPIO_MODER_MODE3_Pos) | (1 << GPIO_MODER_MODE2_Pos);
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL0_Pos);
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL7_Pos) | (2 << GPIO_AFRL_AFSEL6_Pos);
}

void lab2Tim3Init(void) {

	// configure timer 3 for PWM
	RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM3EN_Pos);
	switch (clockSourceChoosen) {
	case HSI_SOURCE:
		TIM3->PSC = 1600 - 1; // 10 kHz freq
		break;
	case HSE_SOURCE:
		TIM3->PSC = 2500 - 1; // 10 kHz freq
		break;
	case PLL_SOURCE:
		TIM3->PSC = 4200 - 1; // 10 kHz freq
		break;
	}
	TIM3->ARR = 250; // period ms * 0.1
	TIM3->CCER |= (1 << TIM_CCER_CC1E_Pos) | (1 << TIM_CCER_CC2E_Pos)
			| (1 << TIM_CCER_CC3E_Pos);
	TIM3->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos)
			| (0b110 << TIM_CCMR1_OC2M_Pos);
	TIM3->CCMR2 |= (0b110 << TIM_CCMR2_OC3M_Pos);
	TIM3->DIER |= (1 << TIM_DIER_UIE_Pos) | (1 << TIM_DIER_CC4IE_Pos);
	NVIC_EnableIRQ(TIM3_IRQn);
	TIM3->CR1 |= (1 << TIM_CR1_CEN_Pos);
}

void lab2AdcPotInit(void) {

	// configure potentiometer ADC1_IN1
	RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos); // adc clocking on
	GPIOA->MODER &= ~(3 << GPIO_MODER_MODE1_Pos);
	GPIOA->MODER |= (3 << GPIO_MODER_MODE1_Pos); // PA1 is in analog mode
	ADC1->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_SWSTART);
	ADC1->CR1 &= ~(ADC_CR1_SCAN); // clear scan
	DMA2->LIFCR = DMA_LIFCR_CTCIF0;
	ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos); // turn on ADC1
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP16);
	ADC1->SQR1 &= ~(ADC_SQR1_L); // clear L
	ADC1->SQR3 &= ~(ADC_SQR3_SQ2 | ADC_SQR3_SQ1); // clear SQ2 and SQ1
	ADC1->SQR3 |= (1 << ADC_SQR3_SQ1_Pos); // put ADC_IN1 to SQ1
}

uint16_t lab2PotReadRawValue(void) {

	ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos);
	while (!(ADC1->SR & (1 << ADC_SR_EOC_Pos))) {

		__NOP();
	}
	ADC1->SR &= ~(1 << ADC_SR_EOC_Pos);
	return ADC1->DR;
}

uint16_t lab2PotGetAverageValue(void) {

	uint32_t tmpVal = 0;
	for (uint8_t i = 0; i < NUM_OF_SAMPLES; i++) {

		tmpVal += lab2PotReadRawValue();
	}
	return tmpVal / NUM_OF_SAMPLES;
}

void TIM3_IRQHandler(void) {

	if (b12ButtonPressCounter > 3) {

		TIM3->CCR3 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR1 = 0;
	}

	switch (b12ButtonPressCounter) {
	case 1:
		TIM3->CCR3 = (lab2PotGetAverageValue() >> 4);
		TIM3->CCR2 = 0;
		TIM3->CCR1 = 0;
		break;
	case 2:
		TIM3->CCR3 = 0;
		TIM3->CCR2 = (lab2PotGetAverageValue() >> 4);
		TIM3->CCR1 = 0;
		break;
	case 3:
		TIM3->CCR3 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR1 = (lab2PotReadRawValue() >> 4);
		break;
	case 4:
		if (TIM3->SR & (1 << TIM_SR_UIF_Pos)) {

			TIM3->SR &= ~(1 << TIM_SR_UIF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BS5_Pos);
		} else {

			TIM3->SR &= ~(1 << TIM_SR_CC4IF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BR5_Pos);
		}
		break;
	case 5:
		if (TIM3->SR & (1 << TIM_SR_UIF_Pos)) {

			TIM3->SR &= ~(1 << TIM_SR_UIF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BS4_Pos);
		} else {

			TIM3->SR &= ~(1 << TIM_SR_CC4IF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BR4_Pos);
		}
		break;
	case 6:
		if (TIM3->SR & (1 << TIM_SR_UIF_Pos)) {

			TIM3->SR &= ~(1 << TIM_SR_UIF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BS3_Pos);
		} else {

			TIM3->SR &= ~(1 << TIM_SR_CC4IF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BR3_Pos);
		}
		break;
	case 7:
		if (TIM3->SR & (1 << TIM_SR_UIF_Pos)) {

			TIM3->SR &= ~(1 << TIM_SR_UIF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BS2_Pos);
		} else {

			TIM3->SR &= ~(1 << TIM_SR_CC4IF_Pos);
			GPIOA->BSRR = (1 << GPIO_BSRR_BR2_Pos);
		}
		break;
	}
}
