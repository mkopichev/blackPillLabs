#include "../Inc/lab3UartAdcDma.h"

extern uint8_t clockSourceChoosen;
extern bool changeLabButtonPressed;
bool lab3InitDone = false, adcResultReady = false;
uint16_t adcResult[2] = { 0, 0 }, adcFilteredResult[2] = { 0, 0 };

void lab3UartDeinit(void);
void lab3AdcDeinit(void);
void lab3DmaDeinit(void);
void lab3UartInit(void);
void lab3AdcInit(void);
void lab3DmaInit(void);
void lab3AdcReadSignals(void);
void lab3AdcGetFilteredData(void);

void lab3Deinit(void) {

	lab3UartDeinit();
	lab3AdcDeinit();
	lab3DmaDeinit();

	adcResultReady = false;
	lab3InitDone = false;
}

void lab3Init(void) {

	lab3UartInit();
	lab3AdcInit();
	lab3DmaInit();

	lab3InitDone = true;
}

void lab3Execute(void) {

	if (changeLabButtonPressed) {

		changeLabButtonPressed = false;
		return;
	}
	lab3AdcGetFilteredData();
	uartTransmitStr("Tjunc: ");
	uartTransmitDec(
			(int16_t) ((((adcFilteredResult[TEMP_SENS] - 705) * 10) / 25) + 25));
	uartTransmitStr("\tT25: ");
	uartTransmitDec(adcFilteredResult[TEMP_SENS]);
	uartTransmitStr("\tIN1: ");
	uartTransmitDec(adcFilteredResult[POT]);
	uartTransmitStr("\r\n");
}

void lab3AdcReadSignals(void) {

	adcResultReady = false;

	ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos);

	while (!adcResultReady) {

		__NOP();
	}
}

void lab3AdcGetFilteredData(void) {

	uint32_t tmp[] = { 0, 0 };
	for (uint8_t i = 0; i < NUM_OF_SAMPLES; i++) {

		lab3AdcReadSignals();
		tmp[TEMP_SENS] += adcResult[TEMP_SENS];
		tmp[POT] += adcResult[POT];
	}
	adcFilteredResult[TEMP_SENS] = tmp[TEMP_SENS] / NUM_OF_SAMPLES;
	adcFilteredResult[POT] = tmp[POT] / NUM_OF_SAMPLES;
}

void lab3UartDeinit(void) {

	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
	RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN);
	USART2->CR1 &= ~(USART_CR1_UE | USART_CR1_TE);
	USART2->BRR &= ~(USART_BRR_DIV_Fraction | USART_BRR_DIV_Mantissa);
}

void lab3AdcDeinit(void) {

	RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE1);
	ADC1->CR2 &= ~(ADC_CR2_ADON);
	ADC1->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_SWSTART);
	ADC->CCR &= ~(ADC_CCR_TSVREFE);
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1 | ADC_SQR3_SQ2);
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP16);
	ADC1->SMPR2 &= ~(ADC_SMPR2_SMP1);
	ADC1->CR1 &= ~(ADC_CR1_SCAN);
	ADC1->SR &= ~(ADC_SR_EOC);
}

void lab3DmaDeinit(void) {

	RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA2EN);
	DMA2_Stream0->CR &= ~(DMA_SxCR_EN);
	DMA2_Stream0->CR &= ~(DMA_SxCR_CHSEL | DMA_SxCR_PL | DMA_SxCR_MINC
			| DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_TCIE | DMA_SxCR_CIRC);
	DMA2_Stream0->PAR = 0;
	DMA2_Stream0->M0AR = 0;
	DMA2_Stream0->NDTR = 0;
	NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	DMA2->LIFCR = DMA_LIFCR_CTCIF0;
}

void lab3UartInit(void) {

	// configure UART
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);
	GPIOA->MODER &=
			~((3 << GPIO_MODER_MODE2_Pos) | (3 << GPIO_MODER_MODE3_Pos));
	GPIOA->MODER |= (2 << GPIO_MODER_MODE2_Pos) | (2 << GPIO_MODER_MODE3_Pos);
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
	RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);
	USART2->CR1 |= (1 << USART_CR1_UE_Pos) | (1 << USART_CR1_TE_Pos);
	switch (clockSourceChoosen) {
	case HSI_SOURCE:
		USART2->BRR = (52UL << USART_BRR_DIV_Mantissa_Pos)
				| (1UL << USART_BRR_DIV_Fraction_Pos); // 52.08
		break;
	case HSE_SOURCE:
		USART2->BRR = (81UL << USART_BRR_DIV_Mantissa_Pos)
				| (6UL << USART_BRR_DIV_Fraction_Pos); // 81.38
		break;
	case PLL_SOURCE:
		USART2->BRR = (136UL << USART_BRR_DIV_Mantissa_Pos)
				| (12UL << USART_BRR_DIV_Fraction_Pos); // 136.72
		break;
	}
}

void lab3AdcInit(void) {

	// configure ADC1 + DMA
	RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos); // adc clocking on
	GPIOA->MODER &= ~(3 << GPIO_MODER_MODE1_Pos);
	GPIOA->MODER |= (3 << GPIO_MODER_MODE1_Pos); // PA1 is in analog mode
	ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos) | (1 << ADC_CR2_DMA_Pos)
			| (1 << ADC_CR2_DDS_Pos); // turn on ADC1, configure DMA
	ADC->CCR |= (1 << ADC_CCR_TSVREFE_Pos); // turn on temp sensor
	ADC1->SQR3 &= ~((31 << ADC_SQR3_SQ1_Pos) | (31 << ADC_SQR3_SQ2_Pos)); // clear SQ1 and SQ2
	ADC1->SQR3 |= (16 << ADC_SQR3_SQ1_Pos) | (1 << ADC_SQR3_SQ2_Pos); // put ADC_IN16 to SQ1 and ADC_IN1 to SQ2
	ADC1->SQR1 |= (1 << ADC_SQR1_L_Pos);
	ADC1->SMPR1 |= (7 << ADC_SMPR1_SMP16_Pos);
	ADC1->SMPR2 |= (7 << ADC_SMPR2_SMP1_Pos);
	ADC1->CR1 |= (1 << ADC_CR1_SCAN_Pos);
}

void lab3DmaInit(void) {

	// configure DMA
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA2EN_Pos);
	DMA2_Stream0->CR &= ~(1 << DMA_SxCR_EN_Pos);
	DMA2_Stream0->CR |= (0 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos)
			| (1 << DMA_SxCR_MINC_Pos) | (1 << DMA_SxCR_MSIZE_Pos)
			| (1 << DMA_SxCR_PSIZE_Pos) | (1 << DMA_SxCR_TCIE_Pos)
			| (1 << DMA_SxCR_CIRC_Pos);
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t) adcResult;
	DMA2_Stream0->NDTR = 2;
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	DMA2_Stream0->CR |= (1 << DMA_SxCR_EN_Pos);
}

void DMA2_Stream0_IRQHandler(void) {

	// Check if the Transfer Complete interrupt flag is set
	if (DMA2->LISR & DMA_LISR_TCIF0) {
		DMA2->LIFCR = DMA_LIFCR_CTCIF0; // Clear the interrupt flag
		adcResultReady = true; // Set a flag for the main loop
		DMA2_Stream0->CR |= (1 << DMA_SxCR_EN_Pos);
	}
}
