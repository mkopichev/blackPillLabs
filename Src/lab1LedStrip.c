#include "../Inc/lab1LedStrip.h"

extern bool b12ButtonToggle;
extern bool changeLabButtonPressed;
bool lab1InitDone = false;

void lab1LedStripDeinit(void);
void lab1LedStripInit(void);
void lab1LedPosSwitch(uint8_t pos, bool state);

void lab1Deinit(void) {

	lab1LedStripDeinit();
	multiFuncButtonDeinit();

	lab1InitDone = false;
}

void lab1Init(void) {

	lab1LedStripInit();
	multiFuncButtonInit();

	lab1InitDone = true;
}

void lab1Execute(void) {

	for (int8_t i = b12ButtonToggle ? 0 : 6;
			b12ButtonToggle ? (i <= 6) : (i >= 0);
			b12ButtonToggle ? i++ : i--) {

		if (changeLabButtonPressed) {

			changeLabButtonPressed = false;
			return;
		}
		lab1LedPosSwitch(i, true);
		delayMs(250);
		lab1LedPosSwitch(i, false);
	}
}

void lab1LedStripDeinit(void) {

	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN);
	GPIOB->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE6 | GPIO_MODER_MODE5
			| GPIO_MODER_MODE4 | GPIO_MODER_MODE3 | GPIO_MODER_MODE2);
}

void lab1LedStripInit(void) {

	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos)
			| (1 << RCC_AHB1ENR_GPIOAEN_Pos);
	GPIOB->MODER &= ~(3 << GPIO_MODER_MODE0_Pos);
	GPIOB->MODER |= (1 << GPIO_MODER_MODE0_Pos);
	GPIOA->MODER &= ~((3 << GPIO_MODER_MODE7_Pos) | (3 << GPIO_MODER_MODE6_Pos)
			| (3 << GPIO_MODER_MODE5_Pos) | (3 << GPIO_MODER_MODE4_Pos)
			| (3 << GPIO_MODER_MODE3_Pos) | (3 << GPIO_MODER_MODE2_Pos));
	GPIOA->MODER |= (1 << GPIO_MODER_MODE7_Pos) | (1 << GPIO_MODER_MODE6_Pos)
			| (1 << GPIO_MODER_MODE5_Pos) | (1 << GPIO_MODER_MODE4_Pos)
			| (1 << GPIO_MODER_MODE3_Pos) | (1 << GPIO_MODER_MODE2_Pos);
}

void lab1LedPosSwitch(uint8_t pos, bool state) {

	switch (pos) {
	case 0: {
		GPIOB->BSRR =
				state ? (1 << GPIO_BSRR_BS0_Pos) : (1 << GPIO_BSRR_BR0_Pos);
		break;
	}
	case 1: {
		GPIOA->BSRR =
				state ? (1 << GPIO_BSRR_BS7_Pos) : (1 << GPIO_BSRR_BR7_Pos);
		break;
	}
	case 2: {
		GPIOA->BSRR =
				state ? (1 << GPIO_BSRR_BS6_Pos) : (1 << GPIO_BSRR_BR6_Pos);
		break;
	}
	case 3: {
		GPIOA->BSRR =
				state ? (1 << GPIO_BSRR_BS5_Pos) : (1 << GPIO_BSRR_BR5_Pos);
		break;
	}
	case 4: {
		GPIOA->BSRR =
				state ? (1 << GPIO_BSRR_BS4_Pos) : (1 << GPIO_BSRR_BR4_Pos);
		break;
	}
	case 5: {
		GPIOA->BSRR =
				state ? (1 << GPIO_BSRR_BS3_Pos) : (1 << GPIO_BSRR_BR3_Pos);
		break;
	}
	case 6: {
		GPIOA->BSRR =
				state ? (1 << GPIO_BSRR_BS2_Pos) : (1 << GPIO_BSRR_BR2_Pos);
		break;
	}
	default:
		break;
	}
}
