#pragma once

#include "stm32f4xx.h"

class Pin
{
public:
	Pin(GPIO_TypeDef* Port, uint32_t Pin)
		: port(Port), pin(Pin)
	{
	}
	GPIO_TypeDef* port;
	uint32_t pin;
};

#define PA0 Pin(GPIOA, GPIO_PIN_0)
#define PA1 Pin(GPIOA, GPIO_PIN_1)
#define PA2 Pin(GPIOA, GPIO_PIN_2)
#define PA3 Pin(GPIOA, GPIO_PIN_3)
#define PA4 Pin(GPIOA, GPIO_PIN_4)
#define PA5 Pin(GPIOA, GPIO_PIN_5)
#define PA6 Pin(GPIOA, GPIO_PIN_6)
#define PA7 Pin(GPIOA, GPIO_PIN_7)
#define PA8 Pin(GPIOA, GPIO_PIN_8)
#define PA9 Pin(GPIOA, GPIO_PIN_9)
#define PA10 Pin(GPIOA, GPIO_PIN_10)
#define PA11 Pin(GPIOA, GPIO_PIN_11)
#define PA12 Pin(GPIOA, GPIO_PIN_12)
#define PA13 Pin(GPIOA, GPIO_PIN_13)
#define PA14 Pin(GPIOA, GPIO_PIN_14)
#define PA15 Pin(GPIOA, GPIO_PIN_15)

#define PB0 Pin(GPIOB, GPIO_PIN_0)
#define PB1 Pin(GPIOB, GPIO_PIN_1)
#define PB2 Pin(GPIOB, GPIO_PIN_2)
#define PB3 Pin(GPIOB, GPIO_PIN_3)
#define PB4 Pin(GPIOB, GPIO_PIN_4)
#define PB5 Pin(GPIOB, GPIO_PIN_5)
#define PB6 Pin(GPIOB, GPIO_PIN_6)
#define PB7 Pin(GPIOB, GPIO_PIN_7)
#define PB8 Pin(GPIOB, GPIO_PIN_8)
#define PB9 Pin(GPIOB, GPIO_PIN_9)
#define PB10 Pin(GPIOB, GPIO_PIN_10)
#define PB11 Pin(GPIOB, GPIO_PIN_11)
#define PB12 Pin(GPIOB, GPIO_PIN_12)
#define PB13 Pin(GPIOB, GPIO_PIN_13)
#define PB14 Pin(GPIOB, GPIO_PIN_14)
#define PB15 Pin(GPIOB, GPIO_PIN_15)

#define PC0 Pin(GPIOC, GPIO_PIN_0)
#define PC1 Pin(GPIOC, GPIO_PIN_1)
#define PC2 Pin(GPIOC, GPIO_PIN_2)
#define PC3 Pin(GPIOC, GPIO_PIN_3)
#define PC4 Pin(GPIOC, GPIO_PIN_4)
#define PC5 Pin(GPIOC, GPIO_PIN_5)
#define PC6 Pin(GPIOC, GPIO_PIN_6)
#define PC7 Pin(GPIOC, GPIO_PIN_7)
#define PC8 Pin(GPIOC, GPIO_PIN_8)
#define PC9 Pin(GPIOC, GPIO_PIN_9)
#define PC10 Pin(GPIOC, GPIO_PIN_10)
#define PC11 Pin(GPIOC, GPIO_PIN_11)
#define PC12 Pin(GPIOC, GPIO_PIN_12)
#define PC13 Pin(GPIOC, GPIO_PIN_13)
#define PC14 Pin(GPIOC, GPIO_PIN_14)
#define PC15 Pin(GPIOC, GPIO_PIN_15)

#define PD0 Pin(GPIOD, GPIO_PIN_0)
#define PD1 Pin(GPIOD, GPIO_PIN_1)
#define PD2 Pin(GPIOD, GPIO_PIN_2)
#define PD3 Pin(GPIOD, GPIO_PIN_3)
#define PD4 Pin(GPIOD, GPIO_PIN_4)
#define PD5 Pin(GPIOD, GPIO_PIN_5)
#define PD6 Pin(GPIOD, GPIO_PIN_6)
#define PD7 Pin(GPIOD, GPIO_PIN_7)
#define PD8 Pin(GPIOD, GPIO_PIN_8)
#define PD9 Pin(GPIOD, GPIO_PIN_9)
#define PD10 Pin(GPIOD, GPIO_PIN_10)
#define PD11 Pin(GPIOD, GPIO_PIN_11)
#define PD12 Pin(GPIOD, GPIO_PIN_12)
#define PD13 Pin(GPIOD, GPIO_PIN_13)
#define PD14 Pin(GPIOD, GPIO_PIN_14)
#define PD15 Pin(GPIOD, GPIO_PIN_15)

#define PE0 Pin(GPIOE, GPIO_PIN_0)
#define PE1 Pin(GPIOE, GPIO_PIN_1)
#define PE2 Pin(GPIOE, GPIO_PIN_2)
#define PE3 Pin(GPIOE, GPIO_PIN_3)
#define PE4 Pin(GPIOE, GPIO_PIN_4)
#define PE5 Pin(GPIOE, GPIO_PIN_5)
#define PE6 Pin(GPIOE, GPIO_PIN_6)
#define PE7 Pin(GPIOE, GPIO_PIN_7)
#define PE8 Pin(GPIOE, GPIO_PIN_8)
#define PE9 Pin(GPIOE, GPIO_PIN_9)
#define PE10 Pin(GPIOE, GPIO_PIN_10)
#define PE11 Pin(GPIOE, GPIO_PIN_11)
#define PE12 Pin(GPIOE, GPIO_PIN_12)
#define PE13 Pin(GPIOE, GPIO_PIN_13)
#define PE14 Pin(GPIOE, GPIO_PIN_14)
#define PE15 Pin(GPIOE, GPIO_PIN_15)

#define PF0 Pin(GPIOF, GPIO_PIN_0)
#define PF1 Pin(GPIOF, GPIO_PIN_1)
#define PF2 Pin(GPIOF, GPIO_PIN_2)
#define PF3 Pin(GPIOF, GPIO_PIN_3)
#define PF4 Pin(GPIOF, GPIO_PIN_4)
#define PF5 Pin(GPIOF, GPIO_PIN_5)
#define PF6 Pin(GPIOF, GPIO_PIN_6)
#define PF7 Pin(GPIOF, GPIO_PIN_7)
#define PF8 Pin(GPIOF, GPIO_PIN_8)
#define PF9 Pin(GPIOF, GPIO_PIN_9)
#define PF10 Pin(GPIOF, GPIO_PIN_10)
#define PF11 Pin(GPIOF, GPIO_PIN_11)
#define PF12 Pin(GPIOF, GPIO_PIN_12)
#define PF13 Pin(GPIOF, GPIO_PIN_13)
#define PF14 Pin(GPIOF, GPIO_PIN_14)
#define PF15 Pin(GPIOF, GPIO_PIN_15)
