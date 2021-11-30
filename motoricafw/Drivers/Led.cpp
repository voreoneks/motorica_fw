#include "Led.h"

void Led::toggle()
{
	HAL_GPIO_TogglePin(ioPort, ioPin);
}

void Led::setState(bool state)
{
	HAL_GPIO_WritePin(ioPort, ioPin, (GPIO_PinState)(state ^ !sink));
}

void Led::on()
{
	HAL_GPIO_WritePin(ioPort, ioPin, (GPIO_PinState)(!sink));
}

void Led::off()
{
	HAL_GPIO_WritePin(ioPort, ioPin, (GPIO_PinState)(sink));
}
