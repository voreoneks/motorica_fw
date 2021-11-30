#pragma once

#include "stdint.h"
#include "tim.h"
#include "gpio.h"
#include "driver.h"

class Led
{
public:
	Led(GPIO_TypeDef* port, uint16_t pin, bool isSink = true) :
		ioPort(port), ioPin(pin), sink(isSink) {}
	void toggle();
	void setState(bool state);
	void on();
	void off();

private:
	GPIO_TypeDef* ioPort = nullptr;
	uint16_t ioPin = 0;
	bool sink = false;
};
