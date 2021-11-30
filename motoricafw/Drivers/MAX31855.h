#pragma once

#include "SpiDevice.h"

class MAX31855 : SpiDevice
{
public:
	MAX31855(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint32_t csPin);
	float getTemp();
	void clearFaultCounter();
	void(*onFault)() = 0;
private:
	uint32_t faultCounter = 0;
};
