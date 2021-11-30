#pragma once

#include "SpiDevice.h"

class AD5160 : SpiDevice
{
public:
	AD5160(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint32_t csPin);
	void setPoint(float percent);
};
