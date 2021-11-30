#pragma once

#include "driver.h"

class SpiDevice
{
public:
	SpiDevice(const SPI_HandleTypeDef* spi, const GPIO_TypeDef* chipSelectPort, const uint32_t chipSelectPin);
	int write(uint8_t* data, uint32_t size);
	int read(uint8_t* data, uint32_t size);
	void setTimeout(uint32_t value);
private:
	const SPI_HandleTypeDef* hspi;
	const GPIO_TypeDef* csPort;
	const uint32_t csPin;
	uint32_t timeout = 1;
};

