#include "SpiDevice.h"

SpiDevice::SpiDevice(const SPI_HandleTypeDef* spi, const GPIO_TypeDef* chipSelectPort, const uint32_t chipSelectPin)
	: hspi(spi)
	, csPort(chipSelectPort)
	, csPin(chipSelectPin)
	, timeout(10)
{
}

int SpiDevice::write(uint8_t* data, uint32_t size)
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin((GPIO_TypeDef*)csPort, csPin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit((SPI_HandleTypeDef*)hspi, data, size, timeout);
	HAL_GPIO_WritePin((GPIO_TypeDef*)csPort, csPin, GPIO_PIN_SET);
	if (status == HAL_OK)
		return size;
	else
		return -status;
}

int SpiDevice::read(uint8_t* data, uint32_t size)
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin((GPIO_TypeDef*)csPort, csPin, GPIO_PIN_RESET);
	status = HAL_SPI_Receive((SPI_HandleTypeDef*)hspi, data, size, timeout);
	HAL_GPIO_WritePin((GPIO_TypeDef*)csPort, csPin, GPIO_PIN_SET);
	if (status == HAL_OK)
		return size;
	else
		return -status;
}

void SpiDevice::setTimeout(uint32_t value)
{
	timeout = value;
}
