#include "AD7191.h"

AD7191::AD7191(const float refVoltage,
	const SPI_HandleTypeDef* spi,
	const GPIO_TypeDef* pdownPort,
	const uint32_t pdownPin,
	const GPIO_TypeDef* ODR1Port,
	const uint32_t ODR1Pin,
	const GPIO_TypeDef* ODR2Port,
	const uint32_t ODR2Pin,
	const GPIO_TypeDef* PGA1Port,
	const uint32_t PGA1Pin,
	const GPIO_TypeDef* PGA2Port,
	const uint32_t PGA2Pin,
	const GPIO_TypeDef* RDYPort,
	const uint32_t RDYPin)
	: SpiDevice(spi, pdownPort, pdownPin),
	odr1port(ODR1Port),
	odr1pin(ODR1Pin),
	odr2port(ODR2Port),
	odr2pin(ODR2Pin),
	pga1port(PGA1Port),
	pga1pin(PGA1Pin),
	pga2port(PGA2Port),
	pga2pin(PGA2Pin),
	rdyport(RDYPort),
	rdypin(RDYPin),
	reference(refVoltage)
{
}

void AD7191::setSettingsPins(AD7191::DataRate rate, AD7191::Gain gain)
{
	HAL_GPIO_WritePin((GPIO_TypeDef*)odr1port, odr1pin, (GPIO_PinState)(((uint32_t)rate) & 0x1));
	HAL_GPIO_WritePin((GPIO_TypeDef*)odr2port, odr2pin, (GPIO_PinState)(((uint32_t)rate) & 0x2));
	HAL_GPIO_WritePin((GPIO_TypeDef*)pga1port, pga2pin, (GPIO_PinState)(((uint32_t)gain) & 0x1));
	HAL_GPIO_WritePin((GPIO_TypeDef*)pga2port, pga2pin, (GPIO_PinState)(((uint32_t)gain) & 0x2));
}

float AD7191::getVoltage()
{
	uint32_t value;
	if (read((uint8_t*)&value, 3) > 0)
		return ((float)value * reference) / 16777216.0;
	else
		return 0;
}
