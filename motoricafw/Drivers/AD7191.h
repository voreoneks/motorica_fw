#pragma once

#include "SpiDevice.h"
#include "pins.h"

class AD7191 : SpiDevice
{
public:
	enum DataRate
	{
		Rate120Hz = 0,
		Rate60Hz,
		Rate50Hz,
		Rate10Hz
	};
	enum Gain
	{
		x1 = 0,
		x2,
		x64,
		x128
	};
	AD7191(const float refVoltage,
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
		const uint32_t RDYPin);
	void setSettingsPins(DataRate rate, Gain gain);
	float getVoltage();
	float getData();
	float linearInterp();
	float u[6] = { 149.01, 29921.53, 59694.05, 89466.57, 119239.09, 149011.61};
	float p[6] = { 0, 200, 400, 600, 800, 1000 };
private:
	const GPIO_TypeDef* odr1port;
	const uint32_t odr1pin;
	const GPIO_TypeDef* odr2port;
	const uint32_t odr2pin;
	const GPIO_TypeDef* pga1port;
	const uint32_t pga1pin;
	const GPIO_TypeDef* pga2port;
	const uint32_t pga2pin;
	const GPIO_TypeDef* rdyport;
	const uint32_t rdypin;
	const float reference;
};
