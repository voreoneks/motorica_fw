#include "MAX31855.h"

MAX31855::MAX31855(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint32_t csPin)
	: SpiDevice(spi, csPort, csPin)
{
	
}

float MAX31855::getTemp()
{
	uint32_t value;
	read((uint8_t*)&value, 4);
	float temp = (float)(value >> 18) / 4.0f;
	float coldTemp = (float)((value >> 4) & (~0xFFFF00)) / 16.0f;
	
	if(value & (1 << 16))
	{
		faultCounter++;
		if (onFault)
			onFault();
	}
	return temp;
}

void MAX31855::clearFaultCounter()
{
	faultCounter = 0;
}
