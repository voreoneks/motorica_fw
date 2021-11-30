#include "AD5160.h"
#include "math.h"

AD5160::AD5160(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint32_t csPin)
	: SpiDevice(spi, csPort, csPin)
{
	
}

void AD5160::setPoint(float percent)
{
	uint8_t value = roundf(percent / 100.0f * 255.0f);
	write(&value, 1);
}
