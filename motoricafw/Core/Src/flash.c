#include "flash.h"
#include "driver.h"

void FlashWrite(uint32_t address, uint8_t *data, uint32_t size)
{
	HAL_FLASH_Unlock();
	
	uint32_t words = size / 4;
	uint32_t wordBytes = words * 4;
	
	for (uint32_t i = 0; i < words; i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4, data[i * 4]);
	
	if (size % 4)
	{
		for (uint32_t i = 0; i < size % 4; i++)
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + wordBytes + i, data[wordBytes + i]);
	}
	HAL_FLASH_Lock();
}

void FlashRead(uint32_t address, uint8_t *data, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++)
	{
		data[i] = *(uint8_t *)(address + i); 
	}
}
