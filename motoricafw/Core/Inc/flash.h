#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{

#endif
	void FlashWrite(uint32_t address, uint8_t *data, uint32_t size);

	void FlashRead(uint32_t address, uint8_t *data, uint32_t size);
	#ifdef __cplusplus
}
#endif