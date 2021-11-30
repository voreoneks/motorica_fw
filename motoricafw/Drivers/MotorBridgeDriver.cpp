#include "MotorBridgeDriver.h"

MotorBridgeDriver::MotorBridgeDriver(TIM_HandleTypeDef& timerHandle, uint32_t channelCW, uint32_t channelCCW)
{
	htim = &timerHandle;
	cwChannel = channelCW;
	ccwChannel = channelCCW;
}
	
void MotorBridgeDriver::setPWM(float percent, bool ccw)
{
	uint32_t ch = ccw ? ccwChannel : cwChannel;
	uint32_t nch = ccw ? cwChannel : ccwChannel;
	__HAL_TIM_SET_COMPARE(htim, nch, 0);
	__HAL_TIM_SET_COMPARE(htim, ch, percent * 10);
}

void MotorBridgeDriver::startTimer()
{
	HAL_TIM_PWM_Start(htim, cwChannel);
	HAL_TIM_PWM_Start(htim, ccwChannel);
	HAL_TIMEx_PWMN_Start(htim, cwChannel);
	HAL_TIMEx_PWMN_Start(htim, ccwChannel);
}

void MotorBridgeDriver::stop()
{
	__HAL_TIM_SET_COMPARE(htim, cwChannel, 0);
	__HAL_TIM_SET_COMPARE(htim, ccwChannel, 0);
}

void MotorBridgeDriver::setDeadtime(uint32_t ticks)
{
	MODIFY_REG(htim->Instance->BDTR, TIM_BDTR_DTG, ticks);
}
