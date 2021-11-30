#pragma once

#include "driver.h"

class MotorBridgeDriver
{
public:
	MotorBridgeDriver(TIM_HandleTypeDef& timerHandle, uint32_t channelCW, uint32_t channelCCW);
	void setPWM(float percent, bool ccw);
	void startTimer();
	void stop();
	void setDeadtime(uint32_t ticks);
private:
	TIM_HandleTypeDef* htim;
	uint32_t cwChannel, ccwChannel;
};

