#include "motorica_stand_bsp.h"
#include "motorica_stand.h"
#include "AD5160.h"
#include "AD7191.h"
#include "MAX31855.h"
#include "MotorBridgeDriver.h"
#include "Led.h"
#include "spi.h"
#include "adc.h"
#include "usb_device.h"
#include "flash.h"
#include "math.h"

constexpr float MotorShuntResistance = 0.01f;
constexpr float STM32_ADC_Vref = 3.3f;

MAX31855 tempSensor(&hspi1, TEMP_CS_GPIO_Port, TEMP_CS_Pin);
AD5160 dcdcPot(&hspi1, POT_CS_GPIO_Port, POT_CS_Pin);
AD7191 loadCellAdc(2.5f, &hspi3, ADC_PDOWN_GPIO_Port, ADC_PDOWN_Pin,
								ADC_ODR1_GPIO_Port, ADC_ODR1_Pin,
								ADC_ODR2_GPIO_Port, ADC_ODR2_Pin,
								ADC_PGA1_GPIO_Port, ADC_PGA1_Pin,
								ADC_PGA2_GPIO_Port, ADC_PGA2_Pin,
								ADC_RDY_GPIO_Port, ADC_RDY_Pin);
Led ledRed(LED_RED_GPIO_Port, LED_RED_Pin, false),
	ledGreen(LED_GREEN_GPIO_Port, LED_GREEN_Pin, false),
	ledBlue(LED_BLUE_GPIO_Port, LED_BLUE_Pin, false);

MotorBridgeDriver motorDriver(htim8, TIM_CHANNEL_2, TIM_CHANNEL_3);

Stand stand;

float getDCCurrent(float voltage)
{
	return voltage / MotorShuntResistance;
}

void BSP_SetDCDCVoltage(float voltage)
{
	float r = -375.162736f + 
			  176.16148f * voltage + 
			  -31.383098f * powf(voltage, 2) + 
			  2.9906926f * powf(voltage, 3) + 
			  -0.1459665f * powf(voltage, 4) + 
			  0.0028699902f * powf(voltage, 5);
	dcdcPot.setPoint(r);
}

void setServoAngle(float degrees)
{
	TIM3->CCR1 = degrees / 360.0f * 1000;
}

void BSP_SetMotorCurrentLimit(float amps)
{
	hadc1.Instance->HTR = MotorShuntResistance * amps / STM32_ADC_Vref * 4096;
}

void BSP_SaveForceCalibration()
{
	stand.calibrationWrite();
}

void BSP_LoadForceCalibration()
{
	stand.calibrationRead();
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}

extern "C" void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	stand.motorBlockingCallback();
}

/**
 * @brief Startup init
 *
 * Executed at startup, before MainThread()
 */
void BSP_Init()
{
	ledGreen.on();
	BSP_LoadForceCalibration();
	tempSensor.onFault = []{ stand.tempSensorFaultCallback(); };
	BSP_SetDCDCVoltage(14);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}

/**
 * @brief Main thread
 *
 * All work is there
 */
void MainThread()
{	
	stand.mainThread();
}

void EverySecondCallback()
{
	static uint32_t cnt = 0;
	if (cnt < 5)
		cnt++;
	else
	{
		cnt = 0;
		if (stand.getPeriodicCycleDataSendFlag())
			stand.startCycleDataSending();
	}
}