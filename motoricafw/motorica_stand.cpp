#include "motorica_stand.h"
#include "motorica_stand_bsp.h"
#include "usb_device.h"
#include "AD5160.h"
#include "AD7191.h"
#include "MAX31855.h"
#include "Led.h"
#include "flash.h"
#include "MotorBridgeDriver.h"
#include "usermath.hpp"

constexpr uint32_t CalibrationFlashAddr = FLASH_END - 1024;

extern MAX31855 tempSensor;
extern AD5160 dcdcPot;
extern AD7191 loadCellAdc;
extern MotorBridgeDriver motorDriver;
extern Led ledRed;
extern Led ledGreen;
extern Led ledBlue;

Stand::Stand()
{
}

void Stand::parseControlPacket(uint8_t* data)
{
	float value;
	switch (data[0])
	{
	case DevCmd::Stop:
		sendStatusContinuously = false;
		sendCycleDataContinuously = false;
		enableCycleDataSendingCounter(false);
		break;
	case DevCmd::Start:
		memcpy(&settings, data + 1, sizeof(settings));
		stopAtCurrent = (settings.endingFlags & 0x01) != 0;
		stopAtForce = (settings.endingFlags & 0x02) != 0;
		stopAtTime = (settings.endingFlags & 0x04) != 0;
		if (stopAtCurrent)
			BSP_SetMotorCurrentLimit(settings.pressingMaxCurrent);
		sendStatusPacket(true);
		sendStatusContinuously = true;
		enableCycleDataSendingCounter(true);
		break;
	case DevCmd::Ack:
		sendCycleDataContinuously = false;
		cycleDataPacketCnt = 0;
		break;
	case DevCmd::Compress:
		ledRed.toggle();
		break;
	case DevCmd::Decompress:
		ledGreen.toggle();
		break;
	case DevCmd::SetServoAngle:
		break;
	case DevCmd::SetVoltage:
		memcpy(&value, data + 1, sizeof(float));
		BSP_SetDCDCVoltage(value);
		break;
	case DevCmd::CalibrationMode:
		break;
	case DevCmd::EraseCalibration:
		break;
	case DevCmd::CancelCalibration:
		break;
	case DevCmd::CalibrationWeight:
		memcpy(&value, data + 1, sizeof(float));
		forceCallibration.weight[weightNum] = value;
		if (weightNum < sizeof(forceCallibration.weight) / sizeof(forceCallibration.weight[0]))
			weightNum++;
		break;
	default:
		break;
	}
}

void Stand::sendStatusPacket(bool ack)
{
	DevCurrentData data;
	data.currentState = 1;
	data.dcCurrent = 6.5 + (float)(rand() % 100) / 1000;
	data.dcVoltage = 12 + (float)(rand() % 1000) / 1000;
	data.force = 0.45 + (float)(rand() % 100) / 1000;
	data.motorTemp = 66 + (float)(rand() % 1000) / 1000;
	data.noiseLevel = 34 + (float)(rand() % 1000) / 1000;
	data.calibrationSuccess = true;
	data.cmdAck = ack;
	data.faultFlags = faults;
	MX_USB_DEVICE_SetTxCmd(2, DevCmd::CurrentData, (uint8_t*)&data, 62);
}

void Stand::sendCycleDataPacket()
{
	DevCycleData cycleData;
	cycleData.number = 1;
	cycleData.duration = 5 + (float)(rand() % 100) / 1000;
	cycleData.dcNoLoadCurrentPeak= 2 + (float)(rand() % 100) / 1000;
	cycleData.dcCurrentPeak = 8 + (float)(rand() % 100) / 1000;
	cycleData.forcePeak = 12 + (float)(rand() % 100) / 1000;
	cycleData.motorTemp = 60 + (float)(rand() % 1000) / 1000;
	cycleData.noiseLevelPeak = 30 + (float)(rand() % 1000) / 1000;
	cycleData.peakPower = 6 + (float)(rand() % 100) / 1000;
	MX_USB_DEVICE_SetTxCmd(2, DevCmd::CycleData, (uint8_t*)&cycleData, 62);
}

void Stand::motorBlockingCallback()
{
	motorDriver.stop();
	
	switch (cycleState)
	{
	case CycleState::Load:
		cycleState = CycleState::AtEnd;
		break;
	case CycleState::Unload:
		currentCycle++;
		cycleState = CycleState::AtStart;
		break;
	default:
		break;
	}
}

void Stand::prosthesisTest()
{
	switch (cycleState)
	{
	case CycleState::Off:
		break;
	case CycleState::AtStart:
		if (currentCycle < settings.cyclesNum)
		{
			motorDriver.setPWM(100, false);
			cycleState = CycleState::Load;
		}
		else
			cycleState = CycleState::Off;
		break;
	case CycleState::Load:
		break;
	case CycleState::AtEnd:
		motorDriver.setPWM(100, true);
		cycleState = CycleState::Unload;
		break;
	case CycleState::Unload:
		
		break;
	default:
		break;
	}
}

void Stand::calibrationWrite()
{
	FlashWrite(CalibrationFlashAddr, (uint8_t*)&forceCallibration, sizeof(forceCallibration));
}

void Stand::calibrationRead()
{
	FlashRead(CalibrationFlashAddr, (uint8_t*)&forceCallibration, sizeof(forceCallibration));
}

float cellVoltage = 0;

void Stand::mainThread()
{	
	usbTransferManage();
	
	cellVoltage = loadCellAdc.getVoltage();

	switch (settings.mode)
	{
	case SessionMode::DCMotor:
		break;
	case SessionMode::Servo:
		break;
	case SessionMode::Prosthesis:
		prosthesisTest();
		break;
	default:
		break;
	}
}

void Stand::tempSensorFaultCallback()
{
	faults |= Faults::TempSensor;
}

void Stand::usbTransferManage()
{
	uint8_t usbReportId;
	static uint8_t usbReceivedData[63];

	if (MX_USB_DEVICE_CheckForRxData(&usbReportId, usbReceivedData))
		parseControlPacket(usbReceivedData);
	
	if (MX_USB_DEVICE_CheckTxDone())
	{
		if (sendCycleDataContinuously && (cycleDataPacketCnt < cycleDataPacketCntMax))
		{
			sendCycleDataPacket();
			cycleDataPacketCnt++;
		}
		else if (sendStatusContinuously)
			sendStatusPacket(false);
	}
	
	if (!MX_USB_DEVICE_Busy())
		MX_USB_DEVICE_CheckedWrite();
}


void Stand::startCycleDataSending()
{
	sendCycleDataContinuously = true;
}


void Stand::enableCycleDataSendingCounter(bool enable)
{
	periodicCycleDataSending = enable;
}

bool Stand::getPeriodicCycleDataSendFlag()
{
	return periodicCycleDataSending;
}
