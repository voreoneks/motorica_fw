#pragma once

#include "stdint.h"

enum SessionMode
{
	DCMotor    = 0,
	Servo,
	Prosthesis
};

enum CycleState
{
	Off     = 0,
	AtStart,
	Load,
	AtEnd,
	Unload
};

enum DevCmd
{
    // Output packets
    CurrentData = 0,
    CycleData,

    // Input packets
    Stop,
    Start,
	Ack,
    Compress,
    Decompress,
    SetServoAngle,
    SetVoltage,

    CalibrationMode,
    EraseCalibration,
    CancelCalibration,
    CalibrationWeight
};

struct CalibrationData
{
    float weight[5];
    float voltage[5];
};

namespace Faults
{
	constexpr uint32_t TempSensor = 0x01;
}

#pragma pack(push, 1)
typedef struct
{
    uint8_t currentState;
    uint8_t calibrationSuccess;
	uint8_t cmdAck;
	uint8_t faultFlags;
    float dcCurrent;
    float dcVoltage;
    float force;
    float noiseLevel;
    float motorTemp;
} DevCurrentData; // sends continuously

typedef struct
{
	uint32_t number;
	float dcVoltageAtPress;
	float dcCurrentPeak;
	float dcNoLoadCurrentPeak;
	float forcePeak;
	float noiseLevelPeak;
	float peakPower;
	float motorTemp;
	float duration;
} DevCycleData; // sends at end of current cycle

typedef struct
{
    SessionMode mode;
    uint8_t endingFlags;
    uint32_t cyclesNum;
    float pressingMaxCurrent;
    float pressingMaxForce;
    float pressingTime;
    float coolingTime;
    float maxCycleTime;
    uint8_t maxCycleOvertimesNum;
    uint8_t maxTemperature;
    uint8_t servoType;
    uint16_t minServoAngle;
    uint16_t maxServoAngle;
} SessionSettings; // sends once at start of session
#pragma pack(pop)

class Stand
{
public:
	Stand();
	void calibrationWrite();
	void calibrationRead();
	void motorBlockingCallback();
	void parseControlPacket(uint8_t* data);
	void startCycleDataSending();
	void enableCycleDataSendingCounter(bool enable);
	bool getPeriodicCycleDataSendFlag();
	void sendStatusPacket(bool ack);
	void sendCycleDataPacket();
	void prosthesisTest();
	void mainThread();
	void tempSensorFaultCallback();
    void usbTransferManage();

private:
    SessionSettings settings;
	CycleState cycleState;
	uint32_t currentCycle;
    bool stopAtCurrent, stopAtTime, stopAtForce;
	float servoAngle;
	uint8_t faults;
	
	// Calibration variables
	CalibrationData forceCallibration;
	uint32_t weightNum;
	
	// USB transfer variables
	bool sendStatusContinuously = false;
	bool sendCycleDataContinuously = false;
	bool periodicCycleDataSending = false;
	uint32_t cycleDataPacketCnt = 0;
	const uint32_t cycleDataPacketCntMax = 500;
};

