#pragma once

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus
	void BSP_Init();
	void MainThread();
	void BSP_SaveForceCalibration();
	void BSP_LoadForceCalibration();
	void BSP_SetDCDCVoltage(float voltage);
	void BSP_SetMotorCurrentLimit(float amps);
	void EverySecondCallback();
#ifdef __cplusplus
}
#endif // __cplusplus