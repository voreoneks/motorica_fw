/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t usbRxNewMsgFlag = 0;
uint8_t* usbRxDataPtr = NULL;
uint8_t usbRxReportId = 0;

uint8_t usbTxNewMsgFlag = 0;
uint8_t usbTxDataBuffer[USB_FS_MAX_PACKET_SIZE];
uint8_t usbTxDataSize = 0;
uint32_t usbTxAttempts = 0;
const uint32_t usbTxMaxAttempts = 10000;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
void MX_USB_DEVICE_Received(uint8_t reportId, uint8_t* data)
{
	usbRxDataPtr = data;
	usbRxReportId = reportId;
	usbRxNewMsgFlag = 1;
}

uint8_t MX_USB_DEVICE_CheckForRxData(uint8_t* reportId, uint8_t* data)
{
	if (usbRxNewMsgFlag != 0)
	{
		memcpy(data, usbRxDataPtr, CUSTOM_HID_EPOUT_SIZE - 1);
		*reportId = usbRxReportId;
		usbRxNewMsgFlag = 0;
		return 1;
	}
	else
		return 0;
}

uint8_t MX_USB_DEVICE_CheckTxDone()
{
	return usbTxNewMsgFlag == 0;
}

void MX_USB_DEVICE_SetTxData(uint8_t reportId, uint8_t* data, uint8_t size)
{
	if (usbTxNewMsgFlag == 0)
	{
		memcpy(usbTxDataBuffer + 1, data, size);
		usbTxDataBuffer[0] = reportId;
		usbTxNewMsgFlag = 1;
	}
}

void MX_USB_DEVICE_SetTxCmd(uint8_t reportId, uint8_t cmd, uint8_t* arg, uint8_t argSize)
{
	if (usbTxNewMsgFlag == 0)
	{
		memcpy(usbTxDataBuffer + 2, arg, argSize);
		usbTxDataBuffer[0] = reportId;
		usbTxDataBuffer[1] = cmd;
		usbTxNewMsgFlag = 1;
	}
}

void MX_USB_DEVICE_CheckedWrite()
{
	if (usbTxNewMsgFlag != 0)
	{
		int8_t status = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usbTxDataBuffer, USB_FS_MAX_PACKET_SIZE);
		if ((status == USBD_OK) || (usbTxAttempts++ > usbTxMaxAttempts))
		{
			usbTxNewMsgFlag = 0;
			usbTxAttempts = 0;
		}
	}
}

uint8_t MX_USB_DEVICE_Busy()
{
	if (hUsbDeviceFS.pClassData != NULL)
	{
		USBD_CUSTOM_HID_HandleTypeDef* hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
		return hhid->state == CUSTOM_HID_BUSY;
	}
	else
		return 1;
}

void MX_USB_Connect()
{
	USBD_Start(&hUsbDeviceFS);
}

void MX_USB_Disconnect()
{
	USBD_Stop(&hUsbDeviceFS);
}
/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CUSTOM_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CUSTOM_HID_RegisterInterface(&hUsbDeviceFS, &USBD_CustomHID_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
