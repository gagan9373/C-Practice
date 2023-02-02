/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#include "ff.h"
static void COMMAND_ProgramFlashMemory(void);
FATFS USBH_fatfs;
FIL MyFile;
FRESULT res;
uint32_t bytesWritten;
uint8_t rtext[200];
uint8_t wtext[] = "USB Host Library : Mass Storage Example";
uint8_t name[10];//name of the file
uint16_t counter=0;
uint32_t i=0;
static int32_t uart_length=0;
extern char USBHPath [];  /* USBH logical drive path */
extern UART_HandleTypeDef huart3;
uint8_t uart_tx_buffer[100];
#define DOWNLOAD_FILENAME          "0:image.BIN"
FIL MyFileR;                    /* File object for download operation */
FILINFO MyFileInfo;	       /* File object information */
#define BUFFER_SIZE        ((uint16_t)512*64)
static uint32_t TmpReadSize = 0x00;
static uint32_t RamAddress = 0x00;
static __IO uint32_t LastPGAddress = APPLICATION_ADDRESS;
static uint8_t RAM_Buf[BUFFER_SIZE] = { 0x00 };


/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
void userFunction(void) {
	if (Appli_state == APPLICATION_READY) {
		uart_length=sprintf(uart_tx_buffer, "Press and release user button to start FW update \n");
		HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)!=GPIO_PIN_SET){}
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET){}
		/* Open the binary file to be downloaded */
		if (f_open(&MyFileR, DOWNLOAD_FILENAME, FA_READ) != FR_OK)
		{
			/*read size of binary file*/
			f_stat(DOWNLOAD_FILENAME,&MyFileInfo);
			/* The binary file is not available: Turn LED1, LED2 and LED4 On and Toggle
			 * LED3 in infinite loop */
			uart_length=sprintf(uart_tx_buffer, "The binary file is not available \n");
			HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
			while(1){}
		}
		if (MyFileInfo.fsize > USER_FLASH_SIZE)
		{
			/* No available Flash memory size for the binary file: Turn LED4 On and
			 * Toggle LED3 in infinite loop */
			uart_length=sprintf(uart_tx_buffer, "No available Flash memory size for the binary file \n");
			HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
			while(1){}
		}

		/* Download On Going: Turn LED4 On */
		uart_length=sprintf(uart_tx_buffer, "Download On Going \n");
		HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);

		// Begin TODO 1 MSC_DFU_HOST_HANDS_ON: Erase the flash sector to download the image
		#warning "TODO 1 MSC_DFU_HOST_HANDS_ON: Erase the flash sector to download the image"
		FLASH_If_FlashUnlock();
		if ( FLASH_If_EraseSectors(APPLICATION_ADDRESS) != 0){
			uart_length=sprintf(uart_tx_buffer, "Erase failed\n");
			HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
			while(1);
		}
		// End MSC_DFU_HOST_HANDS_ON: Erase the flash sector to download the image
		/* Program flash memory */
		COMMAND_ProgramFlashMemory();

		/* Download Done: Turn LED4 Off and LED2 On */
		uart_length=sprintf(uart_tx_buffer, "Download Done\n");
		HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);

		/* Close file */
		f_close(&MyFileR);
		uart_length=sprintf(uart_tx_buffer, "Application going to be reset in 5 seconds\n");
		HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
		HAL_Delay(5000);
		NVIC_SystemReset();
	}
}

static void COMMAND_ProgramFlashMemory(void)
{
	uint32_t programcounter = 0x00;
	uint8_t readflag = TRUE;
	uint16_t bytesread;

	/* RAM Address Initialization */
	RamAddress = (uint32_t) & RAM_Buf;
	/* Erase address init */
	LastPGAddress = APPLICATION_ADDRESS;
	/* While file still contain data */
	while ((readflag == TRUE))
	{
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&MyFileR, RAM_Buf, BUFFER_SIZE, (void *)&bytesread);

		/* Temp variable */
		TmpReadSize = bytesread;
		/* The read data < "BUFFER_SIZE" Kbyte */
		if (TmpReadSize < BUFFER_SIZE)
		{
			readflag = FALSE;
		}
		/* Program flash memory */
		for (programcounter = 0; programcounter < TmpReadSize; programcounter += 4)
		{
			// Begin TODO 2 MSC_DFU_HOST_HANDS_ON: Write word from ram into flash memory
			#warning "TODO 2 MSC_DFU_HOST_HANDS_ON: Write word into flash memory"
			if ( FLASH_If_Write((LastPGAddress+programcounter), *(uint32_t *)(RamAddress+programcounter) ) != 0 )
			{
				uart_length=sprintf(uart_tx_buffer, "Flash write failed\n");
				HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
				while(1);
			}

			// End MSC_DFU_HOST_HANDS_ON: Write word into flash memory

		}
		/* Update last programmed address value */
		LastPGAddress += TmpReadSize;
	}
}

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
  
  /* USER CODE END USB_HOST_Init_PreTreatment */
  
  /* Init host Library, add supported class and start the library. */
  USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS);

  USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS);

  USBH_Start(&hUsbHostFS);

  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
  
  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  uart_length=sprintf(uart_tx_buffer, "application ready \n");
  HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
  if(f_mount(&USBH_fatfs, USBHPath, 0) != FR_OK)
      {
    uart_length=sprintf(uart_tx_buffer, "f_mount fail \n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer,(uint16_t)uart_length, 1000);
      }
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
