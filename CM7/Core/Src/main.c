/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "fatfs.h"
#include "quadspi.h"
#include "rng.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1362.h"
#include "pictures.h"
#include "stm32_flash.h"

#include "config.h"
#include "shared.h"
#include "basetypes.h"
#include "stdio.h"

#include "ff.h" // FATFS include
#include "diskio.h" // DiskIO include

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FATFS fs;   // Filesystem object

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* SD card parameters */
#define SD_CARD_PATH "/"
#define FILE_NAME "CM7.bin"
#define FILE_NAME_2 "CM4.bin"
//#define FILE_NAME_3 "QSPI.bin"

/* Buffer size for reading data */
#define BUFFER_SIZE 512

#define FW_CM7_START_ADDR 0x08040000
#define FW_CM4_START_ADDR 0x08100000

typedef void (*pFunction)(void);


/**
 * @brief Jump to application from the Bootloader
 * @retval None
 */
static void gotoFirmware(uint32_t fwFlashStartAdd)
{
	//  void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (0x08040000 + 4U)));
	//
	//  //__set_MSP(*(volatile uint32_t*) 0x08040000);
	//  app_reset_handler();    //call the app reset handler

	//	SCB_DisableDCache();
	//
	//	SCB_DisableICache();
	//
	HAL_MPU_Disable();
	//
	HAL_SuspendTick();
	//
	__disable_irq(); //disable interrupt
	//
	SysTick->CTRL = 0;  // Enabled in application
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;
	//
	HAL_RCC_DeInit();
	//
	for(uint8_t i = 0; i < 8; i++) //clear all NVIC Enable and Pending registers
	{
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}
	//
	__enable_irq();

	//	void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (0x08040000 + 4U)));
	//	__set_MSP(*(__IO uint32_t*) 0x08040000);
	//
	//	__set_CONTROL(0); //priviage mode
	//	__set_PRIMASK(1);
	//
	//	SCB->VTOR = 0x08040000;
	//
	//	app_reset_handler();

	pFunction appEntry;
	uint32_t appStack;

	appStack = (uint32_t) *((__IO uint32_t*)fwFlashStartAdd);
	appEntry = (pFunction) *(__IO uint32_t*) (fwFlashStartAdd + 4);
	__DMB();
	SCB->VTOR = fwFlashStartAdd;
	__DSB();
	SysTick->CTRL = 0x0;
	HAL_DeInit();
	__set_MSP(appStack);
	appEntry();
}

bool updateFirmware(const TCHAR* fwPath, uint32_t flashBank, uint32_t flashSector, uint32_t NbSectors, uint32_t fwFlashStartAdd)
{
    uint8_t binFileRes[50], readBytes[BUFFER_SIZE];
    UINT bytesRead;
    FSIZE_t file_size;
    FIL file;
    uint32_t flashAdd, addCNTR;
    HAL_StatusTypeDef status;

    if (f_open(&file, fwPath, FA_READ) == FR_OK) {

        file_size = f_size(&file);

        sprintf((char *)binFileRes, ".bin Size: %lu bytes \n\r", file_size);
        HAL_UART_Transmit(&huart1, binFileRes, 20, 100);

        status = STM32Flash_erase_app_memory(flashBank, flashSector, NbSectors);
        if (status != HAL_OK) {
            HAL_UART_Transmit(&huart1, (uint8_t *)"Erase Failed\n\r", 14, 100);
            f_close(&file);
            return false;
        }

        flashAdd = fwFlashStartAdd;
        addCNTR  = 0;

        while (f_read(&file, readBytes, BUFFER_SIZE, &bytesRead) == FR_OK && bytesRead > 0) {
            for (uint32_t i = 0; i < 16; i++) {
                if ((flashAdd + addCNTR) % 4 != 0) {
                    HAL_UART_Transmit(&huart1, (uint8_t *)"Address Misaligned\n\r", 21, 100);
                    f_close(&file);
                    return false;
                }

                status = STM32Flash_write32B(readBytes + (i * 32), flashAdd + addCNTR);
                if (status != HAL_OK) {
                    HAL_UART_Transmit(&huart1, (uint8_t *)"Write Failed\n\r", 14, 100);
                    f_close(&file);
                    return false;
                }
                addCNTR += 32;
            }
            memset(readBytes, 0xFF, sizeof(readBytes));
        }

        f_close(&file);
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t *)"File Open Failed\n\r", 18, 100);
        return false;
    }

    return true;
}

bool updateExternalFlash(const TCHAR* fwPath)
{
	uint8_t binFileRes[50], readBytes[BUFFER_SIZE];
	UINT bytesRead;
	FSIZE_t file_size;
	FIL file;
	uint32_t addCNTR;

	if (f_open(&file, fwPath, FA_READ) == FR_OK) {

		file_size = f_size(&file);

		sprintf((char *)binFileRes, ".bin Size: %lu bytes \n\r", file_size);
		HAL_UART_Transmit(&huart1, binFileRes, 20, 100);

		//		if (CSP_QSPI_Erase_Chip() != HAL_OK)
		//		{
		//			Error_Handler();
		//		}

		addCNTR  = 0;

		while (f_read(&file, readBytes, BUFFER_SIZE, &bytesRead) == FR_OK && bytesRead > 0) {
			// Process the read data here

			//			if (CSP_QSPI_WriteMemory(readBytes, addCNTR, sizeof(readBytes)) != HAL_OK)
			//			{
			//				Error_Handler();
			//			}

			addCNTR += BUFFER_SIZE;
			memset(readBytes, 0xFF, sizeof(readBytes));
		}
		f_close(&file);
	}

	return true;
}

void gui_displayUpdateStarted(void)
{
	ssd1362_clearBuffer();

	// Draw border
	ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, true);
	ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

	ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

	ssd1362_drawString(0, 45, 					  (int8_t *)"        DO NOT POWER OFF      ", 0xF, 8);

	// Display the frame buffer
	ssd1362_writeFullBuffer();
}

void gui_displayUpdateFailed(void)
{
	ssd1362_clearBuffer();

	// Draw border
	ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, false);
	ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

	ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

	ssd1362_drawString(0, 25, 					  (int8_t *)"         UPDATE FAILED        ", 0xF, 8);

	ssd1362_drawString(0, 45, 					  (int8_t *)"        DO NOT POWER OFF      ", 0xF, 8);

	// Display the frame buffer
	ssd1362_writeFullBuffer();
}

void gui_displayUpdateWrited(void)
{
	ssd1362_clearBuffer();

	// Draw border
	ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, false);
	ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

	ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

	ssd1362_drawString(0, 25, 					  (int8_t *)"     WRITE SUCCESS - REBOOT   ", 0xF, 8);

	ssd1362_drawString(0, 45, 					  (int8_t *)"        DO NOT POWER OFF      ", 0xF, 8);

	// Display the frame buffer
	ssd1362_writeFullBuffer();
}

void gui_displayUpdateSuccess(void)
{
	ssd1362_clearBuffer();

	// Draw border
	ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, true);
	ssd1362_drawRect(0, 0, 255, 63, 0xF, false);

	ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

	ssd1362_drawString(0, 25, 					  (int8_t *)"    FIRMWARE UPDATE SUCCESS   ", 0xF, 8);

	ssd1362_drawString(0, 45, 					  (int8_t *)"            REBOOT            ", 0xF, 8);

	// Display the frame buffer
	ssd1362_writeFullBuffer();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */

	/* USER CODE END Boot_Mode_Sequence_0 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();
	/* Enable the CPU Cache */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */

	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */

	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	PersistentData dataRead;
	STM32Flash_readPersistentData(&dataRead);

	if (dataRead.updateState == FW_UPDATE_NONE)
	{
		gotoFirmware(FW_CM7_START_ADDR);
	}

	if (dataRead.updateState == FW_UPDATE_FLASHED)
	{
	    PersistentData dataToWrite = {
	        .updateState = FW_UPDATE_TO_TEST,
	        .padding = {0}
	    };

		/* reboot after we close the connection. */
	    HAL_StatusTypeDef status = STM32Flash_writePersistentData(&dataToWrite);
	    if (status == HAL_OK)
	    {
	        printf("Firmware update must be tested now \n");
	    }
	    else
	    {
	        printf("Failed to write firmware update status in STM32 flash\n");
	    }

		gotoFirmware(FW_CM7_START_ADDR);
	}

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_FMC_Init();
	MX_USART1_UART_Init();
	MX_RNG_Init();
	MX_CRC_Init();
	MX_QUADSPI_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	printf("START BOOTLOADER\n");

	ssd1362_init();
	ssd1362_clearBuffer();
	ssd1362_writeFullBuffer();

	if (dataRead.updateState == FW_UPDATE_DONE)
	{
	    PersistentData dataToWrite = {
	        .updateState = FW_UPDATE_NONE,
	        .padding = {0}
	    };

		/* reboot after we close the connection. */
	    HAL_StatusTypeDef status = STM32Flash_writePersistentData(&dataToWrite);
	    if (status == HAL_OK)
	    {
	        printf("Firmware update done, reset firmware update flag\n");
	    }
	    else
	    {
	        printf("Failed to write firmware update status in STM32 flash\n");
	    }

		gui_displayUpdateSuccess();

		printf("Rebooting in 3\n");
		/* Wait 3 seconds. */
		HAL_Delay(3000);
		NVIC_SystemReset();
	}

	gui_displayUpdateStarted();

	printf("- FILE INITIALIZATION -\n");

	FRESULT fres; // Variable to store the result of FATFS operations

	// Attempt to mount the file system

	fres = f_mount(&fs, "0:", 1); // 1 to mount immediately
	if (fres != FR_OK)
	{
		printf("FS mount ERROR\n");
		gui_displayUpdateFailed();
	}
	else
	{
		printf("FS mount SUCCESS\n");

		// UPDATE CM7 FLASH REGION
		updateFirmware(FILE_NAME, FLASH_BANK_1, FLASH_SECTOR_2, 6, FW_CM7_START_ADDR);

		// UPDATE CM4 FLASH REGION
		updateFirmware(FILE_NAME_2, FLASH_BANK_2, FLASH_SECTOR_0, 8, FW_CM4_START_ADDR);

		// UPDATE QSPI
		//updateExternalFlash(FILE_NAME_3);
	}

	printf("Preparing to reset all cores \n");

    PersistentData dataToWrite = {
        .updateState = FW_UPDATE_FLASHED,
        .padding = {0}
    };

	/* reboot after we close the connection. */
    HAL_StatusTypeDef status = STM32Flash_writePersistentData(&dataToWrite);
    if (status == HAL_OK)
    {
        printf("Firmware update flashed\n");
    }
    else
    {
        printf("Failed to write firmware update status in STM32 flash\n");
    }

    gui_displayUpdateWrited();

	printf("Rebooting in 3\n");
	/* Wait 5 seconds. */
	HAL_Delay(3000);
	NVIC_SystemReset();

	// Reset all required system peripherals and disable interrupts
	HAL_SRAM_MspDeInit(&hsram1);
	HAL_UART_MspDeInit(&huart1);
	HAL_RNG_MspDeInit(&hrng);
	HAL_CRC_MspDeInit(&hcrc);
	HAL_QSPI_MspDeInit(&hqspi);

	// JUMP TO THE NEW APPLICATON
	gotoFirmware(FW_CM7_START_ADDR);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	RCC_OscInitStruct.PLL.PLLR = 4;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{

	/* Disables the MPU */
	HAL_MPU_Disable();
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM2) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
