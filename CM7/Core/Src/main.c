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
#include "basetypes.h"
#include "stdio.h"
#include "stdbool.h"

#include "iwdg.h"

#include "stm32_flash.h"
#include "config.h"
#include "globals.h"
#include "update.h"
#include "update_gui.h"

#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_flash.h"

#include "ff.h" // FATFS include
#include "diskio.h" // DiskIO include

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FATFS fs;   // Filesystem object

typedef void (*pFunction)(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* Function prototypes */
static bool findPackageFile(char* packageFilePath, size_t maxLen);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "stm32h7xx_hal.h"

void disableCM4Boot(void)
{
	  HAL_FLASH_Unlock();
	  HAL_FLASH_OB_Unlock();

	  if (READ_BIT(SYSCFG->UR1, SYSCFG_UR1_BCM4)) // Check if CM4 boot is enabled
	  {
	      FLASH_OBProgramInitTypeDef obConfig;

	      // Configure CM4 boot to "Disabled"
	      obConfig.OptionType = OPTIONBYTE_USER; // Specify that we are modifying user options
	      obConfig.USERType = OB_USER_BCM4; // Target the CM4 boot configuration
	      obConfig.USERConfig = OB_BCM4_DISABLE; // Disable CM4 boot

	      // Apply the new configuration
	      if (HAL_FLASHEx_OBProgram(&obConfig) != HAL_OK)
	      {
	          // Handle error if necessary
	          while (1);
	      }

	      // Launch option bytes reconfiguration
	      if (HAL_FLASH_OB_Launch() != HAL_OK)
	      {
	          // Handle error if necessary
	          while (1);
	      }

	      // Restart the system
	      HAL_NVIC_SystemReset();
	  }

	  HAL_FLASH_OB_Lock();
	  HAL_FLASH_Lock();
}

/**
 * @brief Jump to application from the Bootloader
 * @retval None
 */
static void gotoFirmware(uint32_t fwFlashStartAdd)
{
    HAL_MPU_Disable();
    HAL_SuspendTick();

    __disable_irq(); // Disable interrupt

    SysTick->CTRL = 0;  // Disable SysTick
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    HAL_RCC_DeInit();

    for (uint8_t i = 0; i < 8; i++) // Clear all NVIC Enable and Pending registers
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Validate firmware address
    uint32_t appStack = *((volatile uint32_t*) fwFlashStartAdd);
    uint32_t appEntry = *((volatile uint32_t*) (fwFlashStartAdd + 4));

    SCB_DisableICache();
    SCB_DisableDCache();

    __enable_irq();

    __DMB();
    SCB->VTOR = fwFlashStartAdd;
    __DSB();

    HAL_DeInit();
    __set_MSP(appStack);

    pFunction jumpToApplication = (pFunction)appEntry;
    jumpToApplication();
}

/**
 * @brief Finds the package file on the filesystem.
 *        It looks for files matching "cis_package_*.bin"
 * @param packageFilePath Pointer to buffer where the path will be stored
 * @param maxLen Maximum length of the buffer
 * @retval true if a package file is found, false otherwise
 */
static bool findPackageFile(char *packageFilePath, size_t maxLen)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    char *fn;

    res = f_opendir(&dir, FW_PATH); // Open the directory
    if (res == FR_OK)
    {
        for (;;)
        {
            res = f_readdir(&dir, &fno); // Read a directory item
            if ((res != FR_OK) || (fno.fname[0] == 0))
            {
                break; // End of directory or error
            }

            // Check if it's a file, not a directory
            if (fno.fattrib & AM_DIR)
            {
                continue;
            }
            else
            {
                fn = fno.fname;
                // Look for filenames that start with "cis_package_" and contain ".bin"
                if ((strstr(fn, "cis_package_") == fn) && strstr(fn, ".bin"))
                {
                    // Insert a slash between FW_PATH and fn
                    snprintf(packageFilePath, maxLen, "%s/%s", FW_PATH, fn);
                    f_closedir(&dir);
                    return true;
                }
            }
        }
        f_closedir(&dir);
    }
    return false;
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
  disableCM4Boot();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */

/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

	/* Initialize the Flash Update State */
	FW_UpdateState dataRead;
	STM32Flash_readPersistentData(&dataRead);

	if (dataRead == FW_UPDATE_NONE)
	{
		gotoFirmware(FW_CM7_START_ADDR);
	}

	if (dataRead == FW_UPDATE_TO_TEST)
	{
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

	printf("\n------- START BOOTLOADER -------\n");

	gui_init();

	if (dataRead == FW_UPDATE_DONE)
	{
		/* Reboot after we close the connection. */
		STM32Flash_StatusTypeDef status = STM32Flash_writePersistentData(FW_UPDATE_NONE);
		if (status == STM32FLASH_OK)
		{
			printf("Firmware update done, reset firmware update flag\n");
		}
		else
		{
			printf("Failed to write firmware update status in STM32 flash\n");
		}

		gui_displayUpdateSuccess();

		printf("Rebooting in 2\n");
		/* Wait 2 seconds. */
		HAL_Delay(2000);
		NVIC_SystemReset();
	}

	printf("----- FILE INITIALIZATION ------\n");

	FRESULT fres; // Variable to store the result of FATFS operations

	// Attempt to mount the file system on SD card or USB (where the package is stored)
	fres = f_mount(&fs, "0:", 1);
	if (fres != FR_OK)
	{
	    printf("FS mount ERROR\n");
	    gui_displayUpdateFailed();
	}
	else
	{
		printf("FS mount SUCCESS\n");

		char packageFilePath[64];

		// Find the package file
		if (findPackageFile(packageFilePath, sizeof(packageFilePath)))
		{
			printf("Found package file: %s\n", packageFilePath);

			printf("--------- START UPDATE ---------\n");

			if (update_processPackageFile(packageFilePath))
			{
				printf("Firmware update completed successfully\n");
			}
			else
			{
				printf("Firmware update failed\n");
				gui_displayUpdateFailed();
			}
		}
		else
		{
			printf("No firmware found in /firmware/\n");
			gui_displayUpdateFailed();
		}
	}

	/* Prepare to reboot the system */

	printf("Preparing to reset all cores \n");

	/* Reboot after we close the connection. */
	STM32Flash_StatusTypeDef status = STM32Flash_writePersistentData(FW_UPDATE_TO_TEST);
	if (status == STM32FLASH_OK)
	{
		printf("Firmware update must be tested now \n");
	}
	else
	{
		printf("Failed to write firmware update status in STM32 flash\n");
	}

	gui_displayUpdateWrited();

	HAL_SRAM_MspDeInit(&hsram1);
	HAL_UART_MspDeInit(&huart1);
	HAL_RNG_MspDeInit(&hrng);
	HAL_CRC_MspDeInit(&hcrc);
	HAL_QSPI_MspDeInit(&hqspi);

	printf("Rebooting in 2\n");
	/* Wait 2 seconds. */
	HAL_Delay(2000);
	MX_IWDG1_Init();
	NVIC_SystemReset();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
