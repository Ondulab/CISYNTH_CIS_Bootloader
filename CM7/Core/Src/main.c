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
#include "globals.h"
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

#define BUFFER_SIZE 512

#define FW_CM7_START_ADDR 0x08040000
#define FW_CM4_START_ADDR 0x08100000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FATFS fs;   // Filesystem object

typedef void (*pFunction)(void);
uint8_t tempBuffer[32] __attribute__((aligned(32)));

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* Function prototypes */
static void gui_displayVersion(const char* version);
static bool processPackageFile(const TCHAR* packageFilePath);
static bool findPackageFile(char* packageFilePath, size_t maxLen);

static void gui_displayUpdateStarted(void);
static void gui_displayUpdateFailed(void);
static void gui_displayUpdateWrited(void);
static void gui_displayUpdateSuccess(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void gui_displayUpdateStarted(void)
{
    ssd1362_clearBuffer();

    // Draw border
    ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, true);
    ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

    ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

    ssd1362_progressBar(26, 25, 0, 0xF);

    ssd1362_drawString(0, 45,                    (int8_t *)"        DO NOT POWER OFF      ", 0xF, 8);

    // Display the frame buffer
    ssd1362_writeFullBuffer();
}

static void gui_displayUpdateFailed(void)
{
    ssd1362_clearBuffer();

    // Draw border
    ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, false);
    ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

    ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

    ssd1362_drawString(0, 25,                    (int8_t *)"         UPDATE FAILED        ", 0xF, 8);

    ssd1362_drawString(0, 45,                    (int8_t *)"        DO NOT POWER OFF      ", 0xF, 8);

    // Display the frame buffer
    ssd1362_writeFullBuffer();
}

static void gui_displayUpdateWrited(void)
{
    ssd1362_clearBuffer();

    // Draw border
    ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, false);
    ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

    ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

    ssd1362_drawString(0, 25,                    (int8_t *)"       FIRMWARE  TESTING      ", 0xF, 8);

    ssd1362_drawString(0, 45,                    (int8_t *)"             REBOOT           ", 0xF, 8);

    // Display the frame buffer
    ssd1362_writeFullBuffer();
}

static void gui_displayUpdateSuccess(void)
{
    ssd1362_clearBuffer();

    // Draw border
    ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, true);
    ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

    ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

    ssd1362_drawString(0, 25,                    (int8_t *)"   FIRMWARE TESTING SUCCESS   ", 0xF, 8);

    ssd1362_drawString(0, 45,                    (int8_t *)"             REBOOT           ", 0xF, 8);

    // Display the frame buffer
    ssd1362_writeFullBuffer();
}

static void gui_displayVersion(const char* version)
{
    ssd1362_clearBuffer();

    // Draw border
    ssd1362_fillRect(0, DISPLAY_HEAD_Y1POS, DISPLAY_WIDTH, DISPLAY_HEAD_Y2POS, BANNER_BACKGROUND_COLOR, true);
    ssd1362_drawRect(0, 0, 255, 63, BANNER_BACKGROUND_COLOR, false);

    char versionString[32];
    snprintf(versionString, sizeof(versionString), "   Updating to version %s   ", version);

    ssd1362_drawString(0, DISPLAY_HEAD_Y1POS + 1, (int8_t *)"        FIRMWARE UPDATE       ", 0xF, 8);

    ssd1362_drawString(0, 45,                    (int8_t *)versionString, 0xF, 8);

    // Display the frame buffer
    ssd1362_writeFullBuffer();
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

    SysTick->CTRL = 0;  // Enabled in application
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    HAL_RCC_DeInit();

    for(uint8_t i = 0; i < 8; i++) // Clear all NVIC Enable and Pending registers
    {
        NVIC->ICER[i]=0xFFFFFFFF;
        NVIC->ICPR[i]=0xFFFFFFFF;
    }

    __enable_irq();

    pFunction appEntry;
    uint32_t appStack;

    appStack = (uint32_t) *((volatile uint32_t*) fwFlashStartAdd);
    appEntry = (pFunction) *((volatile uint32_t*) (fwFlashStartAdd + 4));
    __DMB();
    SCB->VTOR = fwFlashStartAdd;
    __DSB();
    SysTick->CTRL = 0x0;
    HAL_DeInit();
    __set_MSP(appStack);
    appEntry();
}

/**
 * @brief Finds the package file on the filesystem.
 *        It looks for files matching "cis_package_*.bin"
 * @param packageFilePath Pointer to buffer where the path will be stored
 * @param maxLen Maximum length of the buffer
 * @retval true if a package file is found, false otherwise
 */
static bool findPackageFile(char* packageFilePath, size_t maxLen)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    char *fn;

    res = f_opendir(&dir, "0:/");                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;   /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {
                continue;                               /* It is a directory */
            } else {
                fn = fno.fname;
                if (strstr(fn, "cis_package_") == fn && strstr(fn, ".bin")) {
                    snprintf(packageFilePath, maxLen, "0:/%s", fn);
                    f_closedir(&dir);
                    return true;
                }
            }
        }
        f_closedir(&dir);
    }
    return false;
}

static uint32_t read_uint32_le(const uint8_t *buffer) {
    return ((uint32_t)buffer[0]) |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
}

static bool processPackageFile(const TCHAR* packageFilePath)
{
    FIL file;
    UINT bytesRead;
    FRESULT res;
    uint32_t totalBytesToWrite = 0, totalBytesWritten = 0;
    uint32_t crc_read;
    uint8_t header[24]; // 4 (magic) + 4 (cm7_size) + 4 (cm4_size) + 4 (external_size) + 8 (version)
    uint32_t cm7_size, cm4_size, external_size;
    char version[9] = {0}; // Version string (8 bytes + null terminator)
    uint8_t magic[4];
    uint32_t flashAddress;
    STM32Flash_StatusTypeDef status;

    // Open the package file
    res = f_open(&file, packageFilePath, FA_READ);
    if (res != FR_OK) {
        printf("Failed to open package file: %s\n", packageFilePath);
        gui_displayUpdateFailed();
        return false;
    }

    // Read the 24-byte header
    res = f_read(&file, header, sizeof(header), &bytesRead);
    if (res != FR_OK || bytesRead != sizeof(header)) {
        printf("Failed to read package header\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Parse the header
    memcpy(magic, header, 4);
    if (memcmp(magic, "BOOT", 4) != 0) {
        printf("Invalid package magic number\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Read values considering endianness (little-endian)
    cm7_size = read_uint32_le(header + 4);
    cm4_size = read_uint32_le(header + 8);
    external_size = read_uint32_le(header + 12);
    memcpy(version, header + 16, 8); // The version is 8 bytes

    printf("Package version: %s\n", version);
    printf("CM7 firmware size: %lu bytes\n", cm7_size);
    printf("CM4 firmware size: %lu bytes\n", cm4_size);
    printf("External data size: %lu bytes\n", external_size);

    // Calculate the total size of data for the progress bar
    totalBytesToWrite = cm7_size + cm4_size + external_size;

    // Display the version on the screen
    gui_displayVersion(version);

    // Get the total file size
    FSIZE_t file_size = f_size(&file);

    // Calculate the position of the CRC (last 4 bytes of the file)
    uint32_t crc_position = file_size - 4;

    // Read the CRC from the footer
    res = f_lseek(&file, crc_position);
    if (res != FR_OK) {
        printf("Failed to reposition to read the CRC\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    uint8_t crc_buffer[4];
    res = f_read(&file, crc_buffer, 4, &bytesRead);
    if (res != FR_OK || bytesRead != 4) {
        printf("Failed to read package CRC\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Convert the read CRC (little-endian)
    crc_read = read_uint32_le(crc_buffer);
    printf("Read CRC from footer: 0x%08lX\n", crc_read);

    // Return to the beginning of the file for CRC calculation
    res = f_lseek(&file, 0);
    if (res != FR_OK) {
        printf("Failed to reposition to the beginning of the file for CRC calculation\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Initialize the CRC calculation
    __HAL_CRC_DR_RESET(&hcrc);

    // Variables
    uint32_t crc_calculated = 0;
    uint32_t totalDataRead = 0;
    uint32_t crc_length = file_size - 4; // Exclude the footer CRC
    uint8_t readBuffer[BUFFER_SIZE] __attribute__((aligned(4))); // Buffer aligned to 4 bytes

    while (totalDataRead < crc_length) {
        uint32_t bytesToRead = (crc_length - totalDataRead > BUFFER_SIZE) ? BUFFER_SIZE : (crc_length - totalDataRead);
        res = f_read(&file, readBuffer, bytesToRead, &bytesRead);
        if (res != FR_OK || bytesRead == 0) {
            // Handle the error
            printf("Error reading the file for CRC calculation\n");
            f_close(&file);
            gui_displayUpdateFailed();
            return false;
        }

        // Calculate the CRC for the read bytes
        crc_calculated = HAL_CRC_Accumulate(&hcrc, (uint32_t *)readBuffer, bytesRead);

        totalDataRead += bytesRead;
    }

    // Perform the final XOR with 0xFFFFFFFF
    crc_calculated ^= 0xFFFFFFFF;

    // Compare the calculated CRC with the CRC from the file
    if (crc_calculated != crc_read) {
        printf("CRC mismatch: calculated 0x%08lX, expected 0x%08lX\n", crc_calculated, crc_read);
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    } else {
        printf("CRC verified successfully\n");
    }

    // Reposition the file pointer after the header to start reading data
    res = f_lseek(&file, sizeof(header));
    if (res != FR_OK) {
        printf("Failed to reposition after the header\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Update the progress bar to 0%
    ssd1362_progressBar(26, 25, 0, 0xF);

    // Erase the necessary flash sectors for the CM7 and CM4 firmwares

    // Erase CM7 sectors
    printf("Erasing CM7 flash sectors...\n");
    uint32_t cm7_flashBank = FLASH_BANK_1;
    uint32_t cm7_flashSector = stm32_flashGetSector(FW_CM7_START_ADDR);
    uint32_t cm7_NbSectors = (cm7_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE; // Calculate the number of required sectors

    for (uint32_t sector = cm7_flashSector; sector < cm7_flashSector + cm7_NbSectors; sector++) {
        status = STM32Flash_erase_sector(cm7_flashBank, sector);
        if (status != STM32FLASH_OK) {
            printf("Failed to erase CM7 sector %lu\n", sector);
            f_close(&file);
            gui_displayUpdateFailed();
            return false;
        }
    }

    // Update the progress bar
    ssd1362_progressBar(26, 25, 5, 0xF); // Example progress

    // Erase CM4 sectors
    printf("Erasing CM4 flash sectors...\n");
    uint32_t cm4_flashBank = FLASH_BANK_2;
    uint32_t cm4_flashSector = stm32_flashGetSector(FW_CM4_START_ADDR);
    uint32_t cm4_NbSectors = (cm4_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE; // Calculate the number of required sectors

    for (uint32_t sector = cm4_flashSector; sector < cm4_flashSector + cm4_NbSectors; sector++) {
        status = STM32Flash_erase_sector(cm4_flashBank, sector);
        if (status != STM32FLASH_OK) {
            printf("Failed to erase CM4 sector %lu\n", sector);
            f_close(&file);
            gui_displayUpdateFailed();
            return false;
        }
    }

    // Update the progress bar
    ssd1362_progressBar(26, 25, 10, 0xF); // Example progress

    // Write the CM7 firmware
    printf("Writing CM7 firmware...\n");
    flashAddress = FW_CM7_START_ADDR;
    totalBytesWritten = 0;
    uint32_t bytesToWrite = cm7_size;

    while (bytesToWrite > 0) {
        uint32_t chunkSize = (bytesToWrite > BUFFER_SIZE) ? BUFFER_SIZE : bytesToWrite;
        res = f_read(&file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize) {
            printf("Failed to read CM7 firmware data\n");
            f_close(&file);
            gui_displayUpdateFailed();
            return false;
        }

        // Write data to flash using STM32Flash_write32B
        uint32_t writeOffset = 0;
        while (writeOffset < bytesRead) {
            uint32_t writeSize = ((bytesRead - writeOffset) >= 32) ? 32 : (bytesRead - writeOffset);
            uint8_t* dataPtr = readBuffer + writeOffset;

            // Ensure that the data is aligned to 32 bytes
            if (((uint32_t)dataPtr % 32 != 0) || (writeSize < 32)) {
                memset(tempBuffer, 0xFF, 32);
                memcpy(tempBuffer, dataPtr, writeSize);
                dataPtr = tempBuffer;
            }

            // Ensure that the flash address is aligned
            if (flashAddress % 32 != 0) {
                printf("Unaligned flash address: 0x%08lx\n", flashAddress);
                f_close(&file);
                gui_displayUpdateFailed();
                return false;
            }

            status = STM32Flash_write32B(dataPtr, flashAddress);
            if (status != STM32FLASH_OK) {
                printf("Failed to write to flash at address 0x%08lx\n", flashAddress);
                f_close(&file);
                gui_displayUpdateFailed();
                return false;
            }

            flashAddress += 32;
            writeOffset += writeSize;
            totalBytesWritten += writeSize;

            // Update the progress bar
            uint8_t progress = (uint8_t)((totalBytesWritten * 100) / totalBytesToWrite);
            ssd1362_progressBar(26, 25, progress, 0xF);
        }

        bytesToWrite -= bytesRead;
    }

    // Write the CM4 firmware
    printf("Writing CM4 firmware...\n");
    flashAddress = FW_CM4_START_ADDR;
    bytesToWrite = cm4_size;

    while (bytesToWrite > 0) {
        uint32_t chunkSize = (bytesToWrite > BUFFER_SIZE) ? BUFFER_SIZE : bytesToWrite;
        res = f_read(&file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize) {
            printf("Failed to read CM4 firmware data\n");
            f_close(&file);
            gui_displayUpdateFailed();
            return false;
        }

        // Write data to flash using STM32Flash_write32B
        uint32_t writeOffset = 0;
        while (writeOffset < bytesRead) {
            uint32_t writeSize = ((bytesRead - writeOffset) >= 32) ? 32 : (bytesRead - writeOffset);
            uint8_t* dataPtr = readBuffer + writeOffset;

            // Ensure that the data is aligned to 32 bytes
            if (((uint32_t)dataPtr % 32 != 0) || (writeSize < 32)) {
                memset(tempBuffer, 0xFF, 32);
                memcpy(tempBuffer, dataPtr, writeSize);
                dataPtr = tempBuffer;
            }

            // Ensure that the flash address is aligned
            if (flashAddress % 32 != 0) {
                printf("Unaligned flash address: 0x%08lx\n", flashAddress);
                f_close(&file);
                gui_displayUpdateFailed();
                return false;
            }

            status = STM32Flash_write32B(dataPtr, flashAddress);
            if (status != STM32FLASH_OK) {
                printf("Failed to write to flash at address 0x%08lx\n", flashAddress);
                f_close(&file);
                gui_displayUpdateFailed();
                return false;
            }

            flashAddress += 32;
            writeOffset += writeSize;
            totalBytesWritten += writeSize;

            // Update the progress bar
            uint8_t progress = (uint8_t)((totalBytesWritten * 100) / totalBytesToWrite);
            ssd1362_progressBar(26, 25, progress, 0xF);
        }

        bytesToWrite -= bytesRead;
    }

    // Write external data to the same file system
    printf("Writing external data to the file system...\n");

    // Open the file for writing
    FIL externalFile;
    res = f_open(&externalFile, "0:/External_MAX8.tar.gz", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("Failed to open the file on the file system\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    bytesToWrite = external_size;

    while (bytesToWrite > 0) {
        uint32_t chunkSize = (bytesToWrite > BUFFER_SIZE) ? BUFFER_SIZE : bytesToWrite;
        res = f_read(&file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize) {
            printf("Failed to read external data\n");
            f_close(&file);
            f_close(&externalFile);
            gui_displayUpdateFailed();
            return false;
        }

        // Write the data to the file
        UINT bytesWritten;
        res = f_write(&externalFile, readBuffer, bytesRead, &bytesWritten);
        if (res != FR_OK || bytesWritten != bytesRead) {
            printf("Failed to write external data to the file system\n");
            f_close(&file);
            f_close(&externalFile);
            gui_displayUpdateFailed();
            return false;
        }

        bytesToWrite -= bytesRead;
        totalBytesWritten += bytesRead;

        // Update the progress bar
        uint8_t progress = (uint8_t)((totalBytesWritten * 100) / totalBytesToWrite);
        ssd1362_progressBar(26, 25, progress, 0xF);
    }

    // Close the external file
    f_close(&externalFile);

    // Close the package file
    f_close(&file);

    // Update the progress bar to 100%
    ssd1362_progressBar(26, 25, 100, 0xF);

    // Display the success message
    gui_displayUpdateSuccess();

    // Return success
    return true;
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

    printf("START BOOTLOADER\n");

    ssd1362_init();
    ssd1362_clearBuffer();
    ssd1362_writeFullBuffer();

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

    gui_displayUpdateStarted();

    printf("- FILE INITIALIZATION -\n");

    FRESULT fres; // Variable to store the result of FATFS operations

    // Attempt to mount the file system on SD card or USB (where the package is stored)
    fres = f_mount(&fs, "0:", 1); // '0:' represents the logical drive number for the SD card or USB
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

            // UPDATE FROM PACKAGE FILE
            if (processPackageFile(packageFilePath))
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
            printf("No package file found\n");
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
