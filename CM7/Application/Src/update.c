/**
 ******************************************************************************
 * @file           : update.c
 * @brief          : Implementation of firmware update functionality.
 ******************************************************************************
 * @attention
 *
 * Copyright (C) 2018-present Reso-nance.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "progress.h"
#include "main.h"
#include "config.h"
#include "basetypes.h"
#include "globals.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "fatfs.h"
#include "stm32_flash.h"

#include "crc.h"

#include "update_gui.h"
#include "update.h"

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE      512
#define HEADER_SIZE      24
#define VERSION_STR_SIZE 9

/* Step numbers for progress tracking */
#define NUM_STEPS             8
#define STEP_CRC_CALCULATION  1
#define STEP_BACKUP_CM7       2
#define STEP_BACKUP_CM4       3
#define STEP_ERASE_CM7        4
#define STEP_ERASE_CM4        5
#define STEP_FLASH_CM7        6
#define STEP_FLASH_CM4        7
#define STEP_SAVE_EXTERNAL    8

/* Private variables ---------------------------------------------------------*/
uint8_t tempBuffer[32] __attribute__((aligned(32)));

/* Function prototypes -------------------------------------------------------*/
static uint32_t update_read_uint32_le(const uint8_t *buffer);
static bool update_calculateCRC(FIL* file, ProgressManager* progressManager);
static bool update_backupFirmware(uint32_t flashStartAddr, uint32_t size, const char* backupFilePath, ProgressManager* progressManager, int step_number);
static bool update_eraseFirmware(uint32_t flashStartAddr, uint32_t size, ProgressManager* progressManager, int step_number);
static bool update_writeFirmware(uint32_t flashStartAddr, FIL* file, uint32_t size, ProgressManager* progressManager, int step_number);
static bool update_writeExternalData(FIL* file, uint32_t external_size, ProgressManager* progressManager, int step_number);

/**
 * @brief Reads a little-endian uint32 from a byte buffer.
 */
static uint32_t update_read_uint32_le(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0]) |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
}

/**
 * @brief Processes a firmware update package file.
 */
bool update_processPackageFile(const TCHAR* packageFilePath)
{
    FIL file;
    UINT bytesRead;
    FRESULT res;
    uint8_t header[HEADER_SIZE];
    uint32_t cm7_size, cm4_size, external_size;
    char version[VERSION_STR_SIZE] = {0};
    uint8_t magic[4];
    ProgressManager progressManager;

    // Initialize progress manager
    progress_init(&progressManager, NUM_STEPS);

    // Open the update package file
    res = f_open(&file, packageFilePath, FA_READ);
    if (res != FR_OK)
    {
        printf("Error: Failed to open the package file: %s\n", packageFilePath);
        gui_displayUpdateFailed();
        return false;
    }

    // Read the header
    res = f_read(&file, header, sizeof(header), &bytesRead);
    if (res != FR_OK || bytesRead != sizeof(header))
    {
        printf("Error: Failed to read the package header\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Parse the header
    memcpy(magic, header, 4);
    if (memcmp(magic, "BOOT", 4) != 0)
    {
        printf("Error: Invalid package magic number\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    cm7_size = update_read_uint32_le(header + 4);
    cm4_size = update_read_uint32_le(header + 8);
    external_size = update_read_uint32_le(header + 12);
    memcpy(version, header + 16, 8);

    printf("Package version: %s\n", version);
    printf("CM7 firmware size: %lu bytes\n", cm7_size);
    printf("CM4 firmware size: %lu bytes\n", cm4_size);
    printf("External data size: %lu bytes\n", external_size);

    // Display version
    gui_displayVersion(version);

    // Step 1: Calculate and verify CRC
    printf("Step 1: Calculate and verify CRC\n");
    if (!update_calculateCRC(&file, &progressManager))
    {
        printf("Error: Failed to calculate or verify CRC\n");
        f_close(&file);
        return false;
    }

    // Step 2: Backup current CM7 firmware
    printf("Step 2: Backup current CM7 firmware\n");
    if (!update_backupFirmware(FW_CM7_START_ADDR, cm7_size, "backup_cm7.bin", &progressManager, STEP_BACKUP_CM7))
    {
        printf("Error: Failed to backup current CM7 firmware\n");
        f_close(&file);
        return false;
    }

    // Step 3: Backup current CM4 firmware
    printf("Step 3: Backup current CM4 firmware\n");
    if (!update_backupFirmware(FW_CM4_START_ADDR, cm4_size, "backup_cm4.bin", &progressManager, STEP_BACKUP_CM4))
    {
        printf("Error: Failed to backup current CM4 firmware\n");
        f_close(&file);
        return false;
    }

    // Step 4: Erase CM7 firmware
    printf("Step 4: Erase CM7 firmware\n");
    if (!update_eraseFirmware(FW_CM7_START_ADDR, cm7_size, &progressManager, STEP_ERASE_CM7))
    {
        printf("Error: Failed to erase CM7 firmware\n");
        f_close(&file);
        return false;
    }

    // Step 5: Erase CM4 firmware
    printf("Step 5: Erase CM4 firmware\n");
    if (!update_eraseFirmware(FW_CM4_START_ADDR, cm4_size, &progressManager, STEP_ERASE_CM4))
    {
        printf("Error: Failed to erase CM4 firmware\n");
        f_close(&file);
        return false;
    }

    // Step 6: Flash new CM7 firmware
    printf("Step 6: Flash new CM7 firmware\n");
    res = f_lseek(&file, HEADER_SIZE);
    if (res != FR_OK)
    {
        printf("Error: Failed to reposition after the header\n");
        gui_displayUpdateFailed();
        f_close(&file);
        return false;
    }

    if (!update_writeFirmware(FW_CM7_START_ADDR, &file, cm7_size, &progressManager, STEP_FLASH_CM7))
    {
        printf("Error: Failed to flash new CM7 firmware\n");
        f_close(&file);
        return false;
    }

    // Step 7: Flash new CM4 firmware
    printf("Step 7: Flash new CM4 firmware\n");
    res = f_lseek(&file, HEADER_SIZE + cm7_size);
    if (res != FR_OK)
    {
        printf("Error: Failed to reposition to CM4 firmware data\n");
        gui_displayUpdateFailed();
        f_close(&file);
        return false;
    }

    if (!update_writeFirmware(FW_CM4_START_ADDR, &file, cm4_size, &progressManager, STEP_FLASH_CM4))
    {
        printf("Error: Failed to flash new CM4 firmware\n");
        f_close(&file);
        return false;
    }

    // Step 8: Save external data
    printf("Step 8: Save external data\n");
    res = f_lseek(&file, HEADER_SIZE + cm7_size + cm4_size);
    if (res != FR_OK)
    {
        printf("Error: Failed to reposition to external data\n");
        gui_displayUpdateFailed();
        f_close(&file);
        return false;
    }

    if (!update_writeExternalData(&file, external_size, &progressManager, STEP_SAVE_EXTERNAL))
    {
        printf("Error: Failed to save external data\n");
        f_close(&file);
        return false;
    }

    // Close the package file
    f_close(&file);

    // Display success message
    gui_displayUpdateSuccess();

    return true;
}

/**
 * @brief Calculates and verifies the CRC of the update package.
 * @param file Pointer to the open file.
 * @param progressManager Pointer to the progress manager.
 * @return True if the CRC matches, false otherwise.
 */
static bool update_calculateCRC(FIL* file, ProgressManager* progressManager)
{
    FSIZE_t file_size = f_size(file);
    FRESULT res;
    UINT bytesRead;
    uint32_t crc_position = file_size - 4;
    uint32_t crc_read;
    uint32_t crc_calculated = 0;
    uint32_t totalDataRead = 0;
    uint32_t crc_length = file_size - 4; // Exclude the footer CRC
    uint8_t readBuffer[BUFFER_SIZE] __attribute__((aligned(4))); // Buffer aligned to 4 bytes
    uint8_t crc_buffer[4];

    // Read the CRC from the footer
    res = f_lseek(file, crc_position);
    if (res != FR_OK)
    {
        printf("Failed to reposition to read the CRC\n");
        gui_displayUpdateFailed();
        return false;
    }

    res = f_read(file, crc_buffer, 4, &bytesRead);
    if (res != FR_OK || bytesRead != 4)
    {
        printf("Failed to read package CRC\n");
        gui_displayUpdateFailed();
        return false;
    }

    // Convert the read CRC (little-endian)
    crc_read = update_read_uint32_le(crc_buffer);
    printf("Read CRC from footer: 0x%08lX\n", crc_read);

    // Return to the beginning of the file for CRC calculation
    res = f_lseek(file, 0);
    if (res != FR_OK)
    {
        printf("Failed to reposition to the beginning of the file for CRC calculation\n");
        gui_displayUpdateFailed();
        return false;
    }

    // Initialize the CRC calculation
    __HAL_CRC_DR_RESET(&hcrc);

    while (totalDataRead < crc_length)
    {
        uint32_t bytesToRead = (crc_length - totalDataRead > BUFFER_SIZE) ? BUFFER_SIZE : (crc_length - totalDataRead);
        res = f_read(file, readBuffer, bytesToRead, &bytesRead);
        if (res != FR_OK || bytesRead == 0)
        {
            printf("Error reading the file for CRC calculation\n");
            gui_displayUpdateFailed();
            return false;
        }

        // Calculate the CRC for the read bytes
        crc_calculated = HAL_CRC_Accumulate(&hcrc, (uint32_t *)readBuffer, bytesRead);

        totalDataRead += bytesRead;

        // Update step progress
        progress_update(progressManager, STEP_CRC_CALCULATION, totalDataRead, crc_length);
    }

    // Perform the final XOR with 0xFFFFFFFF
    crc_calculated ^= 0xFFFFFFFF;

    // Compare the calculated CRC with the CRC from the file
    if (crc_calculated != crc_read)
    {
        printf("CRC mismatch: calculated 0x%08lX, expected 0x%08lX\n", crc_calculated, crc_read);
        gui_displayUpdateFailed();
        return false;
    }
    else
    {
        printf("CRC verified successfully\n");
    }

    return true;
}

/**
 * @brief Backs up firmware from flash memory to a specified file.
 *
 * This function reads firmware data from flash memory and writes it
 * to a backup file. Progress updates are reported during the operation.
 *
 * @param flashStartAddr Starting address in flash memory.
 * @param size Size of the firmware to backup in bytes.
 * @param backupFilePath Path to the backup file to create.
 * @param progressManager Pointer to the progress manager for updates.
 * @param step_number Step number for the progress manager.
 * @return true if the backup is successful, false otherwise.
 */
static bool update_backupFirmware(uint32_t flashStartAddr, uint32_t size, const char* backupFilePath, ProgressManager* progressManager, int step_number)
{
    FIL backupFile;
    FRESULT res;
    UINT bytesWritten;
    uint8_t readBuffer[32] __attribute__((aligned(32))); // Required alignment
    uint32_t bytesRemaining = size;
    uint32_t flashAddress = flashStartAddr;

    // Check flash memory boundaries
    if (flashAddress + size > FLASH_END_ADDR)
    {
        printf("Error: Flash address out of range\n");
        return false;
    }

    // Open the backup file
    res = f_open(&backupFile, backupFilePath, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("Error: Cannot open backup file %s\n", backupFilePath);
        return false;
    }

    uint32_t totalBytesRead = 0;

    while (bytesRemaining > 0)
    {
        uint32_t chunkSize = (bytesRemaining > sizeof(readBuffer)) ? sizeof(readBuffer) : bytesRemaining;

        // Read from flash memory
        memcpy(readBuffer, (uint8_t*)flashAddress, chunkSize);

        // Write to the file
        res = f_write(&backupFile, readBuffer, chunkSize, &bytesWritten);
        if (res != FR_OK || bytesWritten != chunkSize)
        {
            printf("Error: Write failed in backup file %s\n", backupFilePath);
            f_close(&backupFile);
            return false;
        }

        flashAddress += chunkSize;
        bytesRemaining -= chunkSize;
        totalBytesRead += chunkSize;

        // Update progress
        progress_update(progressManager, step_number, totalBytesRead, size);
    }

    f_close(&backupFile);
    return true;
}

/**
 * @brief Writes firmware to flash memory from a specified file.
 *
 * This function reads firmware data from a file and writes it to flash memory
 * in aligned 32-byte blocks. Progress updates are reported during the operation.
 *
 * @param flashStartAddr Starting address in flash memory.
 * @param file Pointer to the file containing the firmware.
 * @param size Size of the firmware to write in bytes.
 * @param progressManager Pointer to the progress manager for updates.
 * @param step_number Step number for the progress manager.
 * @return true if the write operation is successful, false otherwise.
 */
static bool update_writeFirmware(uint32_t flashStartAddr, FIL* file, uint32_t size, ProgressManager* progressManager, int step_number)
{
    uint8_t readBuffer[512] __attribute__((aligned(32)));
    uint8_t writeBlock[32] __attribute__((aligned(32)));
    uint32_t flashAddress = flashStartAddr;
    uint32_t bytesRemaining = size;
    uint32_t totalBytesWritten = 0;
    UINT bytesRead;
    FRESULT res;
    STM32Flash_StatusTypeDef status;

    printf("Flashing firmware to address 0x%08lx...\n", flashAddress);

    // Check initial alignment of flash address
    if ((flashAddress % 32) != 0)
    {
        printf("Error: Flash address misaligned at 0x%08lx\n", flashAddress);
        gui_displayUpdateFailed();
        return false;
    }

    while (bytesRemaining > 0)
    {
        uint32_t chunkSize = (bytesRemaining > sizeof(readBuffer)) ? sizeof(readBuffer) : bytesRemaining;

        // Read data from file
        res = f_read(file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize)
        {
            printf("Error: Failed to read firmware data\n");
            gui_displayUpdateFailed();
            return false;
        }

        bytesRemaining -= bytesRead;

        // Process this chunk in 32-byte blocks
        uint32_t offset = 0;
        while (offset < bytesRead)
        {
            uint32_t blockSize = ((bytesRead - offset) >= 32) ? 32 : (bytesRead - offset);
            uint8_t* dataPtr = readBuffer + offset;

            // If block is not a full 32 bytes, pad with 0xFF
            if (blockSize < 32)
            {
                memset(writeBlock, 0xFF, 32);
                memcpy(writeBlock, dataPtr, blockSize);
                dataPtr = writeBlock;
                blockSize = 32;
            }

            // Write to flash
            status = STM32Flash_write32B(dataPtr, flashAddress);

            if (status != STM32FLASH_OK)
            {
                printf("Error: Failed to write to flash at 0x%08lx\n", flashAddress);
                gui_displayUpdateFailed();
                return false;
            }

            flashAddress += 32;
            offset += blockSize;
            totalBytesWritten += blockSize;

            uint32_t displayedBytes = (totalBytesWritten > size) ? size : totalBytesWritten;
            progress_update(progressManager, step_number, displayedBytes, size);
        }
    }

    return true;
}
/**
 * @brief Erases necessary flash sectors for firmware.
 * @param flashStartAddr Starting address of the firmware in flash.
 * @param size Size of the firmware.
 * @param progressManager Pointer to the progress manager.
 * @param step_number The step number for progress tracking.
 * @return True if the sectors were erased successfully, false otherwise.
 */
static bool update_eraseFirmware(uint32_t flashStartAddr, uint32_t size, ProgressManager* progressManager, int step_number)
{
    STM32Flash_StatusTypeDef status;

    // Determine flash bank and sector
    uint32_t flashBank = (flashStartAddr >= ADDR_FLASH_SECTOR_0_BANK2) ? FLASH_BANK_2 : FLASH_BANK_1;
    uint32_t flashSector = stm32_flashGetSector(flashStartAddr);
    uint32_t NbSectors = (size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
    uint32_t sectorsErased = 0;

    printf("Erasing flash sectors starting from sector %lu...\n", flashSector);
    for (uint32_t sector = flashSector; sector < flashSector + NbSectors; sector++)
    {
        status = STM32Flash_erase_sector(flashBank, sector);
        if (status != STM32FLASH_OK)
        {
            printf("Failed to erase sector %lu\n", sector);
            gui_displayUpdateFailed();
            return false;
        }

        sectorsErased++;

        // Update step progress
        progress_update(progressManager, step_number, sectorsErased, NbSectors);
    }

    return true;
}

/**
 * @brief Writes the external data to the file system.
 * @param file Pointer to the open package file.
 * @param external_size Size of the external data.
 * @param progressManager Pointer to the progress manager.
 * @param step_number The step number for progress tracking.
 * @return True if the external data was written successfully, false otherwise.
 */
static bool update_writeExternalData(FIL* file, uint32_t external_size, ProgressManager* progressManager, int step_number)
{
    UINT bytesRead;
    FRESULT res;
    uint8_t readBuffer[BUFFER_SIZE] __attribute__((aligned(4)));

    uint32_t totalBytesToWrite = external_size;
    uint32_t totalBytesWritten = 0;

    // Write external data to the file system
    printf("Writing external data to the file system...\n");

    // Open the file for writing
    FIL externalFile;
    res = f_open(&externalFile, "0:/External_MAX8.tar.gz", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("Failed to open the file on the file system\n");
        gui_displayUpdateFailed();
        return false;
    }

    uint32_t bytesToWrite = external_size;

    while (bytesToWrite > 0)
    {
        uint32_t chunkSize = (bytesToWrite > BUFFER_SIZE) ? BUFFER_SIZE : bytesToWrite;
        res = f_read(file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize)
        {
            printf("Failed to read external data\n");
            f_close(&externalFile);
            gui_displayUpdateFailed();
            return false;
        }

        // Write the data to the file
        UINT bytesWritten;
        res = f_write(&externalFile, readBuffer, bytesRead, &bytesWritten);
        if (res != FR_OK || bytesWritten != bytesRead)
        {
            printf("Failed to write external data to the file system\n");
            f_close(&externalFile);
            gui_displayUpdateFailed();
            return false;
        }

        bytesToWrite -= bytesRead;
        totalBytesWritten += bytesRead;

        // Update step progress
        progress_update(progressManager, step_number, totalBytesWritten, totalBytesToWrite);
    }

    // Close the external file
    f_close(&externalFile);

    return true;
}
