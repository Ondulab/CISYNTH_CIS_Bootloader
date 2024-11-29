/**
 ******************************************************************************
 * @file           : update.c
 ******************************************************************************
 * @attention
 *
 * Copyright (C) 2018-present Reso-nance Numerique.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config.h"
#include "basetypes.h"
#include "globals.h"

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"

#include "fatfs.h"
#include "stm32_flash.h"

#include "crc.h"

#include "update_gui.h"

#include "update.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE      512

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t tempBuffer[32] __attribute__((aligned(32)));

/* Private function prototypes -----------------------------------------------*/
static uint32_t update_read_uint32_le(const uint8_t *buffer);
static bool update_calculateCRC(FIL* file);
static bool update_backupFirmware(uint32_t flashStartAddr, uint32_t size, const char* backupFilePath);
static bool update_format(uint32_t cm7_size, uint32_t cm4_size);
static bool update_write(FIL* file, uint32_t cm7_size, uint32_t cm4_size, uint32_t external_size, uint32_t totalBytesToWrite);
static bool update_writeFirmware(FIL* file, uint32_t cm7_size, uint32_t cm4_size, uint32_t totalBytesToWrite, uint32_t* totalBytesWritten);
static bool update_writeExternalData(FIL* file, uint32_t external_size, uint32_t totalBytesToWrite, uint32_t* totalBytesWritten);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Reads a little-endian uint32 from a byte buffer.
 * @param buffer Pointer to the byte buffer.
 * @return 32-bit unsigned integer read from the buffer.
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
 * @param packageFilePath Path to the update package file.
 * @return True if the package was processed successfully, false otherwise.
 */
bool update_processPackageFile(const TCHAR* packageFilePath)
{
    FIL file;
    UINT bytesRead;
    FRESULT res;
    uint32_t totalBytesToWrite = 0;
    uint8_t header[24]; // 4 (magic) + 4 (cm7_size) + 4 (cm4_size) + 4 (external_size) + 8 (version)
    uint32_t cm7_size, cm4_size, external_size;
    char version[9] = {0}; // Version string (8 bytes + null terminator)
    uint8_t magic[4];

    // Open the package file
    res = f_open(&file, packageFilePath, FA_READ);
    if (res != FR_OK)
    {
        printf("Failed to open the package file: %s\n", packageFilePath);
        gui_displayUpdateFailed();
        return false;
    }

    // Read the 24-byte header
    res = f_read(&file, header, sizeof(header), &bytesRead);
    if (res != FR_OK || bytesRead != sizeof(header))
    {
        printf("Failed to read the package header\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Parse the header
    memcpy(magic, header, 4);
    if (memcmp(magic, "BOOT", 4) != 0)
    {
        printf("Invalid package magic number\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Read values considering endianness (little-endian)
    cm7_size = update_read_uint32_le(header + 4);
    cm4_size = update_read_uint32_le(header + 8);
    external_size = update_read_uint32_le(header + 12);
    memcpy(version, header + 16, 8); // Version is 8 bytes

    printf("Package version: %s\n", version);
    printf("CM7 firmware size: %lu bytes\n", cm7_size);
    printf("CM4 firmware size: %lu bytes\n", cm4_size);
    printf("External data size: %lu bytes\n", external_size);

    // Calculate total data size for progress bar
    totalBytesToWrite = cm7_size + cm4_size + external_size;

    // Display version on the screen
    gui_displayVersion(version);

    // Backup firmwares before erasing
    printf("Backing up CM7 firmware...\n");
    if (!update_backupFirmware(FW_CM7_START_ADDR, cm7_size, "0:/backup_cm7.bin"))
    {
        printf("Failed to back up CM7 firmware\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    printf("Backing up CM4 firmware...\n");
    if (!update_backupFirmware(FW_CM4_START_ADDR, cm4_size, "0:/backup_cm4.bin"))
    {
        printf("Failed to back up CM4 firmware\n");
        f_close(&file);
        gui_displayUpdateFailed();
        return false;
    }

    // Calculate and verify CRC
    if (!update_calculateCRC(&file))
    {
        f_close(&file);
        return false;
    }

    // Erase necessary flash sectors
    if (!update_format(cm7_size, cm4_size))
    {
        f_close(&file);
        return false;
    }

    // Write firmware and external data
    if (!update_write(&file, cm7_size, cm4_size, external_size, totalBytesToWrite))
    {
        f_close(&file);
        return false;
    }

    // Close the package file
    f_close(&file);

    // Update progress bar to 100%
    gui_displayUpdateProcess(100);

    // Display success message
    gui_displayUpdateSuccess();

    // Return success
    return true;
}

/**
 * @brief Calculates and verifies the CRC of the update package.
 * @param file Pointer to the open file.
 * @return True if the CRC matches, false otherwise.
 */
static bool update_calculateCRC(FIL* file)
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
    int32_t progressBar = 0;

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

    // Update the progress bar to 0%
    gui_displayUpdateProcess(0);

    while (totalDataRead < crc_length)
    {
        uint32_t bytesToRead = (crc_length - totalDataRead > BUFFER_SIZE) ? BUFFER_SIZE : (crc_length - totalDataRead);
        res = f_read(file, readBuffer, bytesToRead, &bytesRead);
        if (res != FR_OK || bytesRead == 0) {
            // Handle the error
            printf("Error reading the file for CRC calculation\n");
            gui_displayUpdateFailed();
            return false;
        }

        // Calculate the CRC for the read bytes
        crc_calculated = HAL_CRC_Accumulate(&hcrc, (uint32_t *)readBuffer, bytesRead);

        totalDataRead += bytesRead;

        progressBar = (totalDataRead * 4) / crc_length + 1;
        gui_displayUpdateProcess(progressBar);
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
 * @brief Backs up firmware from flash to a file.
 * @param flashStartAddr Starting address of the firmware in flash.
 * @param size Size of the firmware to back up.
 * @param backupFilePath Path to the backup file.
 * @return True if the backup was successful, false otherwise.
 */
static bool update_backupFirmware(uint32_t flashStartAddr, uint32_t size, const char* backupFilePath)
{
    FIL backupFile;
    FRESULT res;
    UINT bytesWritten;
    uint32_t bytesToRead = size;
    uint8_t readBuffer[BUFFER_SIZE] __attribute__((aligned(4)));
    uint32_t flashAddress = flashStartAddr;

    // Open the file for writing
    res = f_open(&backupFile, backupFilePath, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("Failed to open the backup file: %s\n", backupFilePath);
        return false;
    }

    while (bytesToRead > 0)
    {
        uint32_t chunkSize = (bytesToRead > BUFFER_SIZE) ? BUFFER_SIZE : bytesToRead;

        // Read data from flash
        memcpy(readBuffer, (uint8_t*)flashAddress, chunkSize);

        // Write data to the backup file
        res = f_write(&backupFile, readBuffer, chunkSize, &bytesWritten);
        if (res != FR_OK || bytesWritten != chunkSize)
        {
            printf("Failed to write to the backup file: %s\n", backupFilePath);
            f_close(&backupFile);
            return false;
        }

        flashAddress += chunkSize;
        bytesToRead -= chunkSize;
    }

    // Close the backup file
    f_close(&backupFile);

    return true;
}

/**
 * @brief Erases necessary flash sectors for CM7 and CM4 firmware.
 * @param cm7_size Size of the CM7 firmware.
 * @param cm4_size Size of the CM4 firmware.
 * @return True if the sectors were erased successfully, false otherwise.
 */
static bool update_format(uint32_t cm7_size, uint32_t cm4_size)
{
    uint32_t cm7_flashBank = FLASH_BANK_1;
    uint32_t cm7_flashSector = stm32_flashGetSector(FW_CM7_START_ADDR);
    uint32_t cm7_NbSectors = (cm7_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE; // Calculate the number of required sectors
    uint32_t cm4_flashBank = FLASH_BANK_2;
    uint32_t cm4_flashSector = stm32_flashGetSector(FW_CM4_START_ADDR);
    uint32_t cm4_NbSectors = (cm4_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE; // Calculate the number of required sectors
    int32_t progressBar = 0;
    STM32Flash_StatusTypeDef status;

    // Erase CM7 sectors
    printf("Erasing CM7 flash sectors...\n");
    for (uint32_t sector = cm7_flashSector; sector < cm7_flashSector + cm7_NbSectors; sector++)
    {
        status = STM32Flash_erase_sector(cm7_flashBank, sector);
        if (status != STM32FLASH_OK)
        {
            printf("Failed to erase CM7 sector %lu\n", sector);
            gui_displayUpdateFailed();
            return false;
        }

        progressBar = ((sector - cm7_flashSector + 1) * 10) / cm7_NbSectors;
        progressBar += 5;

        // Update the progress bar
        gui_displayUpdateProcess(progressBar);
    }

    // Erase CM4 sectors
    printf("Erasing CM4 flash sectors...\n");
    for (uint32_t sector = cm4_flashSector; sector < cm4_flashSector + cm4_NbSectors; sector++)
    {
        status = STM32Flash_erase_sector(cm4_flashBank, sector);
        if (status != STM32FLASH_OK)
        {
            printf("Failed to erase CM4 sector %lu\n", sector);
            gui_displayUpdateFailed();
            return false;
        }

        progressBar = ((sector - cm4_flashSector + 1) * 5) / cm4_NbSectors;
        progressBar += 15;

        // Update the progress bar
        gui_displayUpdateProcess(progressBar);
    }

    return true;
}

/**
 * @brief Writes the firmware and external data from the package file.
 * @param file Pointer to the open package file.
 * @param cm7_size Size of the CM7 firmware.
 * @param cm4_size Size of the CM4 firmware.
 * @param external_size Size of the external data.
 * @param totalBytesToWrite Total bytes to write for progress calculation.
 * @return True if the write operations were successful, false otherwise.
 */
static bool update_write(FIL* file, uint32_t cm7_size, uint32_t cm4_size, uint32_t external_size, uint32_t totalBytesToWrite)
{
    uint32_t totalBytesWritten = 0;
    FRESULT res;

    // Reposition the file pointer after the header to start reading data
    res = f_lseek(file, 24); // size of header is 24 bytes
    if (res != FR_OK)
    {
        printf("Failed to reposition after the header\n");
        gui_displayUpdateFailed();
        return false;
    }

    // Write the firmware
    if (!update_writeFirmware(file, cm7_size, cm4_size, totalBytesToWrite, &totalBytesWritten))
    {
        return false;
    }

    // Write the external data
    if (!update_writeExternalData(file, external_size, totalBytesToWrite, &totalBytesWritten))
    {
        return false;
    }

    return true;
}

/**
 * @brief Writes the CM7 and CM4 firmware to internal flash.
 * @param file Pointer to the open package file.
 * @param cm7_size Size of the CM7 firmware.
 * @param cm4_size Size of the CM4 firmware.
 * @param totalBytesToWrite Total bytes to write for progress calculation.
 * @param totalBytesWritten Pointer to total bytes written so far.
 * @return True if the firmware was written successfully, false otherwise.
 */
static bool update_writeFirmware(FIL* file, uint32_t cm7_size, uint32_t cm4_size, uint32_t totalBytesToWrite, uint32_t* totalBytesWritten)
{
    UINT bytesRead;
    FRESULT res;
    uint32_t flashAddress;
    STM32Flash_StatusTypeDef status;
    uint8_t readBuffer[BUFFER_SIZE] __attribute__((aligned(4))); // Buffer aligned to 4 bytes
    uint8_t tempBuffer[32] __attribute__((aligned(32)));
    int32_t progressBar = 0;

    // Write the CM7 firmware
    printf("Writing CM7 firmware...\n");
    flashAddress = FW_CM7_START_ADDR;
    uint32_t bytesToWrite = cm7_size;

    while (bytesToWrite > 0)
    {
        uint32_t chunkSize = (bytesToWrite > BUFFER_SIZE) ? BUFFER_SIZE : bytesToWrite;
        res = f_read(file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize)
        {
            printf("Failed to read CM7 firmware data\n");
            gui_displayUpdateFailed();
            return false;
        }

        // Write data to flash using STM32Flash_write32B
        uint32_t writeOffset = 0;
        while (writeOffset < bytesRead)
        {
            uint32_t writeSize = ((bytesRead - writeOffset) >= 32) ? 32 : (bytesRead - writeOffset);
            uint8_t* dataPtr = readBuffer + writeOffset;

            // Ensure that the data is aligned to 32 bytes
            if (((uint32_t)dataPtr % 32 != 0) || (writeSize < 32))
            {
                memset(tempBuffer, 0xFF, 32);
                memcpy(tempBuffer, dataPtr, writeSize);
                dataPtr = tempBuffer;
            }

            // Ensure that the flash address is aligned
            if (flashAddress % 32 != 0)
            {
                printf("Unaligned flash address: 0x%08lx\n", flashAddress);
                gui_displayUpdateFailed();
                return false;
            }

            status = STM32Flash_write32B(dataPtr, flashAddress);
            if (status != STM32FLASH_OK)
            {
                printf("Failed to write to flash at address 0x%08lx\n", flashAddress);
                gui_displayUpdateFailed();
                return false;
            }

            flashAddress += 32;
            writeOffset += writeSize;
            *totalBytesWritten += writeSize;
        }

        bytesToWrite -= bytesRead;

        progressBar = ((*totalBytesWritten * 80) / totalBytesToWrite);
        progressBar += 20;

        // Update the progress bar
        gui_displayUpdateProcess(progressBar);
    }

    // Write the CM4 firmware
    printf("Writing CM4 firmware...\n");
    flashAddress = FW_CM4_START_ADDR;
    bytesToWrite = cm4_size;

    while (bytesToWrite > 0)
    {
        uint32_t chunkSize = (bytesToWrite > BUFFER_SIZE) ? BUFFER_SIZE : bytesToWrite;
        res = f_read(file, readBuffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead != chunkSize)
        {
            printf("Failed to read CM4 firmware data\n");
            gui_displayUpdateFailed();
            return false;
        }

        // Write data to flash using STM32Flash_write32B
        uint32_t writeOffset = 0;
        while (writeOffset < bytesRead)
        {
            uint32_t writeSize = ((bytesRead - writeOffset) >= 32) ? 32 : (bytesRead - writeOffset);
            uint8_t* dataPtr = readBuffer + writeOffset;

            // Ensure that the data is aligned to 32 bytes
            if (((uint32_t)dataPtr % 32 != 0) || (writeSize < 32))
            {
                memset(tempBuffer, 0xFF, 32);
                memcpy(tempBuffer, dataPtr, writeSize);
                dataPtr = tempBuffer;
            }

            // Ensure that the flash address is aligned
            if (flashAddress % 32 != 0)
            {
                printf("Unaligned flash address: 0x%08lx\n", flashAddress);
                gui_displayUpdateFailed();
                return false;
            }

            status = STM32Flash_write32B(dataPtr, flashAddress);
            if (status != STM32FLASH_OK)
            {
                printf("Failed to write to flash at address 0x%08lx\n", flashAddress);
                gui_displayUpdateFailed();
                return false;
            }

            flashAddress += 32;
            writeOffset += writeSize;
            *totalBytesWritten += writeSize;
        }

        bytesToWrite -= bytesRead;

        progressBar = ((*totalBytesWritten * 80) / totalBytesToWrite);
        progressBar += 20;

        // Update the progress bar
        gui_displayUpdateProcess(progressBar);
    }

    return true;
}

/**
 * @brief Writes the external data to the file system.
 * @param file Pointer to the open package file.
 * @param external_size Size of the external data.
 * @param totalBytesToWrite Total bytes to write for progress calculation.
 * @param totalBytesWritten Pointer to total bytes written so far.
 * @return True if the external data was written successfully, false otherwise.
 */
static bool update_writeExternalData(FIL* file, uint32_t external_size, uint32_t totalBytesToWrite, uint32_t* totalBytesWritten)
{
    UINT bytesRead;
    FRESULT res;
    uint8_t readBuffer[BUFFER_SIZE] __attribute__((aligned(4))); // Buffer aligned to 4 bytes
    int32_t progressBar = 0;

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
        *totalBytesWritten += bytesRead;

        progressBar = ((*totalBytesWritten * 80) / totalBytesToWrite);
        progressBar += 20;

        // Update the progress bar
        gui_displayUpdateProcess(progressBar);
    }

    // Close the external file
    f_close(&externalFile);

    return true;
}
