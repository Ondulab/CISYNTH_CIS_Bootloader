/**
 ******************************************************************************
 * @file           : update.h
 * @brief          : Header for update.c file.
 *                   Contains prototypes and definitions for update functionality.
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

#ifndef UPDATE_H
#define UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ff.h"       // For FatFS types

/* Exported constants --------------------------------------------------------*/
#define FW_CM7_START_ADDR 0x08040000
#define FW_CM4_START_ADDR 0x08100000

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

bool update_processPackageFile(const TCHAR *packageFilePath);

/* Exported macros -----------------------------------------------------------*/
/* Add any necessary macros here */

#ifdef __cplusplus
}
#endif

#endif /* UPDATE_H */
