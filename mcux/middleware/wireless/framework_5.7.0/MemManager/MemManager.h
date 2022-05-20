/**
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * Copyright 2021 Sendrato - https://sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * This is the header file for the Memory Manager interface.
 * It is a subset of the MCUXpresso SDK and contains the minimum required
 * functionality to support Zephyr.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MEM_MANAGER_H_
#define _MEM_MANAGER_H_

#include <stddef.h>
#include "EmbeddedTypes.h"

/**
 * @brief Memory-statuses used in MEM_BufferAlloc and MEM_BufferFree
 */
typedef enum {
    MEM_SUCCESS_c = 0,                    /* No error occurred */
    MEM_INIT_ERROR_c,                     /* Memory initialization error */
    MEM_ALLOC_ERROR_c,                    /* Memory allocation error */
    MEM_FREE_ERROR_c,                     /* Memory free error */
    MEM_UNKNOWN_ERROR_c                   /* Something bad has happened... */
} memStatus_t;

/**
 * @brief   This function initializes the message module private variables.
 *          Must be called at boot time, or if device is reset.
 *
 * @return  MEM_SUCCESS_c if init is successful. (It's always successful).
*/
memStatus_t MEM_Init(void);

/**
 * @brief  Frees the given buffer.
 *
 * @param[in] buffer   Pointer to memory block to free
 * @return             MEM_SUCCESS_c if free is successful,
 *                     MEM_ALLOC_ERROR_c otherwise
 */
memStatus_t MEM_BufferFree(void *buffer);

/**
 * @brief   Allocate a block from the memory pools. The function uses the
 *          numBytes argument to look up a pool with adequate block sizes.
 *
 * @param[in]  numBytes  Size of buffer to allocate
 * @return               Pointer to the allocated buffer, NULL if failed.
 */
void* MEM_BufferAlloc(uint32_t numBytes);

/**
 * @brief   Allocate a block from the memory pools. The function uses the
 *          numBytes argument to look up a pool with adequate block sizes.
 *
 * @param[in]  numBytes  Size of buffer to allocate
 * @param[in]  poolId    <not used>
 * @param[in]  pCaller   <not used>
 * @return               Pointer to the allocated buffer, NULL if failed.
 */
void* MEM_BufferAllocWithId(uint32_t numBytes, uint8_t poolId, void *pCaller);

#endif /* _MEM_MANAGER_H_ */
