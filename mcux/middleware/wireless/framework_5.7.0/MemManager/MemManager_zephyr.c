/**
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * Copyright 2021 Sendrato - https://sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * This is the source file for the Memory Manager interface.
 * It is a subset of the MCUXpresso SDK and contains the minimum required
 * functionality to support Zephyr.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "MemManager.h"
#include <zephyr.h>

/**
 * @brief Bytes on Zephyr-heap to be allocated for MemManager
 */
K_HEAP_DEFINE(_memHeap, 1024);

/**
 * @brief   This function initializes the message module private variables.
 *          Must be called at boot time, or if device is reset.
 *
 * @return  MEM_SUCCESS_c if init is successful. (It's always successful).
*/
memStatus_t MEM_Init(void)
{
    return MEM_SUCCESS_c;
}


/**
 * @brief  Frees the given buffer.
 *
 * @param[in] buffer   Pointer to memory block to free
 * @return             MEM_SUCCESS_c if free is successful,
 *                     MEM_ALLOC_ERROR_c otherwise
 */
memStatus_t MEM_BufferFree(void* buffer)
{
    if( buffer == NULL ) {
        return MEM_FREE_ERROR_c;
    }

    k_heap_free(&_memHeap, buffer);
    return MEM_SUCCESS_c;
}


/**
 * @brief   Allocate a block from the memory pools. The function uses the
 *          numBytes argument to look up a pool with adequate block sizes.
 *
 * @param[in]  numBytes  Size of buffer to allocate
 * @return               Pointer to the allocated buffer, NULL if failed.
 */
void* MEM_BufferAlloc(uint32_t numBytes)
{
    return k_heap_alloc(&_memHeap, numBytes, K_NO_WAIT);
}


/**
 * @brief   Allocate a block from the memory pools. The function uses the
 *          numBytes argument to look up a pool with adequate block sizes.
 *
 * @param[in]  numBytes  Size of buffer to allocate
 * @param[in]  poolId    <not used>
 * @param[in]  pCaller   <not used>
 * @return               Pointer to the allocated buffer, NULL if failed.
 */
void* MEM_BufferAllocWithId(uint32_t numBytes, uint8_t poolId, void *pCaller)
{
    ARG_UNUSED(poolId);
    ARG_UNUSED(pCaller);
    return k_heap_alloc(&_memHeap, numBytes, K_NO_WAIT);
}
