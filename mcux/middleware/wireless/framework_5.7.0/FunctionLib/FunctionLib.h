/**
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * Copyright 2021 Sendrato - https://sendrato.com
 * All rights reserved.
 *
 * @file
 *
 * This is the header file for the Function Lib module header file.
 * It is a subset of the MCUXpresso SDK and contains the minimum required
 * functionality to support Zephyr.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FUNCTION_LIB_H_
#define _FUNCTION_LIB_H_

#include "EmbeddedTypes.h"

/**
 * @brief  Copy the content of one memory block to another. The amount of data
 *         to copy must be specified in number of bytes.
 *
 * The source and destination buffers must not overlap.
 *
 * @param[out] pDst   Pointer to destination memory block
 * @param[in] pSrc    Pointer to source memory block
 * @param[in] cBytes  Number of bytes to copy
 */
void FLib_MemCpy(void *pDst, const void *pSrc, uint32_t cBytes);

/**
 * @brief  Copy bytes. The byte at index i from the source buffer is copied to
 *         index ((n-1) - i) in the destination buffer (and vice versa).
 *
 * @param[out] pDst   Pointer to destination memory block
 * @param[in] pSrc    Pointer to source memory block
 * @param[in] cBytes  Number of bytes to copy
 */
void FLib_MemCpyReverseOrder(void *pDst, const void *pSrc, uint32_t cBytes);

/**
 * @brief  Reset bytes in a memory block to a certain value. The value, and the
 *         number of bytes to be set, are supplied as arguments.
 *
 * @param[in] pData   Pointer to memory block to reset
 * @param[in] value   Value that memory block will be set to
 * @param[in] cBytes  Number of bytes to set
 */
void FLib_MemSet(void *pData, uint8_t value, uint32_t cBytes);

/**
 * @brief  This function compares two buffers.
 *
 * @param[in]  pData1  First buffer to compare.
 * @param[in]  pData2  Second buffer to compare.
 * @param[in]  cBytes Number of bytes to compare.
 * @return            TRUE if the buffers are equal and FALSE otherwise.
 */
bool_t FLib_MemCmp(const void* pData1, const void *pData2, uint32_t cBytes);

/*! *********************************************************************************
* \brief  Compare each byte of a memory block to a given value. The number of bytes to compare
          must be specified.
*         If all the bytes are equal to the given value, the function returns TRUE,
*         and FALSE otherwise.
*
* \param[in] pAddr   Location to be compared
* \param[in] val     Reference value
* \param[in] length  Number of bytes to compare
*
* \return  TRUE if all bytes match and FALSE otherwise.
*
********************************************************************************** */
bool_t FLib_MemCmpToVal(const void* pAddr,
                        uint8_t val,
                        uint32_t len
);
#endif /* _FUNCTION_LIB_H_ */
