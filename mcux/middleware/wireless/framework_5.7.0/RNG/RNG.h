/**
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * Copyright 2021 Sendrato B.V. - https://sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * It is a subset of the MCUXpresso SDK and contains the minimum required
 * functionality to support Zephyr.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RNG_INTERFACE_H_
#define _RNG_INTERFACE_H_

#include "EmbeddedTypes.h"

/**
 * @brief Return status of RNG functions
 */
#define gRngSuccess_d       (0x00)
#define gRngInternalError_d (0x01)
#define gRngNullPointer_d   (0x80)
#define gRngMaxRequests_d   (100000)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Returns a random 32 bit word
 *
 * A number is read from the HW RNG module. If the HW fails, the SW PRNG is
 * used as backup.
 *
 * @param[out] pRandomNo    Pointer to location where the value will be stored
 * @return                  Status of the RNG initialization procedure.
 */
uint8_t RNG_HwGetRandomNo(uint32_t* pRandomNo);

/**
 * @brief  Returns a random number between the specified min and max values.
 *
 * @param[in] u32Min  minimum value
 * @param[in] u32Max  maximum value
 * @return            random number
 */
uint32_t RND_u32GetRand(uint32_t u32Min, uint32_t u32Max);

#ifdef __cplusplus
}
#endif

#endif /* _RNG_INTERFACE_H_ */
