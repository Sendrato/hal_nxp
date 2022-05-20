/**
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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

#include "RNG.h"

#include <zephyr.h>
#include <random/rand32.h>


/**
 * @brief  Returns a random 32 bit word
 *
 * A number is read from the HW RNG module. If the HW fails, the SW PRNG is
 * used as backup.
 *
 * @param[out] pRandomNo    Pointer to location where the value will be stored
 * @return                  Status of the RNG initialization procedure.
 */
uint8_t RNG_HwGetRandomNo(uint32_t* pRandomNo)
{
    uint8_t status = gRngSuccess_d;
    *pRandomNo = sys_rand32_get();
    return status;
}

/**
 * @brief  Returns a random number between 0 an 256.
 *
 * @return            random number
 */
uint32_t RND_u32GetRand256(void)
{
    uint32_t n = sys_rand32_get();
    return n % 256;
}

/**
 * @brief  Returns a random number between the specified min and max values.
 *
 * @param[in] u32Min  minimum value
 * @param[in] u32Max  maximum value
 * @return            random number
 */
uint32_t RND_u32GetRand(uint32_t u32Min, uint32_t u32Max)
{
    uint32_t n = sys_rand32_get();
    return n % (u32Max - u32Min) + u32Min;
}
