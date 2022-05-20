/**
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2019-2020 NXP
 * Copyright 2021 Sendrato - https://sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PWR_INTERFACE_H_
#define _PWR_INTERFACE_H_

#include "EmbeddedTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief:  lp_32k_dyn must match the clock_32k_hk_t structure
 */
typedef struct  {
    uint32_t freq32k;         /*!< 32k clock actual calculated frequency in Hz */
    uint32_t freq32k_16thHz;  /*!< 32k clock actual calculated frequency in 16th of Hz */
    int32_t  ppm_32k;         /*!< the result of 32k clock software calibration in part per million */
} PWR_clock_32k_hk_t;

/**
 * @brief  This function is used to reset the global variable which
 *         permits(on SET)/restricts(On RESET)the device to enter low power
 *         state.
 */
extern void PWR_AllowDeviceToSleep(void);

/**
 * @brief  This function is used to reset the global variable which
 *         permits(on SET)/restricts(On RESET)the device to enter low power
 *         state.
 */
extern void PWR_DisallowDeviceToSleep(void);

/**
 * @brief  Retrieve pwr-clock Handle
 *
 * @return Handle or NULL.
 */
PWR_clock_32k_hk_t *PWR_GetHk32kHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* _PWR_INTERFACE_H_ */
