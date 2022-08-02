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

#include "EmbeddedTypes.h"
#include "PWR_Configuration.h"
#include "PWR_Interface.h"
#include "fsl_os_abstraction.h"

#define FREQ32K_CAL_SHIFT   (4)

#ifndef gAllowDeviceToSleep_c
#define gAllowDeviceToSleep_c 0
#endif

uint8_t mLPMFlag = gAllowDeviceToSleep_c;

#if (cPWR_FullPowerDownMode)
static PWR_clock_32k_hk_t mHk32k = {
        .freq32k        = 32768,
        /* expressed in 16th of Hz: (1<<19) */
        .freq32k_16thHz = (32768 << FREQ32K_CAL_SHIFT),
        /* initialization of the 32k clock calibration in part per million */
        .ppm_32k        = -0x6000,
};
#endif

/**
 * @brief  This function is used to reset the global variable which
 *         permits(on SET)/restricts(On RESET)the device to enter low power
 *         state.
 */
void PWR_AllowDeviceToSleep(void)
{
#if (cPWR_FullPowerDownMode)
    OSA_InterruptDisable();

    if(mLPMFlag > 0) {
        mLPMFlag--;
    }
    PWR_DBG_LOG("mLPMFlag=%d", mLPMFlag);

    OSA_InterruptEnable();
#endif /* (cPWR_FullPowerDownMode) */
}


/**
 * @brief  This function is used to reset the global variable which
 *         permits(on SET)/restricts(On RESET)the device to enter low power
 *         state.
 */
void PWR_DisallowDeviceToSleep(void)
{
#if (cPWR_FullPowerDownMode)
    uint8_t prot;

    OSA_InterruptDisable();

    prot = mLPMFlag + 1;

    if(prot != 0) {
        mLPMFlag++;
    }
    PWR_DBG_LOG("mLPMFlag=%d", mLPMFlag);

    OSA_InterruptEnable();
#endif /* (cPWR_FullPowerDownMode) */
}


/**
 * @brief  Retrieve pwr-clock Handle
 *
 * @return Handle or NULL.
 */
PWR_clock_32k_hk_t *PWR_GetHk32kHandle(void)
{
#if (cPWR_FullPowerDownMode)
    return &mHk32k;
#else
    return NULL;
#endif
}
