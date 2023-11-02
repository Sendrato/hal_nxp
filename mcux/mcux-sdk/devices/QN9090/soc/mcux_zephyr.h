/**
 * Copyright (c) 2021-2022, sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCUX_ZEPHYR_CONFIG_H_
#define _MCUX_ZEPHYR_CONFIG_H_

/* GAP role */
#if defined(CONFIG_BT_CENTRAL)
#define CFG_CENTRAL
#endif

#if defined(CONFIG_BT_PERIPHERAL)
#define CFG_PERIPHERAL
#endif

#if defined(CONFIG_BT_BROADCASTER)
#define CFG_BROADCASTER
#endif

#if defined(CONFIG_BT_OBSERVER)
#define CFG_OBSERVER
#endif

/* Max number of connections */
#ifndef CONFIG_BT_MAX_CONN
#define gAppMaxConnections_c (1)
#else
#define gAppMaxConnections_c (CONFIG_BT_MAX_CONN)
#endif

#endif /* _MCUX_ZEPHYR_CONFIG_H_ */
