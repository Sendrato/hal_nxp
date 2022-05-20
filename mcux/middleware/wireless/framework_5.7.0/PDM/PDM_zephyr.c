/**
 * Copyright 2021 Sendrato - https://sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * Zephyr Port of Persistent Data Manager.
 *
 * Provide management of data which needs to persist over cold or warm start.
 * Utilises the Zephyr Setting API.
 *
 * It is a subset of the MCUXpresso SDK and contains the minimum required
 * functionality to support Zephyr.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PDM.h"

#include <stdio.h>
#include <string.h>

#include <zephyr.h>
#include <settings/settings.h>
#include <sys/printk.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(pdm, LOG_LEVEL_NONE);

int PDM_handle_set(const char *name, size_t len,
                   settings_read_cb read_cb, void *cb_arg);


typedef struct PDM_load_param {
    size_t  len;
    void   *buf;
    uint8_t fetched;
} PDM_load_param_t;


static struct settings_handler PDM_settings_handler = {
    .name = "pdm",
    .h_get = NULL,              // Not used: direct call to load.
    .h_set = PDM_handle_set,    // Used to load all data-id's from NVM at startup
    .h_commit = NULL,           // Not used: direct call to load.
    .h_export = NULL            // Not used: direct call to store.
};


int PDM_handle_set(const char *name, size_t len,
                   settings_read_cb read_cb, void *cb_arg)
{
    const char *next = NULL;
    settings_name_next(name, &next);

    if (next != NULL) {
        LOG_DBG("read: %s ", name);
        return 0;
    }

    return -ENOENT;
}


/**
 * @brief Read single value from NVM
 *
 * When found, the data of the data-entry and the fetched length is copied to
 * the provided @ref(PDM_load_param_t);
 *
 * @param[in] name    Zephyr-settings name of item to load.
 * @param[in] buf     Length of item to read
 * @param[in] cb      Callback to load from NVM
 * @param[in] cb_arg  Callback arguments
 * @param[in] param   Structure in which data is loaded.
 * @return            0 on success, negative upon error.
 */
static int PDM_settings_load_direct(const char *name, size_t len,
                    settings_read_cb read_cb, void *cb_arg,
                    void *param)
{
    int rv = 0;
    const char *next = NULL;

    PDM_load_param_t *load_param = (PDM_load_param_t *)param;
    size_t name_len = settings_name_next(name, &next);
    LOG_DBG(">> name_len:%d name:%s", name_len, name);

    if (name_len == 0) {
        if (len <= load_param->len) {
            rv = read_cb(cb_arg, load_param->buf, len);
            if (rv >= 0) {
                load_param->fetched = 1;
                load_param->len     = rv;
                LOG_DBG("immediate load: OK, len=%d.", rv);
                return 0;
            }

            LOG_DBG("fail (err %d)", rv);
            return rv;
        }
        return -EINVAL;
    }

    /* Other keys aren't served by the callback. */
    /* Return success in order to skip them and keep storage processing. */
    return 0;
}


/**
 * @brief Wrapper to read single value from NVM
 *
 * @param[in] name  Zephyr-settings name of item to read.
 * @param[in] buf   Pointer to store date
 * @param[in] len   Length of buffer
 * @return          Entry-length on success, negative upon error.
 */
static int PDM_settings_load_one(const char *name, void *buf, size_t len)
{
    int rv = 0;

    PDM_load_param_t load_param;
    load_param.fetched = 0;
    load_param.len     = len;
    load_param.buf     = buf;

    rv = settings_load_subtree_direct(name, PDM_settings_load_direct,
                                      (void *)&load_param);
    if (rv == 0) {
        if (load_param.fetched) {
            rv = load_param.len;
        } else {
            rv = -ENOENT;
        }
    }

    return rv;
}


PDM_teStatus PDM_eSaveRecordData( uint16_t u16IdValue,
                                  void *pvDataBuffer,
                                  uint16_t u16Datalength)
{
    char nameValue[4];
    sprintf(nameValue, "%04x", u16IdValue);

    return PDM_zSaveRecordData( nameValue, pvDataBuffer, u16Datalength);
}


PDM_teStatus PDM_eReadDataFromRecord( uint16_t u16IdValue,
                                      void *pvDataBuffer,
                                      uint16_t u16DataBufferLength,
                                      uint16_t *pu16DataBytesRead)
{
    char nameValue[4];
    sprintf(nameValue, "%04x", u16IdValue);

    return PDM_zReadDataFromRecord(nameValue, pvDataBuffer,
                                   u16DataBufferLength, pu16DataBytesRead);
}


bool_t PDM_bDoesDataExist( uint16_t u16IdValue,
                           uint16_t *pu16DataLength)
{
    int rv = 0;
    uint8_t pvDataBuffer[256];
    uint16_t u16DataBufferLength = 256;

    rv = PDM_eReadDataFromRecord(u16IdValue, (void*)pvDataBuffer,
                                 u16DataBufferLength, pu16DataLength);

    if (rv != PDM_E_STATUS_OK) {
        return false;
    }

    return true;
}


PDM_teStatus PDM_zSaveRecordData(const char *name,
                                 void *pvDataBuffer,
                                 uint16_t u16Datalength)
{
    char nameValue[20] = {0};
    strcpy(&nameValue[0], "pdm/");
    strcpy(&nameValue[4], name);

    LOG_DBG("SAVE: nameValue:%s / len:%d", nameValue, u16Datalength);
    int rv = settings_save_one(nameValue, pvDataBuffer, u16Datalength);
    LOG_DBG("SAVE: rv:%d", rv);

    if (rv != 0) {
        return PDM_E_STATUS_NOT_SAVED;
    }
    return PDM_E_STATUS_OK;

}


PDM_teStatus PDM_zReadDataFromRecord(const char *name,
                                     void *pvDataBuffer,
                                     uint16_t  u16DataBufferLength,
                                     uint16_t *pu16DataBytesRead)
{
    char nameValue[15] = {0};
    strcpy(&nameValue[0], "pdm/");
    strcpy(&nameValue[4], name);

    LOG_DBG("LOAD: nameValue:%s / len:%d", nameValue, u16DataBufferLength);
    int rv = PDM_settings_load_one(nameValue, pvDataBuffer, u16DataBufferLength);
    LOG_DBG("LOAD: rv:%d", rv);

    if (rv >= 0) {
        *pu16DataBytesRead = rv;
    } else {
        return PDM_E_STATUS_INVLD_PARAM;
    }

    return PDM_E_STATUS_OK;
}


/**
 * @brief   This function initializes the PDM.
 *
 * @return  0 if init is successful, negative otherwise
 */
int PDM_Init(void)
{
    int rv = 0;

    rv = settings_subsys_init();
    if (rv != 0) {
        LOG_DBG("settings subsys initialization: fail (err %d)", rv);
        return rv;
    }
    LOG_DBG("settings subsys initialization: OK.");

    rv = settings_register(&PDM_settings_handler);
    if (rv != 0) {
        LOG_DBG("subtree <%s> handler registered: fail (err %d)",
                PDM_settings_handler.name, rv);
        return rv;
    }

    LOG_DBG("subtree <%s> handler registered: OK", PDM_settings_handler.name);
    return 0;
}
