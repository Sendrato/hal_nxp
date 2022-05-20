/**
 * Copyright 2012 NXP B.V.
 * Copyright 2021 Sendrato B.V. - https://sendrato.com
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


#ifndef PDM_H_INCLUDED
#define PDM_H_INCLUDED

#include "EmbeddedTypes.h"

#if defined __cplusplus
extern "C" {
#endif

#define PDM_INVALID_ID ((uint16)(-1))

/* Maximum number of bytes of MAC addr */
// TODO: replace PDM_BT_xx with 'BD_ADDR_SIZE' in board.h
#define PDM_SIZE_BT_MAC_ADDR (6)
#define PDM_NAME_BT_MAC_ADDR "btmac"


/**
 * @brief PDM ID range allocation reservations
 *
 * Each PDM_ID_BASE_xxx below is the base value for a block of 256 (0x100) IDs.
 * Within a module the individual IDs used by that module will be an offset from
 * this base.
 *
 * These ID ranges should not be re-used by other modules, even if the modules
 * are not both present in the build.
 *
 * Values should not be changed. Reserve a new range instead of changing an
 * existing range.
 */

/* 0x0000?0x00ff: ZigBee Application Notes */
#define PDM_ID_BASE_APP_ZB (0x0000)
/* 0xf000?0xf0ff: ZigBee ZPS APL layer */
#define PDM_ID_BASE_ZPSAPL (0xf000)
/* 0xf100?0xf1ff: ZigBee ZPS NWK layer */
#define PDM_ID_BASE_ZPSNWK (0xf100)
/* 0xff00?0xffff: Radio driver */
#define PDM_ID_BASE_RADIO  (0xff00)

/**
 * @brief PDM ID individual allocation reservations
 *
 * Each PDM_ID_xxx below is an individual ID within one of the reserved ranges.
 * Individual IDs need to be declared here only if they are shared between modules.
 * Please include a descriptive comment to aid accurate identification.
 */

/* Holds radio KMOD calibration data */
#define PDM_ID_RADIO_SETTINGS (PDM_ID_BASE_RADIO + 0x0000)

/**
 * @brief PDM return statuses
 */
typedef enum
{
    PDM_E_STATUS_OK,
    PDM_E_STATUS_INVLD_PARAM,
    // NVM based PDM codes
    PDM_E_STATUS_PDM_FULL,
    PDM_E_STATUS_NOT_SAVED,
    PDM_E_STATUS_RECOVERED,
    PDM_E_STATUS_PDM_RECOVERED_NOT_SAVED,
    PDM_E_STATUS_USER_BUFFER_SIZE,
    PDM_E_STATUS_BITMAP_SATURATED_NO_INCREMENT,
    PDM_E_STATUS_BITMAP_SATURATED_OK,
    PDM_E_STATUS_IMAGE_BITMAP_COMPLETE,
    PDM_E_STATUS_IMAGE_BITMAP_INCOMPLETE,
    PDM_E_STATUS_INTERNAL_ERROR
} PDM_teStatus;

/**
 * @brief Save a PDM record
 *
 * This function saves the specified application data from RAM to the specified
 * record in NVM. The record is identified by means of a 16-bit user-defined value.
 * When a data record is saved to the NVM for the first time, the data is
 * written provided there are enough NVM segments available to hold the data.
 * Upon subsequent saverequests, if there has been a change between the
 * RAM-based and NVM-based data buffers then the PDM will attempt to re-save
 * only the segments that have changed (if no data has changed, no save will
 * be performed). This is advantageous due to the restricted size of the NVM
 * and the constraint that old data must be preserved while saving changed data
 * to the NVM.
 *
 * Provided that you have registered a callback function with the PDM (see
 * Section 6.3), the callback mechanism will signal when a save has failed.
 * Upon failure, the callback function will be invoked and pass the event
 * E_PDM_SYSTEM_EVENT_DESCRIPTOR_SAVE_FAILED to the application
 *
 * @param[in] u16IdValue    User-defined ID of the record to be saved
 * @param[in] pvDataBuffer  Pointer to data buffer to be saved in NVM
 * @param[in] u16Datalength Length of data to be saved, in bytes
 *
 * @return          PDM_E_STATUS_OK (success)
 *                  PDM_E_STATUS_INVLD_PARAM (specified record ID is invalid)
 *                  PDM_E_STATUS_NOT_SAVED (save to NVM failed)
 */
PDM_teStatus PDM_eSaveRecordData(uint16_t  u16IdValue,
                                 void *pvDataBuffer,
                                 uint16_t  u16Datalength);

/**
 * @brief Read a PDM record
 *
 * This function reads the specified record of application data from the NVM and
 * stores the read data in the supplied data buffer in RAM. The record is
 * specified using its unique 16-bit identifier.
 * Before calling this function, it may be useful to call PDM_bDoesDataExist()
 * in order to determine whether a record with the specified identifier exists
 * in the EEPROM and, if it does, to obtain its size.
 *
 * @param[in]  u16IdValue            User-defined ID of the record to be read
 * @param[out] pvDataBuffer          Pointer to the data buffer in RAM where the
 *                                   read data is to be stored
 * @param[in]  u16DataBufferLength   Length of the data buffer, in bytes
 * @param[out] pu16DataBytesRead     Pointer to a location to receive the number
 *                                   of data bytes read.
 *
 * @return       PDM_E_STATUS_OK if success
 *               PDM_E_STATUS_INVLD_PARAM otherwise
 */
PDM_teStatus PDM_eReadDataFromRecord(uint16_t  u16IdValue,
                                     void *pvDataBuffer,
                                     uint16_t  u16DataBufferLength,
                                     uint16_t *pu16DataBytesRead);

/**
 * @brief Seeks for a certain record by record identifier
 *
 * This function checks whether data associated with thd specified record ID
 * exists in the NVM. If the data record exists, the function returns the data
 * length, in bytes, in a location to which a pointer must be provided.
 *
 * @param[in] u16IdValue      User-defined ID of the record to be found
 * @param[out] pu16DataLength Pointer to location to receive length, in bytes,
 *                            of data record associated with specified record ID
 *
 * @return               true if record was fond false otherwise
 */
bool_t PDM_bDoesDataExist(uint16_t u16IdValue, uint16_t *pu16DataLength);


/**
 * @brief Save a PDM-Zephyr record
 *
 * Utilises directly the Zephyr Settings Subsystem.
 *
 * @param[in]  name         String-name of setting, max 10 chars
 * @param[in] pvDataBuffer  Pointer to data buffer to be saved in NVM
 * @param[in] u16Datalength Length of data to be saved, in bytes
 *
 * @return          PDM_E_STATUS_OK (success)
 *                  PDM_E_STATUS_INVLD_PARAM (specified record ID is invalid)
 *                  PDM_E_STATUS_NOT_SAVED (save to NVM failed)
 */
 PDM_teStatus PDM_zSaveRecordData(const char *name,
                                 void *pvDataBuffer,
                                 uint16_t u16Datalength);


/**
 * @brief Read a PDM-Zephyr record
 *
 * Utilises directly the Zephyr Settings Subsystem.
 *
 * @param[in]  name                  String-name of setting, max 10 chars
 * @param[out] pvDataBuffer          Pointer to the data buffer in RAM where the
 *                                   read data is to be stored
 * @param[in]  u16DataBufferLength   Length of the data buffer, in bytes
 * @param[out] pu16DataBytesRead     Pointer to a location to receive the number
 *                                   of data bytes read.
 *
 * @return       PDM_E_STATUS_OK if success
 *               PDM_E_STATUS_INVLD_PARAM otherwise
 */
PDM_teStatus PDM_zReadDataFromRecord(const char *name,
                                     void *pvDataBuffer,
                                     uint16_t  u16DataBufferLength,
                                     uint16_t *pu16DataBytesRead);


/**
 * @brief   This function initializes the PDM.
 *
 * @return  0 if init is successful, negative otherwise
 */
int PDM_Init(void);

#endif /*PDM_H_INCLUDED*/
