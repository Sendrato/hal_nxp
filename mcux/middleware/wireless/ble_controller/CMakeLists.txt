#
# Copyright 2021 Sendrato B.V. - https://sendrato.com
# All rights reserved.
#

zephyr_include_directories(
    interface
    config
)

zephyr_library_sources(
    config/controller_config.c
)

zephyr_library_import(
    k32_libBleController
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_ble_controller.a
)

if ((CONFIG_CODE_DATA_RELOCATION) AND ("${MCUX_DEVICE}" STREQUAL "QN9090"))
    zephyr_code_relocate(config/controller_config.c SRAM1_BSS)
    zephyr_code_relocate(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_ble_controller.a SRAM1_BSS)
endif()