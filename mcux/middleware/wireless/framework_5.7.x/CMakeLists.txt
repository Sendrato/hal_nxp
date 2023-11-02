#
# Copyright 2021 Sendrato B.V. - https://sendrato.com
# All rights reserved.
#

# Common
zephyr_include_directories(Common)

# Lists
zephyr_include_directories(Lists)
zephyr_library_sources(Lists/GenericList.c)

# TimersManager
zephyr_include_directories(TimersManager/Interface)

# Panic
zephyr_include_directories(Panic/Interface)
zephyr_library_sources(Panic/Source/Panic.c)

# MemManager
zephyr_include_directories(MemManager)
zephyr_library_sources(MemManager/MemManager_zephyr.c)

# FunctionLib
zephyr_include_directories(FunctionLib)
zephyr_library_sources(FunctionLib/FunctionLib_zephyr.c)

# SecLib
zephyr_include_directories(SecLib)
zephyr_library_sources(SecLib/SecLib.c)
zephyr_library_import(k32_lib_crypto_m4 ${CMAKE_CURRENT_SOURCE_DIR}/SecLib/lib_crypto_m4.a)

# OSAbstraction
zephyr_include_directories(OSAbstraction/Interface)
zephyr_library_sources(OSAbstraction/Source/fsl_os_abstraction_zephyr.c)

# LowPower
zephyr_include_directories(LowPower/Interface/qn9090dk6)
zephyr_library_sources(LowPower/Source/qn9090dk6/PWR_zephyr.c)

# RNG
zephyr_include_directories(RNG/Interface)
zephyr_library_sources(RNG/Source/RNG_zephyr.c)

# XCVR
zephyr_include_directories(XCVR/DK6)
zephyr_library_sources(XCVR/DK6/fsl_xcvr.c)
zephyr_library_import(
        k32_libradio
        ${CMAKE_CURRENT_SOURCE_DIR}/XCVR/lib/libRadio.a
)

if ((CONFIG_CODE_DATA_RELOCATION) AND ("${MCUX_DEVICE}" STREQUAL "QN9090"))
    zephyr_code_relocate(FILES Lists/GenericList.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES Panic/Source/Panic.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES MemManager/MemManager_zephyr.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES FunctionLib/FunctionLib_zephyr.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES SecLib/SecLib.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES ${CMAKE_CURRENT_SOURCE_DIR}/SecLib/lib_crypto_m4.a LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES OSAbstraction/Source/fsl_os_abstraction_zephyr.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES LowPower/Source/qn9090dk6/PWR_zephyr.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES RNG/Source/RNG_zephyr.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES XCVR/DK6/fsl_xcvr.c LOCATION SRAM1_BSS)
    zephyr_code_relocate(FILES ${CMAKE_CURRENT_SOURCE_DIR}/XCVR/lib/libRadio.a LOCATION SRAM1_BSS)
endif()