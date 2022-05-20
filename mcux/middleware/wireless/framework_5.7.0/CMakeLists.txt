#
# Copyright 2021 Sendrato B.V. - https://sendrato.com
# All rights reserved.
#

# Common
zephyr_include_directories(Common)

# FunctionLib
zephyr_include_directories(FunctionLib)
zephyr_library_sources(FunctionLib/FunctionLib_zephyr.c)

# LowPower
zephyr_include_directories(LowPower/Interface/qn9090dk6)
zephyr_library_sources(LowPower/Source/qn9090dk6/PWR_zephyr.c)

# MemManager
zephyr_include_directories(MemManager)
zephyr_library_sources(MemManager/MemManager_zephyr.c)

# OSAbstraction
zephyr_include_directories(OSAbstraction/Interface)
zephyr_library_sources(OSAbstraction/Source/fsl_os_abstraction_zephyr.c)

# Panic
zephyr_include_directories(Panic/Interface)
zephyr_library_sources(Panic/Source/Panic.c)

# PDM
#zephyr_include_directories(PDM)
#zephyr_library_sources(PDM/PDM_zephyr.c)

# RNG
zephyr_include_directories(RNG)
zephyr_library_sources(RNG/RNG_zephyr.c)

# SecLib
zephyr_include_directories(SecLib)
zephyr_library_sources(SecLib/SecLib.c)

# XCVR
zephyr_include_directories(XCVR/DK6)
zephyr_library_sources(XCVR/DK6/fsl_xcvr.c)
zephyr_library_import(
        k32_libradio
        ${CMAKE_CURRENT_SOURCE_DIR}/XCVR/lib/libRadio.a
)
