#[[
Copyright (c) 2015-2021 University of Antwerp, Aloxy NV.

This file is part of Sub-IoT.
See https://github.com/Sub-IoT/Sub-IoT-Stack for further info.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
]]

#Add the linker script to use to the linker flags

INCLUDE_DIRECTORIES(inc)

#Please note we include 'inc' here since for complicated cmake reasons directories exported
#with 'GLOBAL_INCLUDE_DIRECTORIES'from chip directories are not exported to the platform directory
EXPORT_PLATFORM_INCLUDE_DIRECTORIES(inc)

EXPORT_GLOBAL_INCLUDE_DIRECTORIES(inc)

SET(lib_sources stm32_common_atomic.c
                stm32_common_gpio.c
                stm32_common_i2c.c
                stm32_common_mcu.c
                stm32_common_spi.c
                stm32_common_system.c
                stm32_common_timer.c
                stm32_common_rtctimer.c
                stm32_common_uart.c
                stm32_common_watchdog.c
                stm32_common_eeprom.c)
if(FRAMEWORK_SERIAL_PROTOCOL_SUPPORT_DMA)
    list(APPEND lib_sources stm32_common_dma.c)
endif(FRAMEWORK_SERIAL_PROTOCOL_SUPPORT_DMA)
#An object library with name '${CHIP_LIBRARY_NAME}' MUST be generated by the CMakeLists.txt file for every chip
ADD_LIBRARY (${CHIP_LIBRARY_NAME} OBJECT
    ${lib_sources}
)
