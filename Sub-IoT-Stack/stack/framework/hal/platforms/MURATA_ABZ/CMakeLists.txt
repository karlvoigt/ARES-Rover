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

#Check that the correct toolchain for the platform is being used
REQUIRE_TOOLCHAIN(gcc-arm-embedded)

#Define platform specific options
PLATFORM_PARAM(PLATFORM_MCU "STM32L072xx" STRING "The MCU series name of the platform")
SET(JLINK_DEVICE "STM32L072CZ" CACHE STRING "The device passed to JLinkExe for flashing")
PLATFORM_OPTION(PLATFORM_BUILD_BOOTLOADABLE_VERSION "When set builds applications to be deployed using bootloader as well (modified linker script)" FALSE)
PLATFORM_OPTION(PLATFORM_SX127X_USE_PA_BOOST "Use the PA_BOOST output of sx127x chip" FALSE)
PLATFORM_OPTION(PLATFORM_SX127X_OCP_ENABLED "Select whether to enable or disable overcurrent protection" TRUE)
PLATFORM_OPTION(PLATFORM_SX127X_USE_RESET_PIN "Platform has access to the RESET pin of sx127x chip" TRUE)
PLATFORM_OPTION(PLATFORM_SX127X_USE_DIO3_PIN "Platform has access to the DIO3 pin of sx127x chip" TRUE)
PLATFORM_OPTION(PLATFORM_SX127X_USE_VCC_TXCO "Platform has access to the VCC TCXO pin of sx127x chip" TRUE)
PLATFORM_OPTION(PLATFORM_FS_SYSTEMFILES_IN_SEPARATE_LINKER_SECTION "A dedicated linker section is used for storing the systemfiles" TRUE) # on stm32l0 we use this to move the systemfiles to embedded EEPROM

PLATFORM_PARAM(PLATFORM_MODEM_INTERFACE_UART     "0"      STRING "The UART port used by the serial modem interface.")
PLATFORM_PARAM(PLATFORM_MODEM_INTERFACE_BAUDRATE "115200" STRING "The baudrate used by the serial modem interface.")


# CONSOLE
# uses USART2,  (PA2 / PA3)
PLATFORM_PARAM(PLATFORM_CONSOLE_UART     "1"      STRING "The UART port used by the console UART configuration."   )
PLATFORM_PARAM(PLATFORM_CONSOLE_BAUDRATE "115200" STRING "The baudrate used by the second console configuration."       )
PLATFORM_PARAM(PLATFORM_CONSOLE_LOCATION "0" STRING "The location") # TODO remove after removing the pins param from the API

#Add a 'global' definition for the specific MCU
EXPORT_GLOBAL_COMPILE_DEFINITIONS("-D${PLATFORM_MCU}")

#Make the 'inc' directory available so 'platform.h' can be found
EXPORT_GLOBAL_INCLUDE_DIRECTORIES(inc)

INCLUDE_DIRECTORIES(inc)

#Make the 'binary platform dir' available so the 'platform_defs.h' file
#(Generated by PLATFORM_BUILD_SETTINGS_FILE) can be found
EXPORT_GLOBAL_INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

INSERT_C_FLAGS(AFTER "-gdwarf-2" "-mcpu=cortex-m0plus" "-mthumb" "-fmessage-length=0" "-mno-sched-prolog"
    "-fno-builtin" "-ffunction-sections" "-fdata-sections" "-fstack-usage" "-Wstack-usage=128")
INSERT_CXX_FLAGS(AFTER "-gdwarf-2" "-mcpu=cortex-m0plus" "-mthumb" "-fmessage-length=0" "-mno-sched-prolog"
    "-fno-builtin" "-ffunction-sections" "-fdata-sections" "-fstack-usage" "-Wstack-usage=128")

#Add platform specific linker flags
INSERT_LINKER_FLAGS(BEFORE OBJECTS INSERT "--specs=nano.specs" "--specs=nosys.specs" "-Wl,-gc-sections")
INSERT_LINKER_FLAGS(BEFORE LINK_LIBRARIES INSERT "-Wl,--start-group -Wl,--whole-archive") # libs are using weak symbols
INSERT_LINKER_FLAGS(AFTER LINK_LIBRARIES INSERT "-Wl,--no-whole-archive  -Wl,--end-group")

SET(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32L072XZ.ld" CACHE FILEPATH "")
SET(LINKER_FLAGS "-Xlinker -Map=linker.map" CACHE STRING "")

SET(STANDALONE_FLASH_ORIGIN 0x08000000 CACHE STRING "The offset in flash used for linking the text section when not using bootloader")
SET(STANDALONE_FLASH_LENGTH 192K CACHE STRING "The length of the flash section when not using bootloader")

# Add additional definitions to the 'platform_defs.h' file generated by cmake
PLATFORM_HEADER_DEFINE(
    NUMBER PLATFORM_CONSOLE_UART
    PLATFORM_CONSOLE_LOCATION
    PLATFORM_CONSOLE_BAUDRATE
    PLATFORM_MODEM_INTERFACE_UART
    PLATFORM_MODEM_INTERFACE_BAUDRATE
    BOOL
    PLATFORM_SX127X_USE_RESET_PIN
    PLATFORM_SX127X_USE_DIO3_PIN
    PLATFORM_SX127X_USE_PA_BOOST
    PLATFORM_SX127X_OCP_ENABLED
    PLATFORM_SX127X_USE_VCC_TXCO
    PLATFORM_FS_SYSTEMFILES_IN_SEPARATE_LINKER_SECTION
)


#Define the 'platform library'. Every platform must define a 'PLATFORM' object library
ADD_LIBRARY(PLATFORM OBJECT
    platf_main.c
    libc_overrides.c
	inc/platform.h
	inc/ports.h
)

#Include the sources for the stm32l0 chip
ADD_CHIP("stm32l0xx")
ADD_CHIP("stm32_common")

ADD_CHIP("sx127x")

#Build the 'platform_defs.h' settings file
PLATFORM_BUILD_SETTINGS_FILE()
