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
REQUIRE_TOOLCHAIN(aps-gcc)

#Define platform specific options
PLATFORM_PARAM(${PLATFORM_PREFIX}_MCU "aps3r" STRING "The CPU of the cortus fpga board")
PLATFORM_PARAM(${PLATFORM_PREFIX}_RADIO "sx127x" STRING "The radio chip connected to the cortus fpga board")

# CONSOLE
# uses UART0, location 1
PLATFORM_PARAM(PLATFORM_CONSOLE_UART     "0"      STRING "The UART channel used by the console UART configuration."   )
PLATFORM_PARAM(PLATFORM_CONSOLE_LOCATION "1"      STRING "The route location used by the console UART configuration." )
PLATFORM_PARAM(PLATFORM_CONSOLE_BAUDRATE "115200" STRING "The baudrate used by the second console configuration."       )

PLATFORM_PARAM(PLATFORM_MODEM_INTERFACE_UART     "1"      STRING "The UART channel used by the modem interface UART configuration."   )
PLATFORM_PARAM(PLATFORM_MODEM_INTERFACE_BAUDRATE "115200" STRING "The baudrate used by the modem interface configuration."       )

PLATFORM_PARAM(PLATFORM_NUM_LEDS "8" STRING "The number of LEDs.")
PLATFORM_PARAM(PLATFORM_NUM_BUTTONS "2" STRING "The number of buttons.")

INCLUDE_DIRECTORIES(inc)

#Restrict the number of possible options for the radio option in the usual manner...
SET_PROPERTY(CACHE ${PLATFORM_PREFIX}_RADIO PROPERTY STRINGS "cc1101;sx127x;netdev_driver;internal")
SET_PROPERTY(CACHE ${PLATFORM_PREFIX}_MCU PROPERTY STRINGS "aps3r;aps25;ciot25")

PLATFORM_OPTION(PLATFORM_SX127X_USE_PA_BOOST "Use the PA_BOOST output of sx127x chip" TRUE)

IF(${${PLATFORM_PREFIX}_MCU} STREQUAL "aps3r" AND ${${PLATFORM_PREFIX}_RADIO} STREQUAL "internal")
MESSAGE(FATAL_ERROR "Internal transceiver can be used only with chip CIoT25, CMake will exit.")
ENDIF()

#Make the 'inc' directory available so 'platform.h' can be found
EXPORT_GLOBAL_INCLUDE_DIRECTORIES(inc)

#Make the 'binary platform dir' available so the 'platform_defs.h' file
#(Generated by PLATFORM_BUILD_SETTINGS_FILE) can be found
EXPORT_GLOBAL_INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

#Set platform specific compile options
#INSERT_C_FLAGS(AFTER "-c -Os -Wall -Wcast-align -g3 -MMD -MP -fmessage-length=0")
INSERT_C_FLAGS(AFTER "-Os -Wall -Wcast-align -g3 -MMD -MP -fmessage-length=0")
INSERT_CXX_FLAGS(AFTER "-Os -Wall -Wcast-align -g3 -MMD -MP -fmessage-length=0")

#Add platform specific linker flags
INSERT_LINKER_FLAGS(BEFORE OBJECTS INSERT "")
#INSERT_LINKER_FLAGS(BEFORE LINK_LIBRARIES INSERT "-nostartfiles")
INSERT_LINKER_FLAGS(AFTER LINK_LIBRARIES INSERT "-fno-lto -Os -g3 -Wl,-q -Wl,--noinhibit-exec -Wl,-Map,cortus.map -fno-exceptions")
#INSERT_LINKER_FLAGS(AFTER LINK_LIBRARIES INSERT "-lc -L. -fno-lto -Os -g3 -Wl,-q -Wl,--noinhibit-exec -Wl,-Map,cortus.map -fno-exceptions")

#MESSAGE(STATUS "CMAKE_C_COMPILER ${CMAKE_C_COMPILER}")
#MESSAGE(STATUS "CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}")

#Define the 'platform library'. Every platform must define a 'PLATFORM' object library
ADD_LIBRARY(PLATFORM OBJECT 
        cortus_main.c
        cortus_leds.c
        cortus_userbutton.c
        libc_overrides.c
        inc/button.h
)

#Specify the name of the target processor and include the sources for the selected chip 
IF(${${PLATFORM_PREFIX}_MCU} STREQUAL "aps25")
	INSERT_C_FLAGS(BEFORE "-mcpu=aps25")
	ADD_CHIP("ciot25")
ELSE()
	INSERT_C_FLAGS(BEFORE "-mcpu=aps3r")
	IF(${${PLATFORM_PREFIX}_MCU} STREQUAL "ciot25")
		ADD_CHIP("ciot25")
	ELSE()
		ADD_CHIP("cortus")
	ENDIF()
ENDIF()

#Include the sources for the radio chip, if needed
IF("${${PLATFORM_PREFIX}_RADIO}" STREQUAL "internal")
	#The internal transceiver is selected in ciot25 CMakeLists.txt
ELSE()
	ADD_CHIP(${${PLATFORM_PREFIX}_RADIO})
ENDIF()

PLATFORM_HEADER_DEFINE(
 NUMBER PLATFORM_CONSOLE_UART
        PLATFORM_CONSOLE_LOCATION
        PLATFORM_CONSOLE_BAUDRATE
        PLATFORM_MODEM_INTERFACE_UART
        PLATFORM_MODEM_INTERFACE_BAUDRATE
        PLATFORM_NUM_LEDS
        PLATFORM_NUM_BUTTONS
 BOOL   PLATFORM_SX127X_USE_PA_BOOST
)

#Build the 'platform_defs.h' settings file
PLATFORM_BUILD_SETTINGS_FILE()
