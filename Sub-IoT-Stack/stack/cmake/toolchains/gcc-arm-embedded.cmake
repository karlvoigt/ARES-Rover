# 
# OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
# lowpower wireless sensor communication
#
# Copyright 2015 University of Antwerp
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#######################################
# Toolchain setup gcc-arm-embedded
#######################################

include(CMakeForceCompiler)

if("${CMAKE_VERSION}" VERSION_GREATER 3.6.0)
	set(CMAKE_C_COMPILER   "arm-none-eabi-gcc")
	set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
	# without these cmake tries to compile/link a test and fails on _exit
	# this _was_ suppressed by the now deprecated FORCE versions
	set(CMAKE_C_COMPILER_WORKS   1)
	set(CMAKE_CXX_COMPILER_WORKS 1)
else()
	# the _force_ macro's are deprecated as of 3.6
	CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
	CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)
endif()

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_CROSSCOMPILING 1)

# set compiler flags to optimize for code size
SET(CMAKE_C_FLAGS_DEBUG "-Og -Werror -Wcast-align=strict -Wall -Wno-unused -Wno-maybe-uninitialized -Wno-switch -Wreturn-type -Wpacked-not-aligned -Wshadow -Wextra" CACHE STRING "")
SET(CMAKE_C_FLAGS_RELEASE "-Og -Werror -Wcast-align=strict -Wall -Wno-unused -Wno-maybe-uninitialized -Wno-switch -Wreturn-type -Wpacked-not-aligned -Wshadow -Wextra" CACHE STRING "")

# set C++ compiler flags to the same as the C compiler flags, but with some -Wno-error flags because of more restrictive
# C++ compiler behaviour.
#   missing-field-initializers - for configuration of structures like e.g. alp_init_args_t
#   register - C++-17+ doesn't like register storage class specified, but not much we can do about this
#   volatile - Likewise, 'volatile'-qualified parameter is deprecated
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-missing-field-initializers -Wno-register -Wno-volatile" CACHE STRING "")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wno-missing-field-initializers -Wno-register -Wno-volatile" CACHE STRING "")

SET(TOOLCHAIN_DIR "" CACHE PATH "The directory containing all the cross compilation tools. (Compilation will fail if this is not set correctly)")

# where is the target environment 
LIST(APPEND CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_DIR}")

MESSAGE(STATUS "Cross-compiling using gcc-arm-embedded toolchain")

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

