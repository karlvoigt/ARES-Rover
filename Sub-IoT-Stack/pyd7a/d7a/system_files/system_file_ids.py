#
# Copyright (c) 2015-2021 University of Antwerp, Aloxy NV.
#
# This file is part of pyd7a.
# See https://github.com/Sub-IoT/pyd7a for further info.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
from enum import Enum


class SystemFileIds(Enum):
  UID = 0x00
  FACTORY_SETTINGS = 0x01
  FIRMWARE_VERSION = 0x02
  DEVICE_CAPACITY = 0x03
  DEVICE_STATUS = 0x04
  ENGINEERING_MODE = 0x05
  VID = 0x06
  RFU_07 = 0x07
  PHY_CONFIG = 0x08
  PHY_STATUS = 0x09
  DLL_CONFIG = 0x0A
  DLL_STATUS = 0x0B
  NWL_ROUTING = 0x0C
  NWL_SECURITY = 0x0D
  NWL_SECURITY_KEY = 0x0E
  NWL_SSR = 0x0F
  NWL_STATUS = 0x10
  TRL_STATUS = 0x11
  SEL_CONFIG = 0x12
  FOF_STATUS = 0x13
  RFU_14 = 0x14
  RFU_15 = 0x15
  RFU_16 = 0x16
  LOCATION_DATA = 0x17
  ALP_ROOT_AUTHENTICATION_KEY = 0x18
  ALP_USER_AUTHENTICATION_KEY = 0x19
  D7AALP_RFU_1A = 0x1A
  D7AALP_RFU_1B = 0x1B
  D7AALP_RFU_1C = 0x1C
  D7AALP_RFU_1D = 0x1D
  D7AALP_RFU_1E = 0x1E
  D7AALP_RFU_1F = 0x1F
  ACCESS_PROFILE_0 = 0x20
  ACCESS_PROFILE_1 = 0x21
  ACCESS_PROFILE_2 = 0x22
  ACCESS_PROFILE_3 = 0x23
  ACCESS_PROFILE_4 = 0x24
  ACCESS_PROFILE_5 = 0x25
  ACCESS_PROFILE_6 = 0x26
  ACCESS_PROFILE_7 = 0x27
  ACCESS_PROFILE_8 = 0x28
  ACCESS_PROFILE_9 = 0x29
  ACCESS_PROFILE_10 = 0x2A
  ACCESS_PROFILE_11 = 0x2B
  ACCESS_PROFILE_12 = 0x2C
  ACCESS_PROFILE_13 = 0x2D
  ACCESS_PROFILE_14 = 0x2E
  RFU_2F = 0x2F