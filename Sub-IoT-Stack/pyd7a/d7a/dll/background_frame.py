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
import pprint

from d7a.d7anp.addressee import IdType
from d7a.support.schema           import Validatable, Types
from d7a.dll.background_frame_control import BackgroundFrameControl
from d7a.d7anp.frame import Frame as D7anpFrame


class BackgroundFrame(Validatable):

  SCHEMA = [{
    "subnet": Types.BYTE(),
    "control": Types.OBJECT(BackgroundFrameControl),
    "payload": Types.BITS(16),
    "crc16"  : Types.BITS(16) # TODO does not work, look into this later {'validator': validate_crc }
  }]

  def __init__(self, subnet, control, payload, crc16):
    self.subnet = subnet
    self.control = control
    self.payload = payload
    self.crc16 = crc16
    # TODO validate CRC

    super(BackgroundFrame, self).__init__()

  # def validate_crc(self, value, error):
  #   raw_data = []
  #   raw_data.append(self.length)
  #   raw_data.append(self.subnet)
  #   raw_data.append(self.control)
  #   raw_data.append(self.target_address)
  #   raw_data.append(self.payload)
  #   crc = calculate_crc(raw_data)


  @staticmethod
  def parse(s):
    subnet = s.read("int:8")
    control = BackgroundFrameControl.parse(s)
    payload = s.read("uint:16")
    crc = s.read("uint:16")

    return BackgroundFrame(
      subnet=subnet,
      control=control,
      payload=payload,
      crc16=crc
    )


  def __iter__(self):
    yield self.subnet
    for byte in self.control: yield byte
    for byte in self.payload: yield byte
    yield self.crc16

  def __str__(self):
    return pprint.PrettyPrinter().pformat(self.as_dict())
