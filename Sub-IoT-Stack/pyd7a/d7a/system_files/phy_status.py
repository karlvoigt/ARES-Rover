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
import struct

from d7a.support.schema import Validatable, Types
from d7a.system_files.file import File
from d7a.system_files.system_file_ids import SystemFileIds
from d7a.phy.channel_status_identifier import ChannelStatusIdentifier

class PhyStatusFile(File, Validatable):
  SCHEMA = [{
    "up_time": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "rx_time": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "tx_time": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "tx_duty_cycle": Types.INTEGER(min=0, max=1000),
    "channel_status_list_length": Types.INTEGER(min=0, max=0xFF),
    "channel_status_identifier": Types.LIST(ChannelStatusIdentifier, minlength=0, maxlength=0xFF),
    "channel_noise_floor": Types.LIST(minlength=0, maxlength=0xFF)
  }]

  def __init__(self, up_time=0, rx_time=0, tx_time=0, tx_duty_cycle=0, channel_status_list_length=0, channel_status_identifier=[], channel_noise_floor=[]):
    self.up_time = up_time
    self.rx_time = rx_time
    self.tx_time = tx_time
    self.tx_duty_cycle = tx_duty_cycle
    self.channel_status_list_length = channel_status_list_length

    self.channel_status_identifier = channel_status_identifier
    if len(channel_status_identifier) != channel_status_list_length:
      self.channel_status_identifier.extend([ChannelStatusIdentifier()] * (channel_status_list_length - len(channel_status_identifier)))

    self.channel_noise_floor = channel_noise_floor
    if len(channel_noise_floor) != channel_status_list_length:
      self.channel_noise_floor.extend([0] * (channel_status_list_length - len(channel_noise_floor)))

    File.__init__(self, SystemFileIds.PHY_STATUS.value, 15 + (3 * 10))  # allocate enough space for 20 channels
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=15 + (3 * 10)):
    up_time = s.read("uint:32")
    rx_time = s.read("uint:32")
    tx_time = s.read("uint:32")
    tx_duty_cycle = s.read("uint:16")
    channel_status_list_length = s.read("uint:8")
    channel_status_identifier = []
    channel_noise_floor = []
    for counter in range(channel_status_list_length):
      channel_status_identifier.append(ChannelStatusIdentifier().parse(s))
      channel_noise_floor.append(s.read("uint:8"))

    return PhyStatusFile(up_time=up_time, rx_time=rx_time, tx_time=tx_time, tx_duty_cycle=tx_duty_cycle,
                         channel_status_list_length=channel_status_list_length,
                         channel_status_identifier=channel_status_identifier, channel_noise_floor=channel_noise_floor)

  def __iter__(self):
    for byte in bytearray(struct.pack(">I", self.up_time)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.rx_time)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.tx_time)):
      yield byte
    for byte in bytearray(struct.pack(">H", self.tx_duty_cycle)):
      yield byte
    yield self.channel_status_list_length
    for counter in range(self.channel_status_list_length):
      for byte in self.channel_status_identifier[counter]:
        yield byte
      yield self.channel_noise_floor[counter]

  def __str__(self):
    channel_status = ""
    for counter in range(self.channel_status_list_length):
      channel_status = channel_status + "identifier={}, noise_floor={}; ".format(str(self.channel_status_identifier[counter]), self.channel_noise_floor[counter])
    channel_status = "[{}]".format(channel_status[:-2])

    return "up_time={}, rx_time={}, tx_time={}, tx_duty_cycle={}, channel_status_list_length={}, list={}".format(self.up_time, self.rx_time, self.tx_time, self.tx_duty_cycle, self.channel_status_list_length, channel_status)