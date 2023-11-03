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

from d7a.phy.channel_header import ChannelHeader, ChannelCoding, ChannelBand, ChannelClass
from d7a.support.schema import Validatable, Types
from d7a.system_files.file import File
from d7a.system_files.system_file_ids import SystemFileIds


class DllStatusFile(File, Validatable):
  SCHEMA = [{
    "last_rx_packet_level": Types.INTEGER(min=0, max=0xFF),
    "last_rx_packet_link_budget": Types.INTEGER(min=0, max=0xFF),
    "noise_floor": Types.INTEGER(min=0, max=0xFF),
    "channel_header": Types.OBJECT(ChannelHeader),
    "channel_index":Types.INTEGER(min=0, max=0xFFFF),
    "scan_timeout_ratio": Types.INTEGER(min=0, max=0xFFFF),
    "scan_count": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "scan_timeout_count": Types.INTEGER(min=0, max=0xFFFFFFFF)
  }]

  def __init__(self, last_rx_packet_level=0, last_rx_packet_link_budget=0, noise_floor=0,
               channel_header=ChannelHeader(channel_coding=ChannelCoding.FEC_PN9, channel_band=ChannelBand.BAND_868, channel_class=ChannelClass.LO_RATE),
               channel_index=0, scan_timeout_ratio=0, scan_count=0, scan_timeout_count=0):
    self.last_rx_packet_level=last_rx_packet_level
    self.last_rx_packet_link_budget=last_rx_packet_link_budget
    self.noise_floor=noise_floor
    self.channel_header=channel_header
    self.channel_index=channel_index
    self.scan_timeout_ratio=scan_timeout_ratio
    self.scan_count=scan_count
    self.scan_timeout_count=scan_timeout_count
    File.__init__(self, SystemFileIds.DLL_STATUS.value, 16)
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=16):
    last_rx_packet_level = s.read("uint:8")
    last_rx_packet_link_budget = s.read("uint:8")
    noise_floor = s.read("uint:8")
    channel_header = ChannelHeader.parse(s)
    channel_index = s.read("uint:16")
    scan_timeout_ratio = s.read("uint:16")
    scan_count = s.read("uint:32")
    scan_timeout_count = s.read("uint:32")
    return DllStatusFile(last_rx_packet_level=last_rx_packet_level, last_rx_packet_link_budget=last_rx_packet_link_budget,
                         noise_floor=noise_floor, channel_header=channel_header, channel_index=channel_index,
                         scan_timeout_ratio=scan_timeout_ratio, scan_count=scan_count, scan_timeout_count=scan_timeout_count)

  def __iter__(self):
    yield self.last_rx_packet_level
    yield self.last_rx_packet_link_budget
    yield self.noise_floor
    for byte in self.channel_header:
      yield byte
    for byte in bytearray(struct.pack(">H", self.channel_index)):
      yield byte
    for byte in bytearray(struct.pack(">H", self.scan_timeout_ratio)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.scan_count)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.scan_timeout_count)):
      yield byte


  def __str__(self):
    return "last_rx_packet_level={}, last_rx_packet_link_budget={}, noise_floor={}, channel={}{}, scan_timeout_ratio={}, scan_count={}, scan_timeout_count={}".format(self.last_rx_packet_level, self.last_rx_packet_link_budget, self.noise_floor, self.channel_header, self.channel_index, self.scan_timeout_ratio, self.scan_count, self.scan_timeout_count)
