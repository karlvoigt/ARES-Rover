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

from d7a.phy.channel_header import ChannelBand
from d7a.support.schema import Validatable, Types
from enum import Enum

class Bandwidth(Enum):
  kHz25 = 0x01
  kHz200 = 0x00

  def to_string(self):
    return self.name

class ChannelStatusIdentifier(Validatable):
  # TODO update to D7AP v1.1

  SCHEMA = [{
    "channel_band": Types.ENUM(ChannelBand),
    "channel_bandwidth": Types.ENUM(Bandwidth),
    "channel_index": Types.INTEGER(min=0, max=1039)
  }]

  def __init__(self, channel_band=ChannelBand.NOT_IMPL, channel_bandwidth=Bandwidth.kHz25, channel_index=0):
    self.channel_band = channel_band
    self.channel_bandwidth = channel_bandwidth
    self.channel_index = channel_index
    super(ChannelStatusIdentifier, self).__init__()

  def __iter__(self):
    byte = self.channel_band.value << 5
    byte += self.channel_bandwidth.value << 4
    byte += 0 << 3  # RFU
    byte += (self.channel_index >> 8) & 0x07
    yield byte
    yield self.channel_index & 0xFF

  @staticmethod
  def parse(s):
    channel_band = ChannelBand(s.read("uint:3"))
    channel_bandwidth = Bandwidth(s.read("uint:1"))
    s.read("uint:1")  # RFU
    channel_index = s.read("uint:11")
    return ChannelStatusIdentifier(channel_band=channel_band, channel_bandwidth=channel_bandwidth, channel_index=channel_index)

  def __str__(self):
    return "channel_band={}, channel_bandwidth={}, channel_index={}".format(
      self.channel_band.name.lstrip("BAND_"),
      self.channel_bandwidth.to_string(),
      self.channel_index
    )
