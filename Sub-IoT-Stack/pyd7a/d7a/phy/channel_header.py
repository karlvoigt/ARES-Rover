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

from d7a.support.schema import Validatable, Types


class ChannelCoding(Enum):
  PN9 = 0x00
  RFU = 0x01
  FEC_PN9 = 0x02
  CW = 0x03

  def to_char(self):
    return self.name[:1]

  @staticmethod
  def from_char(c):
    if c == "P": return ChannelCoding.PN9
    if c == "F": return ChannelCoding.FEC_PN9
    if c == "C": return ChannelCoding.CW

    raise NotImplementedError

class ChannelClass(Enum):
  LO_RATE = 0x00
  LORA = 0x01 # TODO not part of spec
  NORMAL_RATE = 0x02
  HI_RATE = 0x03

  def to_char(self):
    c = self.name[:1]
    if self.value == ChannelClass.LORA:
      c = "R"

    return c

  @staticmethod
  def from_char(c):
    if c == "L": return ChannelClass.LO_RATE
    if c == "N": return ChannelClass.NORMAL_RATE
    if c == "H": return ChannelClass.HI_RATE
    if c == "R": return ChannelClass.LORA

    raise NotImplementedError

class ChannelBand(Enum):
  NOT_IMPL = 0x00
  BAND_433 = 0x02
  BAND_868 = 0x03
  BAND_915 = 0x04

  @staticmethod
  def from_string(s):
    if s == "433": return ChannelBand.BAND_433
    if s == "868": return ChannelBand.BAND_868
    if s == "915": return ChannelBand.BAND_915

    raise NotImplementedError

class ChannelHeader(Validatable):
  # TODO
  SCHEMA = [{
    "channel_coding": Types.ENUM(ChannelCoding),
    "channel_class": Types.ENUM(ChannelClass),
    "channel_band": Types.ENUM(ChannelBand)
  }]

  def __init__(self, channel_coding, channel_class, channel_band):
    self.channel_coding = channel_coding
    self.channel_class = channel_class
    self.channel_band = channel_band
    super(ChannelHeader, self).__init__()

  def __iter__(self):
    byte = self.channel_band.value << 4
    byte += self.channel_class.value << 2
    byte += self.channel_coding.value
    yield byte

  @staticmethod
  def parse(s):
    s.read("uint:1") # RFU
    channel_band = ChannelBand(s.read("uint:3"))
    channel_class = ChannelClass(s.read("uint:2"))
    channel_coding = ChannelCoding(s.read("uint:2"))
    return ChannelHeader(channel_coding=channel_coding, channel_class=channel_class, channel_band=channel_band)

  def __str__(self):
    band = self.channel_band.name.lstrip("BAND_")
    cl = self.channel_class.to_char()
    coding = self.channel_coding.to_char()

    return "{0}{1}{2}".format(band, cl, coding)

  @staticmethod
  def from_string(s):
    channel_band = ChannelBand.from_string(s[0:3])
    channel_class = ChannelClass.from_char(s[3])
    channel_coding = ChannelCoding.from_char(s[4])
    return ChannelHeader(channel_band=channel_band, channel_class=channel_class, channel_coding=channel_coding)

  def __eq__(self, other):
    if type(other) is type(self):
      return self.__dict__ == other.__dict__
    return False

  def __ne__(self, other):
    if isinstance(other, self.__class__):
      return not self.__eq__(other)
    return False