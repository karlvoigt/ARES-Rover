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

from d7a.phy.channel_header import ChannelHeader
from d7a.support.schema import Validatable, Types


class SubBand(Validatable):
  # TODO update to D7AP v1.1

  SCHEMA = [{
    "channel_index_start": Types.INTEGER(min=0, max=0xFFFF),
    "channel_index_end": Types.INTEGER(min=0, max=0xFFFF),
    "eirp": Types.INTEGER(min=-128, max=127),
    "cca": Types.INTEGER(min=0, max=255),
    "duty": Types.INTEGER(min=0, max=255),
  }]

  def __init__(self, channel_index_start=0, channel_index_end=0, eirp=0, cca=86, duty=255):
    self.channel_index_start = channel_index_start
    self.channel_index_end = channel_index_end
    self.eirp = eirp
    self.cca = cca
    self.duty = duty
    super(SubBand, self).__init__()

  def __iter__(self):
    for byte in bytearray(struct.pack(">H", self.channel_index_start)): yield byte
    for byte in bytearray(struct.pack(">H", self.channel_index_end)): yield byte
    for byte in bytearray(struct.pack("b", self.eirp)): yield byte
    yield self.cca
    yield self.duty

  @staticmethod
  def parse(s):
    channel_index_start = struct.unpack(">H", s.read("bytes:2"))[0]
    channel_index_end = struct.unpack(">H", s.read("bytes:2"))[0]
    eirp = s.read("int:8")
    cca = s.read("uint:8")
    duty = s.read("uint:8")

    return SubBand(channel_index_start=channel_index_start,
                   channel_index_end=channel_index_end,
                   eirp=eirp,
                   cca=cca,
                   duty=duty)

  def __str__(self):
    return "channel_index_start={}, channel_index_end={}, eirp={}, cca={}, duty={}".format(
      self.channel_index_start,
      self.channel_index_end,
      self.eirp,
      self.cca,
      self.duty
    )