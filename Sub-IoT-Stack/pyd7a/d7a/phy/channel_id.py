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


class ChannelID(Validatable):
  
  SCHEMA = [{
    "channel_header": Types.OBJECT(ChannelHeader),
    "channel_index": Types.INTEGER(min=0, max=0xFFFF),
  }]

  def __init__(self, channel_header, channel_index):
    self.channel_header = channel_header
    self.channel_index = channel_index
    super(ChannelID, self).__init__()

  def __iter__(self):
    for byte in self.channel_header: yield byte
    for byte in bytearray(struct.pack(">H", self.channel_index)): yield byte


  @staticmethod
  def parse(s):
    channel_header = ChannelHeader.parse(s)
    channel_index = s.read("uint:16")
    return ChannelID(channel_header=channel_header, channel_index=channel_index)

  def __str__(self):
    return "{0}{1:0>3}".format(self.channel_header, self.channel_index)

  @staticmethod
  def from_string(s):
      channel_header = ChannelHeader.from_string(s[0:5])
      channel_index = int(s[5:8])
      return ChannelID(channel_header=channel_header, channel_index=channel_index)