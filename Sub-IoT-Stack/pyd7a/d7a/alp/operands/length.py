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


class Length(Validatable):

  SCHEMA = [
    {
      "value": Types.INTEGER(min=0, max=0x3FFFFFFF)
    }
  ]

  def __init__(self, value=0):
    self.value = value
    super(Length, self).__init__()

  @staticmethod
  def parse(s):
    size = s.read("uint:2")  # + 1 = already read
    value = s.read("uint:" + str(6 + (size * 8)))
    return Length(value=value)

  def __iter__(self):
    byte = 0
    size = 1
    if self.value > 0x3F:
      size = 2
    if self.value > 0x3FFF:
      size = 3
    if self.value > 0x3FFFFF:
      size = 4

    byte += (size - 1) << 6

    if size == 1:
      byte += self.value
      yield byte
    else:
      length_bytes = bytearray(struct.pack(">I", self.value))
      if size == 2:   length_bytes = length_bytes[2:]
      elif size == 3: length_bytes = length_bytes[1:]

      byte += length_bytes[0]
      yield byte
      for byte in length_bytes[1:]: yield byte

  def __str__(self):
    return str(self.value)

  def __eq__(self, other):
      if isinstance(other, self.__class__):
        return self.value == other.value
      elif isinstance(other, int):
        return self.value == other
      else:
        return False

  def __ne__(self, other):
      return not self.__eq__(other)