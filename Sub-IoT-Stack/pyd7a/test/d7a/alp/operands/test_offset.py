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
import unittest

from bitstring import ConstBitStream

from d7a.alp.operands.length import Length
from d7a.alp.operands.offset import Offset


class TestOffset(unittest.TestCase):
  def test_byte_generation(self):
    bytes = bytearray(Offset())
    self.assertEqual(len(bytes), 2)
    self.assertEqual(bytes[0], int('00000000', 2))
    self.assertEqual(bytes[1], int('00000000', 2))

    bytes = bytearray(Offset(id=0xFF))
    self.assertEqual(len(bytes), 2)
    self.assertEqual(bytes[0], int('11111111', 2))
    self.assertEqual(bytes[1], int('00000000', 2))

    bytes = bytearray(Offset(offset=Length(65120)))
    self.assertEqual(len(bytes), 4)
    self.assertEqual(bytes[0], int('00000000', 2))
    self.assertEqual(bytes[1], int('10000000', 2))
    self.assertEqual(bytes[2], int('11111110', 2))
    self.assertEqual(bytes[3], int('01100000', 2))

  def test_parse(self):
    offset_bytes = [0x20, 0x01]
    offset = Offset.parse(ConstBitStream(bytes=offset_bytes))
    self.assertEqual(offset.id, 32)
    self.assertEqual(offset.offset.value, 1)

  def test_parse_two_bytes(self):
    offset_bytes = [0x20, 0x40, 0x41]
    offset = Offset.parse(ConstBitStream(bytes=offset_bytes))
    self.assertEqual(offset.offset.value, 65)

  def test_parse_three_bytes(self):
    offset_bytes = [0x01, 0x80, 0xfe, 0x60]
    offset = Offset.parse(ConstBitStream(bytes=offset_bytes))
    self.assertEqual(offset.id, 1)
    self.assertEqual(offset.offset.value, 65120)

  def test_to_str(self):
    Offset().__str__()