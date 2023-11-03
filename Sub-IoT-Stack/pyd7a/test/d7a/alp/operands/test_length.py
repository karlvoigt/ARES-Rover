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


class TestLength(unittest.TestCase):
  def test_byte_generation(self):
    bytes = bytearray(Length())
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], int('00000000', 2))

    bytes = bytearray(Length(value=4))
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], int('00000100', 2))

    bytes = bytearray(Length(value=65120))
    self.assertEqual(len(bytes), 3)
    self.assertEqual(bytes[0], int('10000000', 2))
    self.assertEqual(bytes[1], int('11111110', 2))
    self.assertEqual(bytes[2], int('01100000', 2))

  def test_parsing(self):
    length_bytes = [0x01]
    length = Length.parse(ConstBitStream(bytes=length_bytes))
    self.assertEqual(length.value, 1)

    length_bytes = [0x40, 0x41]
    length = Length.parse(ConstBitStream(bytes=length_bytes))
    self.assertEqual(length.value, 65)

    length_bytes = [0xC0, 0x41, 0x10, 0x00]
    length = Length.parse(ConstBitStream(bytes=length_bytes))
    self.assertEqual(length.value, 4263936)