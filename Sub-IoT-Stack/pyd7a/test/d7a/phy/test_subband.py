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
from d7a.phy.subband import SubBand


class TestSubband(unittest.TestCase):
  def test_default_ctor(self):
    sb = SubBand()
    self.assertEqual(sb.channel_index_start, 0)
    self.assertEqual(sb.channel_index_end, 0)
    self.assertEqual(sb.eirp, 0)
    self.assertEqual(sb.cca, 86)
    self.assertEqual(sb.duty, 255)

  def test_validation_ok(self):
    sb = SubBand(channel_index_start=0,
                 channel_index_end=0,
                 eirp=10,
                 cca=86,
                 duty=10)

  def test_validation_channel_index_start(self):
    def bad(): sb = SubBand(channel_index_start=-10)
    self.assertRaises(ValueError, bad)


  def test_validation_channel_index_end(self):
    def bad(): sb = SubBand(channel_index_end=-10)
    self.assertRaises(ValueError, bad)

  def test_validation_eirp(self):
    def bad(): sb = SubBand(eirp=200)
    self.assertRaises(ValueError, bad)

  def test_validation_cca(self):
    def bad(): sb = SubBand(cca=-10)
    self.assertRaises(ValueError, bad)

  def test_validation_duty(self):
    def bad(): sb = SubBand(duty=500)
    self.assertRaises(ValueError, bad)


  def test_byte_generation(self):
    expected = [
      0, 0, # channel index start
      0, 16, # channel index end
      10, # eirp
      86, # ccao
      255 # duty
    ]
    sb = SubBand(channel_index_start=0,
                 channel_index_end=16,
                 eirp=10,
                 cca=86,
                 duty=255)
    bytes = bytearray(sb)
    for i in range(len(bytes)):
      self.assertEqual(expected[i], bytes[i])

    self.assertEqual(len(expected), len(bytes))


  def test_byte_generation_neg_eirp(self):
    expected = [
      0, 0, # channel index start
      0, 16, # channel index end
      0xF6, # eirp
      86, # ccao
      255 # duty
    ]
    sb = SubBand(channel_index_start=0,
                 channel_index_end=16,
                 eirp=-10,
                 cca=86,
                 duty=255)
    bytes = bytearray(sb)
    for i in range(len(bytes)):
      self.assertEqual(expected[i], bytes[i])

    self.assertEqual(len(expected), len(bytes))

  def test_parse_neg_eirp(self):
    bytes = [
      0, 0,  # channel index start
      0, 16,  # channel index end
      0xF6,  # eirp
      86,  # ccao
      255  # duty
    ]

    sb = SubBand.parse(ConstBitStream(bytes=bytes))
    self.assertEqual(sb.channel_index_start, 0)
    self.assertEqual(sb.channel_index_end, 16)
    self.assertEqual(sb.eirp, -10)
    self.assertEqual(sb.cca, 86)
    self.assertEqual(sb.duty, 255)