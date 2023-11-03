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

from d7a.phy.channel_header import ChannelHeader, ChannelCoding, ChannelClass, ChannelBand


class TestChannelHeader(unittest.TestCase):
  def test_validation_ok(self):
    ch = ChannelHeader(channel_coding=ChannelCoding.PN9,
                       channel_class=ChannelClass.NORMAL_RATE,
                       channel_band=ChannelBand.BAND_433)

  def test_validation_channel_coding(self):
    def bad():
      ch = ChannelHeader(channel_coding="wrong",
                         channel_class=ChannelClass.NORMAL_RATE,
                         channel_band=ChannelBand.BAND_433)

    self.assertRaises(ValueError, bad)

  def test_validation_channel_class(self):
    def bad():
      ch = ChannelHeader(channel_coding=ChannelCoding.PN9,
                         channel_class="wrong",
                         channel_band=ChannelBand.BAND_433)

    self.assertRaises(ValueError, bad)

  def test_validation_channel_band(self):
    def bad():
      ch = ChannelHeader(channel_coding=ChannelCoding.PN9,
                         channel_class=ChannelClass.NORMAL_RATE,
                         channel_band="wrong")

    self.assertRaises(ValueError, bad)

  def test_byte_generation(self):
    expected = [
      0b00101000
    ]

    channel_header = ChannelHeader(channel_coding=ChannelCoding.PN9,
                                   channel_class=ChannelClass.NORMAL_RATE,
                                   channel_band=ChannelBand.BAND_433)
    bytes = bytearray(channel_header)
    for i in range(len(bytes)):
      self.assertEqual(expected[i], bytes[i])

    self.assertEqual(len(expected), len(bytes))

  def test_parse(self):
    bytes = [
      0b00101000
    ]

    ch = ChannelHeader.parse(ConstBitStream(bytes=bytes))

    self.assertEqual(ch.channel_coding, ChannelCoding.PN9)
    self.assertEqual(ch.channel_class, ChannelClass.NORMAL_RATE)
    self.assertEqual(ch.channel_band, ChannelBand.BAND_433)
