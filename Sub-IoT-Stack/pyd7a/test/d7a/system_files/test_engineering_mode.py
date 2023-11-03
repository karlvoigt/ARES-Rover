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

from d7a.system_files.engineering_mode import EngineeringModeFile, EngineeringModeMode
from d7a.phy.channel_header import ChannelHeader, ChannelCoding, ChannelClass, ChannelBand
from d7a.phy.channel_id import ChannelID


class TestEngineeringModeFile(unittest.TestCase):

  def test_default_constructor(self):
    c = EngineeringModeFile()
    self.assertEqual(c.mode, EngineeringModeMode.ENGINEERING_MODE_MODE_OFF)
    self.assertEqual(c.flags, 0)
    self.assertEqual(c.timeout, 0)
    self.assertEqual(c.channel_id.channel_header, ChannelHeader(ChannelCoding.PN9, ChannelClass.LO_RATE, ChannelBand.BAND_868))
    self.assertEqual(c.channel_id.channel_index, 0)
    self.assertEqual(c.eirp, 0)

  def test_parsing(self):
    file_contents = [
      1,                 # Engineeringmode mode
      0x01,              # flags
      0x02,              # timeout
      0x4a, 0x00, 0x09,  # ChannelID
      0x04,              # eirp
     ]

    config = EngineeringModeFile.parse(ConstBitStream(bytes=file_contents))
    self.assertEqual(config.mode, EngineeringModeMode.ENGINEERING_MODE_MODE_CONT_TX)
    self.assertEqual(config.flags, 1)
    self.assertEqual(config.timeout, 2)
    self.assertEqual(config.channel_id.channel_header.channel_coding, ChannelCoding.FEC_PN9)
    self.assertEqual(config.channel_id.channel_header, ChannelHeader(ChannelCoding.FEC_PN9, ChannelClass.NORMAL_RATE, ChannelBand.BAND_915))
    self.assertEqual(config.channel_id.channel_index, 9)
    self.assertEqual(config.eirp, 4)

  def test_byte_generation(self):
    bytes = bytearray(EngineeringModeFile())
    self.assertEqual(len(bytes), 9)
    self.assertEqual(bytes[0], 0)
    self.assertEqual(bytes[1], 0)
    self.assertEqual(bytes[2], 0)
    self.assertEqual(bytes[3], 0x30)
    self.assertEqual(bytes[4], 0)
    self.assertEqual(bytes[5], 0)
    self.assertEqual(bytes[6], 0)
    self.assertEqual(bytes[7], 0)
    self.assertEqual(bytes[8], 0)

    bytes = bytearray(EngineeringModeFile(mode=EngineeringModeMode.ENGINEERING_MODE_MODE_PER_RX, flags=9, timeout=8,
                                          channel_id=ChannelID(channel_header=ChannelHeader(ChannelCoding.CW,ChannelClass.HI_RATE,ChannelBand.BAND_868),
                                                               channel_index=7),
                                          eirp=6))
    self.assertEqual(len(bytes), 9)
    self.assertEqual(bytes[0], 3)
    self.assertEqual(bytes[1], 9)
    self.assertEqual(bytes[2], 8)
    self.assertEqual(bytes[3], 0x3F)
    self.assertEqual(bytes[4], 0)
    self.assertEqual(bytes[5], 7)
    self.assertEqual(bytes[6], 6)
    self.assertEqual(bytes[7], 0)
    self.assertEqual(bytes[8], 0)

    bytes = bytearray(EngineeringModeFile(mode=EngineeringModeMode.ENGINEERING_MODE_MODE_PER_RX, flags=9, timeout=8,
                                          channel_id=ChannelID(
                                            channel_header=ChannelHeader(ChannelCoding.CW, ChannelClass.HI_RATE,
                                                                         ChannelBand.BAND_868),
                                            channel_index=7),
                                          eirp=-6))

    self.assertEqual(len(bytes), 9)
    self.assertEqual(bytes[0], 3)
    self.assertEqual(bytes[1], 9)
    self.assertEqual(bytes[2], 8)
    self.assertEqual(bytes[3], 0x3F)
    self.assertEqual(bytes[4], 0)
    self.assertEqual(bytes[5], 7)
    self.assertEqual(bytes[6], 0xFA)
    self.assertEqual(bytes[7], 0)
    self.assertEqual(bytes[8], 0)