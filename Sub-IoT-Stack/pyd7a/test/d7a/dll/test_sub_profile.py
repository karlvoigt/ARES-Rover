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
from d7a.dll.sub_profile import SubProfile
from d7a.types.ct import CT


class TestSubProfile(unittest.TestCase):

  def test_default_constructor(self):
    sp = SubProfile()
    self.assertEqual(sp.subband_bitmap, 0x00)
    ct = CT()
    self.assertEqual(sp.scan_automation_period.mant, ct.mant)
    self.assertEqual(sp.scan_automation_period.exp, ct.exp)

  def test_byte_generation(self):
    expected = [
      0b10000001, # subband bitmap
      0, # scan automation period
    ]

    sp = SubProfile(subband_bitmap=0b10000001, scan_automation_period=CT(0))
    bytes = bytearray(sp)
    for i in range(len(bytes)):
      self.assertEqual(expected[i], bytes[i])

    self.assertEqual(len(expected), len(bytes))

  def test_parse(self):
    bytes = [
       0b10000001, # subband bitmap
       0,  # scan automation period
     ]

    sp = SubProfile.parse(ConstBitStream(bytes=bytes))
    self.assertEqual(sp.subband_bitmap, 0b10000001)
    self.assertEqual(sp.scan_automation_period.mant, CT(0).mant)
    self.assertEqual(sp.scan_automation_period.exp, CT(0).exp)
