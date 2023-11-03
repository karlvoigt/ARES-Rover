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

from d7a.system_files.dll_config import DllConfigFile


class TestDllConfigFile(unittest.TestCase):

  def test_default_constructor(self):
    c = DllConfigFile()
    self.assertEqual(c.active_access_class, 0)
    self.assertEqual(c.lq_filter, 0)
    self.assertEqual(c.nf_ctrl, 0)
    self.assertEqual(c.rx_nf_method_parameter, 0)
    self.assertEqual(c.tx_nf_method_parameter, 0)

  def test_invalid_access_class(self):
    def bad(): DllConfigFile(active_access_class=0xFF01) # can be max 0xFF
    self.assertRaises(ValueError, bad)

  def test_parsing(self):
    file_contents = [
      5,          # active access class
      0x00, 0x00, # RFU
      0x01,       # LQ filter
      0x02,       # NF CTRL
      0x03,       # RX NF Method Parameter
      0x04,       # TX NF Method Parameter
     ]

    config = DllConfigFile.parse(ConstBitStream(bytes=file_contents))
    self.assertEqual(config.active_access_class, 5)
    self.assertEqual(config.lq_filter, 1)
    self.assertEqual(config.nf_ctrl, 2)
    self.assertEqual(config.rx_nf_method_parameter, 3)
    self.assertEqual(config.tx_nf_method_parameter, 4)

  def test_byte_generation(self):
    bytes = bytearray(DllConfigFile())
    self.assertEqual(len(bytes), 7)
    self.assertEqual(bytes[0], 0)
    self.assertEqual(bytes[1], 0)
    self.assertEqual(bytes[2], 0)
    self.assertEqual(bytes[3], 0)
    self.assertEqual(bytes[4], 0)
    self.assertEqual(bytes[5], 0)
    self.assertEqual(bytes[6], 0)

    bytes = bytearray(DllConfigFile(active_access_class=5, lq_filter=1, nf_ctrl=2,
                                    rx_nf_method_parameter=3, tx_nf_method_parameter=4))
    self.assertEqual(len(bytes), 7)
    self.assertEqual(bytes[0], 5)
    self.assertEqual(bytes[1], 0)
    self.assertEqual(bytes[2], 0)
    self.assertEqual(bytes[3], 1)
    self.assertEqual(bytes[4], 2)
    self.assertEqual(bytes[5], 3)
    self.assertEqual(bytes[6], 4)