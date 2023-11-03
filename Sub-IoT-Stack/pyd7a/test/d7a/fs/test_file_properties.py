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

from d7a.fs.file_properties import FileProperties, ActionCondition, StorageClass


class TestFileProperties(unittest.TestCase):

  def test_parsing(self):
    properties_bytes = [
      0xB3
    ]

    prop = FileProperties.parse(ConstBitStream(bytes=properties_bytes))

    self.assertEqual(prop.act_enabled, True)
    self.assertEqual(prop.act_condition, ActionCondition.WRITE_FLUSH)
    self.assertEqual(prop.storage_class, StorageClass.PERMANENT)

  def test_byte_generation(self):
    prop = FileProperties(act_enabled=True, act_condition=ActionCondition.WRITE, storage_class=StorageClass.VOLATILE)
    bytes = bytearray(prop)
    self.assertEqual(bytes, bytearray([0xA1]))