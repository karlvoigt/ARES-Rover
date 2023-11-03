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
import unittest
from bitstring import ConstBitStream
from d7a.system_files.uid import UidFile


class TestUiFile(unittest.TestCase):

  def test_default_constructor(self):
    f = UidFile()
    self.assertEqual(f.uid, 0)

  def test_constructor(self):
    uid = 0xDEADBEEF
    f = UidFile(uid=uid)
    self.assertEqual(f.uid, uid)

  def test_invalid_id(self):
    def bad(): UidFile(uid=-1)
    self.assertRaises(ValueError, bad)

  def test_parsing(self):
    file_contents = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09  # UID
     ]

    config = UidFile.parse(ConstBitStream(bytes=file_contents))
    self.assertEqual(config.uid, 9)

  def test_byte_generation(self):
    uid = 12345
    bytes = bytearray(UidFile(uid))
    self.assertEqual(len(bytes), 8)
    self.assertEqual(struct.unpack(">Q", bytes)[0], uid)
