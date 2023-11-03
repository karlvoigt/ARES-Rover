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

from d7a.fs.file_header import FileHeader
from d7a.fs.file_permissions import FilePermissions
from d7a.fs.file_properties import ActionCondition, StorageClass, FileProperties


class TestFileHeader(unittest.TestCase):

  def test_parsing(self):
    file_header_bytes = [
      0xFC,  # permissions
      0xB3,  # properties
      0x41,  # ALP cmd file id
      0x42,  # interface file id
      0x00, 0x00, 0x01, 0x00,     # file size
      0x00, 0x00, 0x02, 0x00      # allocated size
    ]

    file_header = FileHeader.parse(ConstBitStream(bytes=file_header_bytes))
    permission = file_header.permissions
    self.assertEqual(permission.encrypted, True)
    self.assertEqual(permission.executable, True)
    self.assertEqual(permission.user_readable, True)
    self.assertEqual(permission.user_writable, True)
    self.assertEqual(permission.user_executable, True)
    self.assertEqual(permission.guest_readable, True)
    self.assertEqual(permission.guest_writable, False)
    self.assertEqual(permission.guest_executable, False)
    prop = file_header.properties
    self.assertEqual(prop.act_enabled, True)
    self.assertEqual(prop.act_condition, ActionCondition.WRITE_FLUSH)
    self.assertEqual(prop.storage_class, StorageClass.PERMANENT)
    self.assertEqual(file_header.alp_command_file_id, 0x41)
    self.assertEqual(file_header.interface_file_id, 0x42)
    self.assertEqual(file_header.file_size, 256)
    self.assertEqual(file_header.allocated_size, 512)


  def test_byte_generation(self):
    file_header = FileHeader(
      permissions=FilePermissions(encrypted=True, executable=True, user_readable=True, user_writable=True, user_executable=True,
                   guest_readable=True, guest_writable=False, guest_executable=False),
      properties=FileProperties(act_enabled=True, act_condition=ActionCondition.WRITE_FLUSH, storage_class=StorageClass.PERMANENT),
      alp_command_file_id=0x41,
      interface_file_id=0x42,
      file_size=20,
      allocated_size=40
    )

    bytes = bytearray(file_header)
    self.assertEqual(bytes[0], 0xFC)
    self.assertEqual(bytes[1], 0xB3)
    self.assertEqual(bytes[2], 0x41)
    self.assertEqual(bytes[3], 0x42)
    self.assertEqual(struct.unpack(">I", bytes[4:8])[0], 20)
    self.assertEqual(struct.unpack(">I", bytes[8:12])[0], 40)