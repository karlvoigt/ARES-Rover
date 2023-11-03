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

from d7a.system_files.vid import VidFile

class TestVidFile(unittest.TestCase):

    def test_default_constructor(self):
        c = VidFile()
        self.assertEqual(c.vid, 0xFFFF)
        self.assertEqual(c.control, 0)


    def test_parsing(self):
        file_contents = [
            0x05, 0x03,  # VID
            0x65,        # Control
        ]

        config = VidFile.parse(ConstBitStream(bytes=file_contents))
        self.assertEqual(config.vid, 0x0503)
        self.assertEqual(config.control, 0x65)

    def test_byte_generation(self):
        bytes = bytearray(VidFile())
        self.assertEqual(len(bytes), 3)
        self.assertEqual(bytes[0], 0xFF)
        self.assertEqual(bytes[1], 0xFF)
        self.assertEqual(bytes[2], 0)

        bytes = bytearray(VidFile(vid=0x4536, control=0x12))
        self.assertEqual(len(bytes), 3)
        self.assertEqual(bytes[0], 0x45)
        self.assertEqual(bytes[1], 0x36)
        self.assertEqual(bytes[2], 0x12)
