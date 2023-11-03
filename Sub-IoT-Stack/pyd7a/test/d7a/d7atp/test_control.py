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

from d7a.d7atp.control import Control


class TestControl(unittest.TestCase):
  def test_parsing(self):
    ctrl = Control.parse(ConstBitStream(bytes=[0x93]))
    self.assertEqual(ctrl.is_dialog_start, True)
    self.assertEqual(ctrl.has_tl, False)
    self.assertEqual(ctrl.has_te, True)
    self.assertEqual(ctrl.is_ack_requested, False)
    self.assertEqual(ctrl.is_ack_not_void, False)
    self.assertEqual(ctrl.is_ack_record_requested, True)
    self.assertEqual(ctrl.has_agc, True)

  def test_byte_generation(self):
    ctrl = Control(
      is_dialog_start=True,
      has_tl=True,
      has_te=False,
      is_ack_requested=True,
      is_ack_not_void=True,
      is_ack_record_requested=False,
      has_agc=False
    )

    data = bytearray(ctrl)
    self.assertEqual(len(data), 1)
    self.assertEqual(data[0], 0xAC)
