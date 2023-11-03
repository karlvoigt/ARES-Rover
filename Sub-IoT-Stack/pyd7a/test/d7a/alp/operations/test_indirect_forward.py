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

from d7a.alp.operands.indirect_interface_operand import IndirectInterfaceOperand
from d7a.alp.operations.indirect_forward import IndirectForward
from d7a.d7anp.addressee import Addressee


class TestIndirectForward(unittest.TestCase):
  def test_byte_generation(self):
    indirect_forward_action = IndirectForward(
      operand=IndirectInterfaceOperand(
        interface_file_id=0x40,
        interface_configuration_overload=None
      )
    )

    bytes = bytearray(indirect_forward_action)
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], 0x40)

  def test_byte_generation_with_overload(self):
    indirect_forward_action = IndirectForward(
      operand=IndirectInterfaceOperand(
        interface_file_id=0x40,
        interface_configuration_overload=Addressee()
      )
    )

    bytes = bytearray(indirect_forward_action)
    self.assertEqual(len(bytes), 3)
    self.assertEqual(bytes[0], 0x40)
