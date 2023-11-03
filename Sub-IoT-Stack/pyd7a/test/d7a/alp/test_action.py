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

# author: Christophe VG <contact@christophe.vg>
# unit tests for the D7 ALP Action

import unittest

from d7a.alp.action         import Action
from d7a.alp.regular_action import RegularAction


class TestAction(unittest.TestCase):
  def test_default_action_constructor(self):
    Action()
  
  def test_action_construction_switches(self):
    RegularAction(group=True, resp=True)

  def test_default_nop_action_operand(self):
    a = Action()
    self.assertEqual(a.op, 0)

  def test_action_bad_operation(self):
    def bad(): Action(Action())
    self.assertRaises(ValueError, bad)

  def test_byte_generation(self):
    bytes = bytearray(RegularAction())
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], int('00000000', 2))

    bytes = bytearray(RegularAction(group=True, resp=True))
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], int('11000000', 2))

    # TODO: use mocking to create operations

if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestAction)
  unittest.TextTestRunner(verbosity=2).run(suite)
