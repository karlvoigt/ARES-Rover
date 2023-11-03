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
# unit tests for the D7 ALP NoOperation

import unittest

from d7a.alp.operations.nop import NoOperation

class TestNoOperation(unittest.TestCase):
  def test_constructor_and_op_code(self):
    nop = NoOperation()
    self.assertEqual(nop.op, 0)
    self.assertIsNone(nop.operand)

  def test_byte_generation(self):
    bytes = bytearray(NoOperation())
    self.assertEqual(len(bytes), 0)

if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestNoOperation)
  unittest.TextTestRunner(verbosity=2).run(suite)
