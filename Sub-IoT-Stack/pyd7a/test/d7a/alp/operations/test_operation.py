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
# unit tests for the Operation base class

import unittest

from d7a.alp.operations.operation import Operation

class OperandX(object): pass
class OperandY(object): pass

def make_operation(operand=None, op=None):
  class MyOperation(Operation):
    def __init__(self, *args, **kwargs):
      self.operand_class = operand
      self.op     = op
      super(MyOperation, self).__init__(*args, **kwargs)
  return MyOperation

class TestOperation(unittest.TestCase):
  
  def test_not_implemented_operand(self):
    def bad(): Operation()
    self.assertRaises(AttributeError, bad)

  def test_missing_operand(self):
    MyOperation = make_operation(operand=OperandX)
    def bad(): MyOperation()
    self.assertRaises(ValueError, bad)

  def test_unexpected_operand(self):
    MyOperation = make_operation()
    def bad(): MyOperation(OperandY())
    self.assertRaises(ValueError, bad)
    

  def test_incorrect_operand(self):
    MyOperation = make_operation(operand=OperandX)
    def bad(): MyOperation(OperandY())
    self.assertRaises(ValueError, bad)

  def test_op_property(self):
    MyOperation = make_operation(operand=None, op=123)
    op = MyOperation()
    self.assertEqual(op.op, 123)

if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestOperation)
  unittest.TextTestRunner(verbosity=2).run(suite)
