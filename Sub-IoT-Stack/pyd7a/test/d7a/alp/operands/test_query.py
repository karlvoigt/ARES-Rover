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

from d7a.alp.operands.length import Length
from d7a.alp.operands.offset import Offset
from d7a.alp.operands.query import QueryOperand, QueryType, ArithComparisonType, ArithQueryParams


class TestQuery(unittest.TestCase):
  def test_arith_comp_with_value_byte_generation(self):
    query = QueryOperand(
      type=QueryType.ARITH_COMP_WITH_VALUE,
      mask_present=False,
      params=ArithQueryParams(signed_data_type=False, comp_type=ArithComparisonType.GREATER_THAN),
      compare_length = Length(1),
      compare_value=[25],
      file_a_offset=Offset(id=32, offset=Length(1))
    )

    bytes = bytearray(query)
    self.assertEqual(len(bytes), 5)
    self.assertEqual(bytes[0], 0x44)
    self.assertEqual(bytes[1], 0x01)
    self.assertEqual(bytes[2], 25)
    self.assertEqual(bytes[3], 0x20)
    self.assertEqual(bytes[4], 0x01)


  def test_arith_comp_with_value_parsing(self):
    bytes = [
      0x44, # arith comp with value, no mask, unsigned, >
      0x01, # compare length
      25, # compare value
      0x20, 0x01 # file offset
    ]

    query = QueryOperand.parse(ConstBitStream(bytes=bytes))
    self.assertEqual(query.type, QueryType.ARITH_COMP_WITH_VALUE)
    self.assertEqual(query.compare_length, 1)
    self.assertEqual(query.mask_present, False)
    self.assertEqual(query.params.signed_data_type, False)
    self.assertEqual(query.params.comp_type, ArithComparisonType.GREATER_THAN)
    self.assertEqual(query.compare_value, [25])
    self.assertEqual(query.file_a_offset.id, 32)
    self.assertEqual(query.file_a_offset.offset.value, 1)
