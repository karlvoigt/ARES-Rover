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

# unit tests for Compressed Time

import unittest

from d7a.types.ct import CT

class TestCT(unittest.TestCase):
  def test_default_constructor_is_zero(self):
    t = CT()
    self.assertEqual(int(t), 0)

  def test_ct_construction(self):
    for exp in [1, 7]:
      for mant in [1, 31]:
        try:
          t = CT(exp=exp, mant=mant)
        except ValueError:
          self.fail("CT constructor raised ExceptionType unexpectedly " +
                    "for exp={0}, mant={1}".format(exp, mant))

  def test_invalid_ct_constructions(self):
    def bad(args, kwargs): CT(**kwargs)
    self.assertRaises(ValueError, bad, [], { "exp"  : -1 })
    self.assertRaises(ValueError, bad, [], { "mant" : -1 })
    self.assertRaises(ValueError, bad, [], { "exp"  :  8 })
    self.assertRaises(ValueError, bad, [], { "mant" : 32 })

  def test_ct_conversion_to_int(self):
    self.assertEqual(int(CT(1,1)),   4)
    self.assertEqual(int(CT(2,2)),  32)
    self.assertEqual(int(CT(3,3)), 192)

  def test_byte_generation(self):
    self.assertEqual( bytearray(CT(1, 1))[0], int('00100001', 2))
    self.assertEqual( bytearray(CT(7,31))[0], int('11111111', 2))

  def test_compress(self):
    self.assertEqual(1024, CT.compress(1024).decompress())

if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestCT)
  unittest.TextTestRunner(verbosity=2).run(suite)
