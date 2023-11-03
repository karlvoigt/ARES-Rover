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
# unit tests for the D7A SP QoS Paramters 

import unittest

from bitstring import ConstBitStream

from d7a.sp.qos import QoS, ResponseMode, RetryMode


class TestQoS(unittest.TestCase):
  def test_default_constructor(self):
    qos = QoS()

  def test_byte_generation(self):
    bytes = bytearray(QoS())
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], int('00000000', 2))

    bytes = bytearray(QoS(
      retry_mod = RetryMode.RETRY_MODE_NO,
      resp_mod    = ResponseMode.RESP_MODE_ANY,
      record=True,
      stop_on_err=True,
    ))
    self.assertEqual(len(bytes), 1)
    self.assertEqual(bytes[0], int('11000010', 2))

  def test_parse(self):
    bytes = [
      0b11000010
    ]

    qos = QoS.parse(ConstBitStream(bytes=bytes))

    self.assertEqual(qos.retry_mod, RetryMode.RETRY_MODE_NO)
    self.assertEqual(qos.resp_mod, ResponseMode.RESP_MODE_ANY)
    self.assertEqual(qos.record, True)
    self.assertEqual(qos.stop_on_err, True)

if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestQoS)
  unittest.TextTestRunner(verbosity=2).run(suite)
