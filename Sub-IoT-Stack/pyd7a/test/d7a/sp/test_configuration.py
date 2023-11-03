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

# unit tests for the D7A SP (FIFO) Configuration

import unittest

from bitstring import ConstBitStream

from d7a.d7anp.addressee import IdType, NlsMethod

from d7a.types.ct         import CT
from d7a.sp.qos           import QoS, ResponseMode, RetryMode
from d7a.sp.configuration import Configuration

class TestConfiguration(unittest.TestCase):
  def test_default_constructor(self):
    c = Configuration()

  def test_invalid_configuration_construction(self):
    def bad(args, kwargs): Configuration(**kwargs)
    self.assertRaises(ValueError, bad, [], { "qos"       : None  })
    self.assertRaises(ValueError, bad, [], { "addressee" : None  })
    self.assertRaises(ValueError, bad, [], { "dorm_to"   : None  })

  def test_configuration_bad_composed_objects(self):
    def bad(args, kwargs): Configuration(**kwargs)
    self.assertRaises(ValueError, bad, [], { "qos":       CT()  })
    self.assertRaises(ValueError, bad, [], { "dorm_to":   QoS() })
    self.assertRaises(ValueError, bad, [], { "addressee": QoS() })

  def test_byte_generation(self):
    # TODO: use mocking framework to mock sub-objects
    bytes = bytearray(Configuration())
    self.assertEqual(len(bytes), 4)
    self.assertEqual(bytes[0], int( '00000000', 2)) # qos
    self.assertEqual(bytes[1], int( '00000000', 2)) # dorm_to (CT)
    self.assertEqual(bytes[2], int( '00010000', 2)) # addressee control NOID
    self.assertEqual(bytes[3], 0)  # access class

  def test_parse(self):
    bytes = [
      0b00000000,
      0,
      0b00010000,
      0
    ]

    config = Configuration.parse(ConstBitStream(bytes=bytes))

    self.assertEqual(config.qos.resp_mod, ResponseMode.RESP_MODE_NO)
    self.assertEqual(config.qos.retry_mod, RetryMode.RETRY_MODE_NO)
    self.assertEqual(config.qos.stop_on_err, False)
    self.assertEqual(config.qos.record, False)
    self.assertEqual(config.addressee.id_type, IdType.NOID)
    self.assertEqual(config.addressee.nls_method, NlsMethod.NONE)
    self.assertEqual(config.addressee.access_class, 0)

if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestConfiguration)
  unittest.TextTestRunner(verbosity=1).run(suite)
