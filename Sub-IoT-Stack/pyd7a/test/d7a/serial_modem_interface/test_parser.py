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

# unit tests for the serial interface parser

import unittest
from d7a.support.Crc import calculate_crc

from d7a.serial_modem_interface.parser import Parser

class TestParser(unittest.TestCase):
  def setUp(self):
    self.parser = Parser()
    self.counter = 0

  def test_basic_valid_message(self):
    self.counter = self.counter + 1
    alp_cmd_bytes = [
      0x20,                                           # action=32/ReturnFileData
      0x40,                                           # File ID
      0x00,                                           # offset
      0x04,                                           # length
      0x00, 0xf3, 0x00, 0x00                          # data
    ]
    # calculate crc
    crc_array = calculate_crc(bytearray(alp_cmd_bytes))

    # |sync|sync|counter|message type|length|crc1|crc2|
    frame = [
      0xC0,                                           # interface sync byte
      0,                                              # interface version
      self.counter,                                   # counter
      0x01,                                           # message type,
      len(alp_cmd_bytes),                             # ALP cmd length
      crc_array[0], crc_array[1]                      # ALP crc

    ] + alp_cmd_bytes

    (message_type, cmds, info) = self.parser.parse(frame)
    self.assertEqual(cmds[0].actions[0].operation.op, 32)
    self.assertEqual(cmds[0].actions[0].operation.operand.length.value, 4)

  def test_bad_identifier(self):
    (message_type, cmds, info) = self.parser.parse([
      0x0c, # that's 0c not c0 ! ;-)
      0x04, 0x00, 0x00, 0x00,
      0x20,
      0x24, 0x8a, 0xb6, 0x00, 0x52, 0x0b, 0x35, 0x2c,
      0x20,
      0x40,
      0x00,
      0x00
    ])
    self.assertEqual(len(cmds), 0)
    self.assertEqual(len(info["errors"]), 1)

  def test_buffer_skipping(self):
    self.parser.buffer = bytearray([ 0x10, 0x20, 0x30, 0xc0, 0x10, 0x20, 0x30 ])
    skipped = self.parser.skip_bad_buffer_content()
    self.assertEqual(skipped, 3)
    self.assertEqual(self.parser.buffer, bytearray([ 0xc0, 0x10, 0x20, 0x30 ]))

  def test_entire_buffer_skipping(self):
    self.parser.buffer = [ 0x10, 0x20, 0x30, 0x10, 0x20, 0x30 ]
    skipped = self.parser.skip_bad_buffer_content()
    self.assertEqual(skipped, 6)
    self.assertEqual(self.parser.buffer, bytearray())

  def test_empty_buffer_skipping(self):
    self.parser.buffer = []
    skipped = self.parser.skip_bad_buffer_content()
    self.assertEqual(skipped, 0)
    self.assertEqual(self.parser.buffer, [])

  def test_buffer_skipping_with_first_item_the_id(self):
    self.parser.buffer = [ 0xc0, 0x10, 0x20, 0x30 ]
    skipped = self.parser.skip_bad_buffer_content()
    self.assertEqual(skipped, 4)
    self.assertEqual(self.parser.buffer, bytearray())

  def test_buffer_skipping_with_first_and_second_item_the_id(self):
    self.parser.buffer = bytearray([ 0xc0, 0xc0, 0x10, 0x20, 0x30 ])
    skipped = self.parser.skip_bad_buffer_content()
    self.assertEqual(skipped, 1)
    self.assertEqual(self.parser.buffer, bytearray([ 0xc0, 0x10, 0x20, 0x30 ]))

  # |sync|sync|counter|message type|length|crc1|crc2|
  def test_bad_identifier_with_identifier_in_body(self):
    self.counter = self.counter + 1
    (message_type, cmds, info) = self.parser.parse(bytearray([
      0x0c, # that's 0c not c0 ! ;-)
      0x04, 0x00, 0x00, 0x00,
      0x20,
      0x24, 0x8a,
      0xc0, 0x00,                       # here's another one
      self.counter, 0X01, 7, 0x33, 0xAD,        # calculated crc beforehand
      0x0b, 0x35, 0x2c,
      0x7d,
      0x40,
      0x00,
      0x00
    ]))
    self.assertEqual(len(cmds), 0)
    self.assertEqual(len(info["errors"]), 2)
    self.assertEqual(info["errors"][0]["skipped"], 8)

  def test_partial_command(self):
    self.counter = self.counter + 1
    alp_action_bytes = bytearray([
      0x20,                                           # action=32/ReturnFileData
      0x40,                                           # File ID
      0x00,                                           # offset
      0x04,                                           # length
      0x00, 0xf3, 0x00, 0x00                          # data
    ])

    (message_type, cmds, info) = self.parser.parse(bytearray([
      0xc0,                                           # interface start
      0,
      self.counter, 0x01, 2 * len(alp_action_bytes), 0xDC, 0xF8  # expect 2 ALP actions but only one in buffer
    ]) + alp_action_bytes)
    self.assertEqual(len(cmds), 0)
    self.assertEqual(len(info["errors"]), 0)
    self.assertEqual(info["parsed"], 0)

  def test_continue_partial_command(self): # ?
    self.test_partial_command() # incomplete command, add second ALP action to complete it ...
    (message_type, cmds, info) = self.parser.parse(bytearray([
      0x20,                                           # action=32/ReturnFileData
      0x40,                                           # File ID
      0x00,                                           # offset
      0x04,                                           # length
      0x00, 0xf3, 0x00, 0x00                          # data
    ]))
    self.assertEqual(len(info["errors"]), 0)
    self.assertEqual(len(cmds[0].actions), 2)


  def test_continue_from_bad_buffer(self):
    self.test_bad_identifier_with_identifier_in_body() # buffer is bad now
    self.test_basic_valid_message()                    # cont. with valid msg


  def test_continue_partial_second_frame(self):
    self.counter = self.counter + 1
    alp_cmd_bytes = [
      0x20,                                           # action=32/ReturnFileData
      0x40,                                           # File ID
      0x00,                                           # offset
      0x04,                                           # length
      0x00, 0xf3, 0x00, 0x00                          # data
    ]

    frame = [
      0xC0,                                           # interface sync byte
      0,                                              # interface version
      self.counter, 0x01, len(alp_cmd_bytes), 0xA4, 0xBE,     # calculated crc beforehand
    ] + alp_cmd_bytes + [
      # second, partial frame
      0xC0,                                           # interface sync byte
      0,                                              # interface version
      self.counter + 1, 0x01, len(alp_cmd_bytes), 0xA4, 0xBE,     # calculated crc beforehand
    ]
    self.counter = self.counter + 1
    # first frame should parse
    (message_type, cmds, info) = self.parser.parse(frame)
    self.assertEqual(cmds[0].actions[0].operation.op, 32)
    self.assertEqual(cmds[0].actions[0].operation.operand.length.value, 4)

    # and now complete the second frame and check this is parsed as well
    (message_type, cmds, info) = self.parser.parse(alp_cmd_bytes) # the missing bytes only, without the frame header
    self.assertEqual(cmds[0].actions[0].operation.op, 32)
    self.assertEqual(cmds[0].actions[0].operation.operand.length.value, 4)


if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestParser)
  unittest.TextTestRunner(verbosity=2).run(suite)
