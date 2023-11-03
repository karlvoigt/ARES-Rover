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

# unit tests for the D7 ALP command byte generation

import unittest
from d7a.alp.operations.status            import InterfaceStatus
from d7a.alp.parser                       import Parser
from d7a.alp.command                      import Command
from d7a.alp.action                       import Action
from d7a.alp.operations.responses         import ReturnFileData
from d7a.alp.operands.file                import Data
from d7a.alp.operands.offset import Offset
from d7a.alp.regular_action import RegularAction
from d7a.alp.status_action import StatusAction, StatusActionOperandExtensions
from d7a.d7anp.addressee import Addressee
from d7a.phy.channel_header import ChannelHeader, ChannelBand, ChannelClass, ChannelCoding
from d7a.phy.channel_id import ChannelID
from d7a.sp.status                        import Status as D7ASpStatus
from d7a.alp.operands.interface_status    import InterfaceStatusOperand
from d7a.types.ct import CT


class TestCommand(unittest.TestCase):
  def setUp(self):
    self.parser = Parser()

  def test_simple_received_return_file_data_command(self):
    cmd = Command(
        generate_tag_request_action=False,
        actions=[
          RegularAction(
            operation=ReturnFileData(
              operand=Data(
                data=bytearray("Hello world", "utf-8"),
                offset=Offset(id=0x51)
              )
            )
          ),
          StatusAction(
            status_operand_extension=StatusActionOperandExtensions.INTERFACE_STATUS,
            operation=InterfaceStatus(
              operand=InterfaceStatusOperand(
                interface_id=0xD7,
                interface_status=D7ASpStatus(
                  channel_id=ChannelID(
                    channel_header=ChannelHeader(channel_band=ChannelBand.BAND_433,
                                               channel_class=ChannelClass.LO_RATE,
                                               channel_coding=ChannelCoding.PN9),
                    channel_index=16
                  ),
                  rx_level=70,
                  link_budget=80,
                  target_rx_level=80,
                  nls=False,
                  missed=False,
                  retry=False,
                  unicast=False,
                  fifo_token=200,
                  seq_nr=0,
                  response_to=CT(mant=20),
                  addressee=Addressee()
                )
              )
            )
          )
        ]
    )
    expected = [
      0x62,                                           # Interface Status action
      0xD7,                                           # D7ASP interface
      32,                                              # channel header
      0, 16,                                          # channel_id
      70,                                             # rxlevel (- dBm)
      80,                                             # link budget
      80,                                             # target rx level
      0,                                              # status
      200,                                            # fifo token
      0,                                              # seq
      20,                                             # response timeout
      0x10,                                           # addressee ctrl (NOID)
      0,                                              # access class
      0x20,                                           # action=32/ReturnFileData
      0x51,                                           # File ID
      0x00,                                           # offset
      0x0b,                                           # length
      0x48, 0x65, 0x6c, 0x6c, 0x6f,                   # Hello
      0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64              # World
    ]
    cmdBytes = [cmdByte for cmdByte in cmd]
    self.assertEqual(cmdBytes, expected)


  def test_simple_received_return_file_data_command_with_tag_request(self):
    cmd = Command(
      tag_id=25,
      actions=[
        RegularAction(
          operation=ReturnFileData(
            operand=Data(
              data=list(bytearray("Hello world", "utf-8")),
              offset=Offset(id=0x51)
            )
          )
        ),
        StatusAction(
          status_operand_extension=StatusActionOperandExtensions.INTERFACE_STATUS,
          operation=InterfaceStatus(
            operand=InterfaceStatusOperand(
              interface_id=0xD7,
              interface_status=D7ASpStatus(
                channel_id=ChannelID(
                  channel_header=ChannelHeader(channel_band=ChannelBand.BAND_433,
                                             channel_class=ChannelClass.LO_RATE,
                                             channel_coding=ChannelCoding.PN9),
                  channel_index=16
                ),
                rx_level=70,
                link_budget=80,
                target_rx_level=80,
                nls=False,
                missed=False,
                retry=False,
                unicast=False,
                fifo_token=200,
                seq_nr=0,
                response_to=CT(mant=20),
                addressee=Addressee()
              )
            )
          )
        )
      ]
    )
    expected = [
      0xB4,   # tag request with send response bit set
      25,     # tag ID
      0x62,  # Interface Status action
      0xD7,  # D7ASP interface
      32,  # channel header
      0, 16,  # channel_id
      70,  # rxlevel (- dBm)
      80,  # link budget
      80,  # target rx level
      0,  # status
      200,  # fifo token
      0,  # seq
      20,  # response timeout
      0x10,  # addressee ctrl (NOID)
      0,     # access class
      0x20,  # action=32/ReturnFileData
      0x51,  # File ID
      0x00,  # offset
      0x0b,  # length
      0x48, 0x65, 0x6c, 0x6c, 0x6f,  # Hello
      0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64  # World
    ]
    cmdBytes = [cmdByte for cmdByte in cmd]
    self.assertEqual(cmdBytes, expected)


if __name__ == '__main__':
  suite = unittest.TestLoader().loadTestsFromTestCase(TestCommand)
  unittest.TextTestRunner(verbosity=2).run(suite)
