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
import argparse

from bitstring import ConstBitStream

from d7a.alp.parser import Parser as AlpParser
from d7a.dll.parser import Parser as DllFrameParser, FrameType
from d7a.serial_modem_interface.parser import Parser as SerialParser
from d7a.system_files.system_file_ids import SystemFileIds
from d7a.system_files.system_files import SystemFiles

parser_types = ["fg", "bg", "alp", "serial", "systemfile"]
argparser = argparse.ArgumentParser()
argparser.add_argument("-t", "--type", choices=parser_types, required=True)
argparser.add_argument("-f", "--file-id", help="the ID of the system file to parse", type=int)
argparser.add_argument('data', help="The data to be parsed, input as an hexstring")
args = argparser.parse_args()

hexstring = args.data.strip().replace(' ', '')
data = bytearray(bytes.fromhex(hexstring))
if args.type == "alp":
  print(AlpParser().parse(ConstBitStream(data), len(data)))
  exit(0)
if args.type == "serial":
  parser = SerialParser()
if args.type == "fg":
  parser = DllFrameParser(frame_type=FrameType.FOREGROUND)
if args.type == "bg":
  parser = DllFrameParser(frame_type=FrameType.BACKGROUND)
if args.type == "systemfile":
  file = SystemFileIds(args.file_id)
  file_type = SystemFiles().files[file]
  print((file_type.parse(ConstBitStream(data))))
  exit(0)

msgtype, cmds, info = parser.parse(data)
for cmd in cmds:
  print(cmd)

print(info)