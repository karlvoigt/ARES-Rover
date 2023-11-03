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
import struct

from d7a.support.schema import Validatable, Types
from d7a.system_files.file import File
from d7a.system_files.system_file_ids import SystemFileIds


class VidFile(File, Validatable):
  SCHEMA = [{
    "vid": Types.INTEGER(min=0, max=0xFFFF),
    "control": Types.INTEGER(min=0, max=0xFF)
  }]


  def __init__(self, vid=0xFFFF, control=0x00):
    self.vid = vid
    self.control = control
    File.__init__(self, SystemFileIds.VID.value, 3)
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=3):
    vid = s.read("uint:16")
    control = s.read("uint:8")
    return VidFile(vid=vid, control=control)

  def __iter__(self):
    for byte in bytearray(struct.pack(">H", self.vid)):
      yield byte
    yield self.control


  def __str__(self):
    return "vid={}, control={}".format(hex(self.vid), hex(self.control))
