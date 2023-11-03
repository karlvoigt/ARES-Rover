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


class UidFile(File, Validatable):
  SCHEMA = [{
    "uid": Types.INTEGER(min=0, max=0xFFFFFFFFFFFFFFFF)
  }]


  def __init__(self, uid=0):
    self.uid = uid
    File.__init__(self, SystemFileIds.UID.value, 8)
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=8):
    uid = s.read("uint:64")
    return UidFile(uid=uid)

  def __iter__(self):
    for byte in bytearray(struct.pack(">Q", self.uid)):
      yield byte


  def __str__(self):
    return "uid={}".format(hex(self.uid))