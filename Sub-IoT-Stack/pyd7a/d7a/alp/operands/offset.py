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

from d7a.alp.operands.length import Length
from d7a.support.schema import Validatable, Types


class Offset(Validatable):

  SCHEMA = [
    {
      "id"    : Types.BYTE(),
      "offset": Types.OBJECT(Length)
    }
  ]

  def __init__(self, id=0, offset=Length()):
    self.id     = id
    self.offset = offset
    super(Offset, self).__init__()

  @staticmethod
  def parse(s):
    id = s.read("uint:8")
    offset = Length.parse(s)
    return Offset(id=id, offset=offset)

  def __iter__(self):
    yield self.id
    for byte in self.offset: yield byte

  def __str__(self):
    return "file-id={}, offset={}".format(self.id, self.offset)