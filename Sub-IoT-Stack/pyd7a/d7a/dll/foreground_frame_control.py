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
from d7a.d7anp.addressee import IdType
from d7a.support.schema import Validatable, Types

class ForegroundFrameControl(Validatable):

  SCHEMA = [{
    "id_type": Types.ENUM(IdType),
    "eirp_index": Types.INTEGER(None, 0, 63)
  }]

  def __init__(self, id_type, eirp_index=0):
    self.id_type = id_type
    self.eirp_index = eirp_index
    super(ForegroundFrameControl, self).__init__()

  @staticmethod
  def parse(s):
    id_type = IdType(s.read("uint:2"))
    eirp_index = s.read("uint:6")
    return ForegroundFrameControl(
      id_type=id_type,
      eirp_index=eirp_index
    )

  def __iter__(self):
    byte = 0
    byte |= self.id_type.value << 6
    byte += self.eirp_index
    yield byte
