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
from d7a.sp.status import Status
from d7a.support.schema import Validatable, Types


class InterfaceStatusOperand(Validatable):

  SCHEMA = [{
    "interface_id"        : Types.BYTE(),
    "interface_status"    : Types.OBJECT(Status, nullable=True)
  }]

  def __init__(self, interface_id, interface_status):
    self.interface_id = interface_id
    self.interface_status   = interface_status
    super(InterfaceStatusOperand, self).__init__()

  def __iter__(self):
    yield self.interface_id
    for byte in self.interface_status: yield byte

  def __str__(self):
    return "interface-id={}, status={}".format(self.interface_id, self.interface_status)
