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
from d7a.alp.action import Action
from d7a.alp.operands.interface_configuration import InterfaceConfiguration
from d7a.alp.operations.forward import Forward
from d7a.alp.operations.nop import NoOperation
from d7a.support.schema import Types


class ForwardAction(Action):
  SCHEMA = [{
    "resp"     : Types.BOOLEAN(),
    "op"       : Types.BITS(6),
    "operation": Types.OBJECT(Forward),
    "operand"  : Types.OBJECT(InterfaceConfiguration)  # TODO for now only D7 interface is supported
  }]

  def __init__(self, resp=False, operation=NoOperation()):
    self.resp      = resp
    super(ForwardAction, self).__init__(operation)

  def __iter__(self):
    byte = 0
    if self.group: byte |= 1 << 7
    if self.resp:  byte |= 1 << 6
    byte += self.op
    yield byte

    for byte in self.operation: yield byte
