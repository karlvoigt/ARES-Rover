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
from d7a.alp.operations.nop import NoOperation
from d7a.alp.operations.operation import Operation
from d7a.support.schema import Types

__author__ = 'glenn'

class StatusActionOperandExtensions(object):
  ACTION_STATUS = 0
  INTERFACE_STATUS = 1
  ALL   = [ ACTION_STATUS, INTERFACE_STATUS ]

  @staticmethod
  def SCHEMA():
    return { "type": "integer", "allowed" : StatusActionOperandExtensions.ALL }


class StatusAction(Action):
  SCHEMA = [{
    "status_operand_extension"    : Types.INTEGER(values=StatusActionOperandExtensions.ALL),
    "operation": Types.OBJECT(Operation),
    "operand"  : Types.OBJECT(nullable=True)  # there is no Operand base-class
  }]

  def __init__(self, status_operand_extension, operation):
    self.status_operand_extension = status_operand_extension
    self.operation = operation
    super(StatusAction, self).__init__(operation=operation)

  def __iter__(self):
    byte = 0
    byte |= self.status_operand_extension << 6
    byte += self.op
    yield byte

    for byte in self.operation: yield byte

  def __str__(self):
    return "{}".format(self.operand)