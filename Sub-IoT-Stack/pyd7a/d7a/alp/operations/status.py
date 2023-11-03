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
from d7a.alp.operands.interface_status import InterfaceStatusOperand
from d7a.alp.operations.operation import Operation


class InterfaceStatus(Operation):
  def __init__(self, operand):
    self.op     = 34 # NOTE: 34 is shared with different status types depending on status operand extension bits
    self.operand_class = InterfaceStatusOperand
    super(InterfaceStatus, self).__init__(operand=operand)