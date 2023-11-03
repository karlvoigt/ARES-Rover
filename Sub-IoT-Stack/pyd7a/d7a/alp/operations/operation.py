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

# author: Christophe VG <contact@christophe.vg>
# abstract base-class for operations

from abc import ABCMeta

from d7a.support.schema import Validatable

class Operation(Validatable, metaclass=ABCMeta):
  def __init__(self, operand=None):
    if self.operand_class is None and operand is not None:
      raise ValueError("{0} doesn't require an operand.".format(
                        self.__class__.__name__
                      ))
    if (operand is None and self.operand_class is not None) or \
       (operand is not None and not isinstance(operand, self.operand_class)):
      raise ValueError("{0} requires a {1} operand".format(
                        self.__class__.__name__,
                        self.operand_class.__name__
                      ))
    self.operand = operand

  def __iter__(self):
    if self.operand:
      for byte in self.operand: yield byte

  def __str__(self):
    s = self.__class__.__name__
    s += ": " + str(self.operand)
    return s