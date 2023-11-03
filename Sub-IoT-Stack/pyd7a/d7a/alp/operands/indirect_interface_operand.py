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
from d7a.alp.interface import InterfaceType
from d7a.d7anp.addressee import Addressee
from d7a.sp.configuration import Configuration
from d7a.support.schema import Validatable, Types


class IndirectInterfaceOperand(Validatable):

  SCHEMA = [{
    "interface_file_id"        : Types.INTEGER(min=0, max=0xFF),
    "interface_configuration_overload"    : Types.OBJECT(Addressee, nullable=True) # TODO assuming D7ASP interface
  }]

  def __init__(self, interface_file_id, interface_configuration_overload=None):
    self.interface_file_id = interface_file_id
    self.interface_configuration_overload = interface_configuration_overload
    super(IndirectInterfaceOperand, self).__init__()

  def __iter__(self):
    yield self.interface_file_id
    if self.interface_configuration_overload is not None:
      for byte in self.interface_configuration_overload: yield byte

  def __str__(self):
    return "interface-file-id={}, configuration-overload={}".format(self.interface_file_id, self.interface_configuration_overload)
