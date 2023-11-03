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

from d7a.fs.file_permissions import FilePermissions
from d7a.fs.file_properties import FileProperties
from d7a.support.schema import Validatable, Types


class FileHeader(Validatable):
  SCHEMA = [{
    "permissions": Types.OBJECT(FilePermissions),
    "properties": Types.OBJECT(FileProperties),
    "alp_command_file_id": Types.BYTE(),
    "interface_file_id": Types.BYTE(),
    "file_size": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "allocated_size": Types.INTEGER(min=0, max=0xFFFFFFFF)
  }]

  def __init__(self, permissions, properties, alp_command_file_id, interface_file_id, file_size, allocated_size):
    self.permissions = permissions
    self.properties = properties
    self.alp_command_file_id = alp_command_file_id
    self.interface_file_id = interface_file_id
    self.file_size = file_size
    self.allocated_size = allocated_size
    Validatable.__init__(self)

  @staticmethod
  def parse(s):
    permissions = FilePermissions.parse(s)
    properties = FileProperties.parse(s)
    alp_command_file_id = s.read("uint:8")
    interface_file_id = s.read("uint:8")
    file_size = s.read("uint:32")
    allocated_size = s.read("uint:32")
    return FileHeader(permissions, properties, alp_command_file_id, interface_file_id, file_size, allocated_size)

  def __iter__(self):
    for byte in self.permissions:
      yield byte

    for byte in self.properties:
      yield byte

    yield self.alp_command_file_id
    yield self.interface_file_id
    for byte in bytearray(struct.pack(">I", self.file_size)):
      yield byte

    for byte in bytearray(struct.pack(">I", self.allocated_size)):
      yield byte

  def __eq__(self, other):
    if isinstance(other, FileHeader):
      return self.__dict__ == other.__dict__

    return False

  def __ne__(self, other):
    return not self.__eq__(other)

  def __str__(self):
    return "permissions={}, properties=({}), alp_command_file_id={}, interface_file_id={}, file_size={}, allocated_size={}".format(
      self.permissions, self.properties, self.alp_command_file_id, self.interface_file_id, self.file_size, self.allocated_size
    )