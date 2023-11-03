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
# class implementations of File {*} Operands
from d7a.alp.operands.length import Length
from d7a.alp.operands.offset import Offset
from d7a.support.schema import Validatable, Types


class Data(Validatable):

  SCHEMA = [{
    "offset" : Types.OBJECT(Offset),
    "length" : Types.OBJECT(Length),
    "data"   : Types.BYTES()
  }]
  
  def __init__(self, data=[], offset=Offset()):
    self.offset = offset
    self.data   = data
    self.length = Length(len(data))
    super(Data, self).__init__()

  # for consistency with schema, e.g. if using generic attribute conversion, etc
  # @property
  # def length(self):
  #   return len(self.data)

  # the Python way ;-)
  def __len__(self):
    return self.length.value

  def __iter__(self):
    for byte in self.offset: yield byte
    for byte in self.length: yield byte
    for byte in self.data: yield byte

  def __str__(self):
    return "{}, length={}, data={}".format(self.offset, self.length, self.data)


class DataRequest(Validatable):

  SCHEMA = [{
    "offset" : Types.OBJECT(Offset),
    "length" : Types.OBJECT(Length)
  }]

  def __init__(self, length=Length(), offset=Offset()):
    self.offset = offset
    self.length = length
    super(DataRequest, self).__init__()

  def __iter__(self):
    for byte in self.offset: yield byte
    for byte in self.length: yield byte

  def __str__(self):
    return "{}, length={}".format(self.offset, self.length)


class FileIdOperand(Validatable):

  SCHEMA = [{
    "file_id": Types.BYTE()
  }]

  def __init__(self, file_id):
    self.file_id = file_id
    super(FileIdOperand, self).__init__()

  def __iter__(self):
    yield self.file_id

  def __str__(self):
    return "file-id={}".format(self.file_id)