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
from enum import Enum

from d7a.support.schema import Validatable, Types


class ActionCondition(Enum):
  LIST = 0
  READ = 1
  WRITE = 2
  WRITE_FLUSH = 3


class StorageClass(Enum):
  TRANSIENT = 0
  VOLATILE = 1
  RESTORABLE = 2
  PERMANENT = 3


class FileProperties(Validatable):
  SCHEMA = [{
    "act_enabled": Types.BOOLEAN(),
    "act_cond": Types.ENUM(ActionCondition),
    "storage_class": Types.ENUM(StorageClass),
  }]

  def __init__(self, act_enabled, act_condition, storage_class):
    self.act_enabled = act_enabled
    self.act_condition = act_condition
    self.storage_class = storage_class

    Validatable.__init__(self)

  @staticmethod
  def parse(s):
    act_enabled = s.read("bool")
    act_condition = ActionCondition(s.read("uint:3"))
    _rfu = s.read("uint:2")
    storage_class = StorageClass(s.read("uint:2"))
    return FileProperties(act_enabled, act_condition, storage_class)

  def __iter__(self):
    byte = 0
    if self.act_enabled: byte += 1 << 7
    byte += self.act_condition.value << 4
    byte += self.storage_class.value
    yield byte

  def __str__(self):
    return "act_enabled={}, act_condition={}, storage_class={}".format(
      self.act_enabled,
      self.act_condition,
      self.storage_class
    )

  def __eq__(self, other):
    if isinstance(other, FileProperties):
      return self.__dict__ == other.__dict__

    return False

  def __ne__(self, other):
    return not self.__eq__(other)
