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
from d7a.support.schema import Validatable, Types


class FilePermissions(Validatable):
  SCHEMA = [{
    "encrypted": Types.BOOLEAN(),
    "executable": Types.BOOLEAN(),
    "user_readable": Types.BOOLEAN(),
    "user_writable": Types.BOOLEAN(),
    "user_executable": Types.BOOLEAN(),
    "guest_readable": Types.BOOLEAN(),
    "guest_writable": Types.BOOLEAN(),
    "guest_executable": Types.BOOLEAN()
  }]

  def __init__(self, encrypted=False, executable=False, user_readable=True, user_writable=True, user_executable=True,
               guest_readable= True, guest_writable=True, guest_executable=True):
    self.encrypted = encrypted
    self.executable = executable
    self.user_readable = user_readable
    self.user_writable = user_writable
    self.user_executable = user_executable
    self.guest_readable = guest_readable
    self.guest_writable = guest_writable
    self.guest_executable = guest_executable

    Validatable.__init__(self)

  @staticmethod
  def parse(s):
    encrypted = s.read("bool")
    executable = s.read("bool")
    user_readable = s.read("bool")
    user_writable = s.read("bool")
    user_executable = s.read("bool")
    guest_readable = s.read("bool")
    guest_writable = s.read("bool")
    guest_executable = s.read("bool")
    return FilePermissions(encrypted=encrypted, executable=executable, user_readable=user_readable,
                      user_writable=user_writable, user_executable=user_executable,
                      guest_readable=guest_readable, guest_writable=guest_writable, guest_executable=guest_executable)

  def __iter__(self):
    byte = 0
    if self.encrypted: byte += 1 << 7
    if self.executable: byte += 1 << 6
    if self.user_readable: byte += 1 << 5
    if self.user_writable: byte += 1 << 4
    if self.user_executable: byte += 1 << 3
    if self.guest_readable: byte += 1 << 2
    if self.guest_writable: byte += 1 << 1
    if self.guest_executable: byte += 1
    yield byte

  def __str__(self):
    return "" #TODO

  def __eq__(self, other):
    if isinstance(other, FilePermissions):
      return self.__dict__ == other.__dict__

    return False

  def __ne__(self, other):
    return not self.__eq__(other)
