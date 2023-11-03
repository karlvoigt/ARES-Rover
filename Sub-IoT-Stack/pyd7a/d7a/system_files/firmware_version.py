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

from d7a.support.schema import Validatable, Types
from d7a.system_files.file import File
from d7a.system_files.system_file_ids import SystemFileIds


class FirmwareVersionFile(File, Validatable):

  SCHEMA = [{
    "d7a_protocol_version_major": Types.INTEGER(min=0, max=255),
    "d7a_protocol_version_minor": Types.INTEGER(min=0, max=255),
    # custom fields, specific to Sub-IoT
    "filesystem_version_major": Types.INTEGER(min=0, max=255),
    "filesystem_version_minor": Types.INTEGER(min=0, max=255),
    "application_name": Types.STRING(maxlength=6),
    "git_sha1": Types.STRING(maxlength=7)
  }]

  def __init__(self, d7a_protocol_version_major=0, d7a_protocol_version_minor=0, filesystem_version_major=0, filesystem_version_minor=0, application_name="", git_sha1=""):
    self.d7a_protocol_version_major = d7a_protocol_version_major
    self.d7a_protocol_version_minor = d7a_protocol_version_minor
    self.filesystem_version_major = filesystem_version_major
    self.filesystem_version_minor = filesystem_version_minor
    self.application_name = application_name
    self.git_sha1 = git_sha1
    Validatable.__init__(self)
    File.__init__(self, SystemFileIds.FIRMWARE_VERSION.value, 17)

  @property
  def d7ap_version(self):
    return str(self.d7a_protocol_version_major) + '.' + str(self.d7a_protocol_version_minor)

  @property
  def filesystem_version(self):
    return str(self.filesystem_version_major) + '.' + str(self.filesystem_version_minor)

  @staticmethod
  def parse(s, offset=0, length=17):
    version_file = FirmwareVersionFile()
    if (offset <= 0) and (length + offset >= 1):
      version_file.d7a_protocol_version_major = s.read("uint:8")
    if (offset <= 1) and (length + offset >= 2):
      version_file.d7a_protocol_version_minor = s.read("uint:8")
    if (offset <= 2) and (length + offset >= 3):
      version_file.filesystem_version_major = s.read("uint:8")
    if (offset <= 3) and (length + offset >= 4):
      version_file.filesystem_version_minor = s.read("uint:8")
    if (offset <= 4) and (length + offset >= 10):
      try:
        version_file.application_name = s.read("bytes:6").decode("ascii")
      except UnicodeDecodeError:
        version_file.application_name = "DecErr"
        pass
    if (offset <= 10) and (length + offset >= 17):
      version_file.git_sha1 = s.read("bytes:7").decode("ascii")
    return version_file

  def __iter__(self):
    yield self.d7a_protocol_version_major
    yield self.d7a_protocol_version_minor

    yield self.filesystem_version_major
    yield self.filesystem_version_minor

    for byte in bytearray(self.application_name.encode("ASCII").ljust(6)):
      yield byte

    for byte in bytearray(self.git_sha1.encode("ASCII").ljust(7)):
      yield byte

  def __str__(self):
    return "d7ap v{}, filesystem v{}, application_name={}, git_sha1={}".format(self.d7ap_version, self.filesystem_version, self.application_name, self.git_sha1)
