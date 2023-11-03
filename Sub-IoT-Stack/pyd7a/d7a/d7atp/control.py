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

class Control(Validatable):

  SCHEMA = [{
    "is_dialog_start": Types.BOOLEAN(),
    "has_tl": Types.BOOLEAN(),
    "has_te": Types.BOOLEAN(),
    "is_ack_requested": Types.BOOLEAN(),
    "is_ack_not_void": Types.BOOLEAN(),
    "is_ack_record_requested": Types.BOOLEAN(),
    "has_agc": Types.BOOLEAN()
  }]

  def __init__(self, is_dialog_start, has_tl, has_te, is_ack_requested, is_ack_not_void, is_ack_record_requested, has_agc):
    self.is_dialog_start = is_dialog_start
    self.has_tl = has_tl
    self.has_te = has_te
    self.is_ack_requested = is_ack_requested
    self.is_ack_not_void = is_ack_not_void
    self.is_ack_record_requested = is_ack_record_requested
    self.has_agc = has_agc
    super(Control, self).__init__()

  @staticmethod
  def parse(s):
    is_dialog_start = s.read("bool")
    _ = s.read("bool") # RFU
    has_tl = s.read("bool")
    has_te = s.read("bool")
    is_ack_requested = s.read("bool")
    is_ack_not_void = s.read("bool")
    is_ack_record_requested = s.read("bool")
    has_agc = s.read("bool")

    return Control(
      is_dialog_start=is_dialog_start,
      has_tl=has_tl,
      has_te=has_te,
      is_ack_requested=is_ack_requested,
      is_ack_not_void=is_ack_not_void,
      is_ack_record_requested=is_ack_record_requested,
      has_agc=has_agc
    )

  def __iter__(self):
    byte = 0
    if self.is_dialog_start: byte |= 1 << 7
    if self.has_tl:  byte |= 1 << 5
    if self.has_te:  byte |= 1 << 4
    if self.is_ack_requested:  byte |= 1 << 3
    if self.is_ack_not_void:  byte |= 1 << 2
    if self.is_ack_record_requested: byte |= 1 << 1
    byte += self.has_agc
    yield byte