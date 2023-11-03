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
import pprint

from d7a.support.schema import Validatable, Types
from d7a.types.ct import CT


class SubProfile(Validatable):

  SCHEMA = [{
    "subband_bitmap": Types.BYTE(),
    "scan_automation_period": Types.OBJECT(CT)
  }]

  def __init__(self, subband_bitmap=0, scan_automation_period=CT()):
    self.subband_bitmap = subband_bitmap
    self.scan_automation_period = scan_automation_period
    super(SubProfile, self).__init__()

  @staticmethod
  def parse(s):
    subband_bitmap = s.read("uint:8")
    scan_automation_period = CT.parse(s)
    return SubProfile(subband_bitmap=subband_bitmap, scan_automation_period=scan_automation_period)

  def __iter__(self):
    yield self.subband_bitmap
    for byte in self.scan_automation_period: yield byte

  def __str__(self):
    return pprint.PrettyPrinter().pformat(self.as_dict())