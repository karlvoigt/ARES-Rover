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

# author: Liam Oorts <liam.oorts@aloxy.io>
# class implementation of autoscaling control
from enum import Enum

from d7a.support.schema import Validatable, Types

class AutoscalingCtrl(Validatable):
  SCHEMA = [{
    "toggle": Types.BOOLEAN(),
    "rxlev" : Types.BOOLEAN(),
    "on"     : Types.BOOLEAN()
  }]
  
  def __init__(self, toggle=False, rxlev=False, on=False):
    self.toggle = toggle
    self.rxlev = rxlev
    self.on = on
    super(AutoscalingCtrl, self).__init__()

  def __iter__(self):
    byte = 0
    if self.on: byte |= 1  # b0
    if self.rxlev: byte |= 1 << 1  # b1
    if self.toggle: byte |= 1 << 2  # b2
    yield byte

  @staticmethod
  def parse(s):
    _rfu = s.read("uint:5")
    toggle = s.read("bool")
    rxlev = s.read("bool")
    on = s.read("bool")
    return AutoscalingCtrl(toggle=toggle, rxlev=rxlev, on=on)

  def __str__(self):
    return str(self.as_dict())