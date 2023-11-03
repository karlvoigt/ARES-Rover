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
# class implementation of segment filter options
from enum import Enum

from d7a.support.schema import Validatable, Types

class SegmentFilterOptions(Validatable):
  SCHEMA = [{
    "not_cnt_duty_err": Types.BOOLEAN(),
    "not_cnt_CSMA_CA_err": Types.BOOLEAN(),
    "filter_segments_retry": Types.BOOLEAN(),
    "xoff": Types.BOOLEAN()
  }]
  
  def __init__(self, not_cnt_duty_err=False, not_cnt_CSMA_CA_err=False, filter_segments_retry=False, xoff=False):
    self.not_cnt_duty_err = not_cnt_duty_err
    self.not_cnt_CSMA_CA_err = not_cnt_CSMA_CA_err
    self.filter_segments_retry = filter_segments_retry
    self.xoff = xoff
    super(SegmentFilterOptions, self).__init__()

  def __iter__(self):
    byte = 0
    if self.filter_segments_retry: byte |= 1  # b0
    if self.not_cnt_CSMA_CA_err: byte |= 1 << 1  # b1
    if self.not_cnt_duty_err: byte |= 1 << 2  # b2
    #b3 is rfu
    if self.xoff: byte |= 1 << 4  # b4
    yield byte

  @staticmethod
  def parse(s):
    _rfu = s.read("uint:3")
    xoff = s.read("bool")
    _rfu = s.read("bool")
    not_cnt_duty_err = s.read("bool")
    not_cnt_CSMA_CA_err = s.read("bool")
    filter_segments_retry = s.read("bool")
    return AutoscalingCtrl(not_cnt_duty_err=not_cnt_duty_err, not_cnt_CSMA_CA_err=not_cnt_CSMA_CA_err, filter_segments_retry=filter_segments_retry, xoff=xoff)

  def __str__(self):
    return str(self.as_dict())