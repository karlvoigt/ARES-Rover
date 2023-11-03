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
from d7a.phy.channel_status_identifier import ChannelStatusIdentifier
from d7a.sp.autoscaling_ctrl import AutoscalingCtrl
from d7a.sp.segment_filter_options import SegmentFilterOptions

class SELConfigFile(File, Validatable):
  SCHEMA = [{
    "autoscaling_ctrl": Types.OBJECT(AutoscalingCtrl),
    "autoscaling_parameter": Types.INTEGER(min=0, max=0xFF),
    "retry": Types.INTEGER(min=0, max=0xFF),
    "repeat": Types.INTEGER(min=0, max=0xFF),
    "ACK_WIN": Types.INTEGER(min=0, max=0xFF),
    "segment_filter_options": Types.OBJECT(SegmentFilterOptions)
  }]

  def __init__(self, autoscaling_ctrl=AutoscalingCtrl(), autoscaling_parameter=0, retry=0, repeat=0, ACK_WIN=0, segment_filter_options=SegmentFilterOptions()):
    self.autoscaling_ctrl = autoscaling_ctrl
    self.autoscaling_parameter = autoscaling_parameter
    self.retry = retry
    self.repeat = repeat
    self.ACK_WIN = ACK_WIN
    self.segment_filter_options = segment_filter_options
    File.__init__(self, SystemFileIds.SEL_CONFIG.value, 6)
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=6):
    autoscaling_ctrl = AutoscalingCtrl.parse(s)
    autoscaling_parameter = s.read("uint:8")
    retry = s.read("uint:8")
    repeat = s.read("uint:8")
    ACK_WIN = s.read("uint:8")
    segment_filter_options = SegmentFilterOptions.parse(s)

    return SELConfigFile(autoscaling_ctrl=autoscaling_ctrl, autoscaling_parameter=autoscaling_parameter, retry=retry,
                         repeat=repeat, ACK_WIN=ACK_WIN, segment_filter_options=segment_filter_options)

  def __iter__(self):
    for byte in self.autoscaling_ctrl:
      yield byte
    yield self.autoscaling_parameter
    yield self.retry
    yield self.repeat
    yield self.ACK_WIN
    for byte in self.segment_filter_options:
      yield byte

  def __str__(self):
    return "autoscaling_ctrl={}, autoscaling_parameter={}, retry={}, repeat={}, ACK_WIN={}, segment_filter_options={}".format(self.autoscaling_ctrl, self.autoscaling_parameter, self.retry, self.repeat, self.ACK_WIN, self.segment_filter_options)
