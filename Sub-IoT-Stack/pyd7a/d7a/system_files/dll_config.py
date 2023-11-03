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


class DllConfigFile(File, Validatable):
  SCHEMA = [{
    "active_access_class": Types.INTEGER(min=0, max=0xFF),
    "LQ_filter": Types.INTEGER(min=0, max=0xFF),
    "NF_CTRL": Types.INTEGER(min=0, max=0xFF),
    "RX_NF_method_parameter": Types.INTEGER(min=0, max=0xFF),
    "TX_NF_method_parameter": Types.INTEGER(min=0, max=0xFF)
  }]

  def __init__(self, active_access_class=0, lq_filter=0x00, nf_ctrl=0x00, rx_nf_method_parameter=0x00
               , tx_nf_method_parameter=0x00):
    self.active_access_class = active_access_class
    self.lq_filter = lq_filter
    self.nf_ctrl = nf_ctrl
    self.rx_nf_method_parameter = rx_nf_method_parameter
    self.tx_nf_method_parameter = tx_nf_method_parameter
    File.__init__(self, SystemFileIds.DLL_CONFIG.value, 7)
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=7):
    ac = s.read("uint:8")
    _rfu = s.read("uint:16")
    lq_filter = s.read("uint:8")
    nf_ctrl = s.read("uint:8")
    rx_nf_method_parameter = s.read("uint:8")
    tx_nf_method_parameter = s.read("uint:8")
    return DllConfigFile(active_access_class=ac, lq_filter=lq_filter, nf_ctrl=nf_ctrl,
                         rx_nf_method_parameter=rx_nf_method_parameter, tx_nf_method_parameter=tx_nf_method_parameter)

  def __iter__(self):
    yield self.active_access_class
    for byte in bytearray(struct.pack(">H", 0)):  # RFU
      yield byte
    yield self.lq_filter
    yield self.nf_ctrl
    yield self.rx_nf_method_parameter
    yield self.tx_nf_method_parameter


  def __str__(self):
    return "active_access_class={}, lq_filter={}, nf_ctrl={}, rx_nf_method_parameter, tx_nf_method_parameter".format(self.active_access_class, self.lq_filter, self.nf_ctrl, self.rx_nf_method_parameter, self.tx_nf_method_parameter)
