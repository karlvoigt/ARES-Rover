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

from math import log, ceil

class FactorySettingsFile(File, Validatable):

  SCHEMA = [{
    "gain": Types.INTEGER(min=-128, max=127),
    "rx_bw_low_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "rx_bw_normal_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "rx_bw_high_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "bitrate_lo_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "fdev_lo_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "bitrate_normal_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "fdev_normal_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "bitrate_hi_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "fdev_hi_rate": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "preamble_size_lo_rate": Types.INTEGER(min=0, max=255),
    "preamble_size_normal_rate": Types.INTEGER(min=0, max=255),
    "preamble_size_hi_rate": Types.INTEGER(min=0, max=255),
    "preamble_detector_size_lo_rate": Types.INTEGER(min=0, max=255),
    "preamble_detector_size_normal_rate": Types.INTEGER(min=0, max=255),
    "preamble_detector_size_hi_rate": Types.INTEGER(min=0, max=255),
    "preamble_tol": Types.INTEGER(min=0, max=255),
    "rssi_smoothing": Types.INTEGER(min=0, max=255),
    "rssi_offset": Types.INTEGER(min=-126, max=125),
    "lora_bw": Types.INTEGER(min=0, max=0xFFFFFFFF),
    "lora_SF": Types.INTEGER(min=6, max=12),
    "gaussian" : Types.INTEGER(min=0, max=3),
    "paramp" : Types.INTEGER(min=0, max=0xFFFF)
  }]

  def __init__(self, gain=0, rx_bw_low_rate=10468, rx_bw_normal_rate=78646, rx_bw_high_rate=125868,
               bitrate_lo_rate=9600, fdev_lo_rate=4800,
               bitrate_normal_rate=55555, fdev_normal_rate=50000, bitrate_hi_rate=166667, fdev_hi_rate=41667,
               preamble_size_lo_rate=5, preamble_size_normal_rate=5, preamble_size_hi_rate=7,
               preamble_detector_size_lo_rate=3, preamble_detector_size_normal_rate=3, preamble_detector_size_hi_rate=3,
               preamble_tol_lo_rate=15, preamble_tol_normal_rate=10, preamble_tol_hi_rate=10,
               rssi_smoothing=8, rssi_offset=0,
               lora_bw=125000, lora_SF=9,
               gaussian=2, paramp=40):
    self.gain = gain
    self.rx_bw_low_rate = rx_bw_low_rate
    self.rx_bw_normal_rate = rx_bw_normal_rate
    self.rx_bw_high_rate = rx_bw_high_rate
    self.bitrate_lo_rate = bitrate_lo_rate
    self.fdev_lo_rate = fdev_lo_rate
    self.bitrate_normal_rate = bitrate_normal_rate
    self.fdev_normal_rate = fdev_normal_rate
    self.bitrate_hi_rate = bitrate_hi_rate
    self.fdev_hi_rate = fdev_hi_rate
    self.preamble_size_lo_rate = preamble_size_lo_rate
    self.preamble_size_normal_rate = preamble_size_normal_rate
    self.preamble_size_hi_rate = preamble_size_hi_rate
    self.preamble_detector_size_lo_rate = preamble_detector_size_lo_rate
    self.preamble_detector_size_normal_rate = preamble_detector_size_normal_rate
    self.preamble_detector_size_hi_rate = preamble_detector_size_hi_rate
    self.preamble_tol_lo_rate = preamble_tol_lo_rate
    self.preamble_tol_normal_rate = preamble_tol_normal_rate
    self.preamble_tol_hi_rate = preamble_tol_hi_rate
    self.rssi_smoothing = int(ceil(log(rssi_smoothing, 2)))-1
    self.rssi_offset = rssi_offset
    self.lora_bw = lora_bw
    self.lora_SF = lora_SF
    self.gaussian = gaussian
    self.paramp = paramp
    File.__init__(self, SystemFileIds.FACTORY_SETTINGS.value, 56)
    Validatable.__init__(self)

  @staticmethod
  def parse(s, offset=0, length=56):
    gain = s.read("int:8")
    rx_bw_low_rate = s.read("uint:32")
    rx_bw_normal_rate = s.read("uint:32")
    rx_bw_high_rate = s.read("uint:32")
    bitrate_lo_rate = s.read("uint:32")
    fdev_lo_rate = s.read("uint:32")
    bitrate_normal_rate = s.read("uint:32")
    fdev_normal_rate = s.read("uint:32")
    bitrate_hi_rate = s.read("uint:32")
    fdev_hi_rate = s.read("uint:32")

    preamble_size_lo_rate = s.read("uint:8")
    preamble_size_normal_rate = s.read("uint:8")
    preamble_size_hi_rate = s.read("uint:8")

    preamble_detector_size_lo_rate = s.read("uint:8")
    preamble_detector_size_normal_rate = s.read("uint:8")
    preamble_detector_size_hi_rate = s.read("uint:8")
    preamble_tol_lo_rate = s.read("uint:8")
    preamble_tol_normal_rate = s.read("uint:8")
    preamble_tol_hi_rate = s.read("uint:8")

    rssi_smoothing = s.read("uint:8")
    rssi_offset = s.read("uint:8")

    lora_bw = s.read("uint:32")
    lora_SF = s.read("uint:8")

    gaussian = s.read("uint:8")
    paramp = s.read("uint:16")

    return FactorySettingsFile(gain=gain, rx_bw_low_rate=rx_bw_low_rate, rx_bw_normal_rate=rx_bw_normal_rate, rx_bw_high_rate=rx_bw_high_rate,
                               bitrate_lo_rate=bitrate_lo_rate, fdev_lo_rate=fdev_lo_rate,
                               bitrate_normal_rate=bitrate_normal_rate, fdev_normal_rate=fdev_normal_rate,
                               bitrate_hi_rate=bitrate_hi_rate, fdev_hi_rate=fdev_hi_rate,
                               preamble_size_lo_rate=preamble_size_lo_rate, preamble_size_normal_rate=preamble_size_normal_rate,
                               preamble_size_hi_rate=preamble_size_hi_rate, preamble_detector_size_lo_rate = preamble_detector_size_lo_rate,
                               preamble_detector_size_normal_rate=preamble_detector_size_normal_rate, preamble_detector_size_hi_rate = preamble_detector_size_hi_rate,
                               preamble_tol_lo_rate = preamble_tol_lo_rate, preamble_tol_normal_rate = preamble_tol_normal_rate,
                               preamble_tol_hi_rate = preamble_tol_hi_rate,
                               rssi_smoothing=rssi_smoothing, rssi_offset=rssi_offset,
                               lora_bw=lora_bw, lora_SF=lora_SF,
                               gaussian=gaussian, paramp=paramp)

  def __iter__(self):
    yield self.gain
    for byte in bytearray(struct.pack(">I", self.rx_bw_low_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.rx_bw_normal_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.rx_bw_high_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.bitrate_lo_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.fdev_lo_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.bitrate_normal_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.fdev_normal_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.bitrate_hi_rate)):
      yield byte
    for byte in bytearray(struct.pack(">I", self.fdev_hi_rate)):
      yield byte
    yield self.preamble_size_lo_rate
    yield self.preamble_size_normal_rate
    yield self.preamble_size_hi_rate
    yield self.preamble_detector_size_lo_rate
    yield self.preamble_detector_size_normal_rate
    yield self.preamble_detector_size_hi_rate
    yield self.preamble_tol_lo_rate
    yield self.preamble_tol_normal_rate
    yield self.preamble_tol_hi_rate
    yield self.rssi_smoothing
    yield self.rssi_offset
    for byte in bytearray(struct.pack(">I", self.lora_bw)):
      yield byte
    yield self.lora_SF
    yield self.gaussian
    for byte in bytearray(struct.pack(">H", self.paramp)):
      yield byte

  def __str__(self):
    return "gain={}, rx_bw_low_rate={}, rx_bw_normal_rate={}, rx_bw_high_rate={}, low rate={} : {}, normal rate={} : {}, high rate={} : {}, preamble sizes {} : {} : {}, preamble detector size {} : {} : {} with tol {} : {} : {}, rssi smoothing {} with offset {}\nlora sf set to {}, bw to {}\ngaussian set to {} and paramp to {} microseconds".format(self.gain, self.rx_bw_low_rate, self.rx_bw_normal_rate, self.rx_bw_high_rate,
                                                                                         self.bitrate_lo_rate, self.fdev_lo_rate,
                                                                                         self.bitrate_normal_rate, self.fdev_normal_rate, self.bitrate_hi_rate, self.fdev_hi_rate,
                                                                                         self.preamble_size_lo_rate, self.preamble_size_normal_rate, self.preamble_size_hi_rate,
                                                                                         self.preamble_detector_size_lo_rate, self.preamble_detector_size_normal_rate, self.preamble_detector_size_hi_rate,
                                                                                         self.preamble_tol_lo_rate, self.preamble_tol_normal_rate, self.preamble_tol_hi_rate,
                                                                                         self.rssi_smoothing, self.rssi_offset,
                                                                                         self.lora_bw, self.lora_SF,
                                                                                         self.gaussian, self.paramp)