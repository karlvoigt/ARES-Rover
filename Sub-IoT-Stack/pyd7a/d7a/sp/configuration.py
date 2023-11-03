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

# author: Christophe VG <contact@christophe.vg>
# class implementation of (FIFO) configuration

from d7a.support.schema   import Validatable, Types

from d7a.types.ct         import CT

from d7a.sp.qos           import QoS
from d7a.sp.session       import States

from d7a.d7anp.addressee import Addressee

class Configuration(Validatable):

  SCHEMA = [{
    "qos"        : Types.OBJECT(QoS),
    "dorm_to"    : Types.OBJECT(CT),
    "addressee"  : Types.OBJECT(Addressee)
  }]

  def __init__(self, qos=QoS(), dorm_to=CT(), addressee=Addressee()):
    self.qos         = qos
    self.dorm_to     = dorm_to
    self.addressee   = addressee
    super(Configuration, self).__init__()

  def __iter__(self):
    for byte in self.qos: yield byte
    for byte in self.dorm_to: yield byte
    for byte in self.addressee: yield byte

  def __str__(self):
    return str(self.as_dict())

  @staticmethod
  def parse(s):
    qos = QoS.parse(s)
    dorm_to = CT.parse(s)
    addressee = Addressee.parse(s)
    return Configuration(qos=qos, dorm_to=dorm_to, addressee=addressee)