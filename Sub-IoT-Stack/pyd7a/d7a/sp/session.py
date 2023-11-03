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

# TODO can be removed?
class States(object):
  IDLE    = 0  # Inactive Session
  DORMANT = 1  # The group of Requests needs to be executed within a timeout 
               # period, which has not expired. After completion of the 
               # period, the Dormant Session is transformed into Pending 
               # Session.
  PENDING = 2  # The group of Requests needs to be executed as soon as 
               # possible.
  ACTIVE  = 3  # The Session is being currently executed using the D7A 
               # Session Protocol.
  DONE    = 4  # Terminated Session

  ALL     = [ IDLE, DORMANT, PENDING, ACTIVE, DONE ]

  @staticmethod
  def SCHEMA():
    return { "type": "integer", "allowed" : States.ALL }
