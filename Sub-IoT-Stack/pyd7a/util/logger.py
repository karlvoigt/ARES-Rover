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
import logging

# configure the logger when using the supplied examples, as opposed to running as a library
# called from an external program (which then should take care of configuring the logging)
def configure_default_logger(is_verbose_enabled, logging_level = logging.INFO):
  log = logging.getLogger()
  formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
  handler = logging.StreamHandler()
  handler.setFormatter(formatter)
  log.addHandler(handler)
  log.setLevel(logging_level)
  if is_verbose_enabled:
    log.setLevel(logging.DEBUG)