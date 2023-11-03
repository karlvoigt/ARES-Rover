#!/usr/bin/env python
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

import argparse

import logging
import platform
import signal
import sys
import traceback

import serial
import time

import paho.mqtt.client as mqtt

from modem.modem import Modem
from util.logger import configure_default_logger


class Modem2Mqtt():

  def __init__(self):
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-d", "--device", help="serial device /dev file modem",
                           default="/dev/ttyACM0")
    argparser.add_argument("-di", "--device-id", help="gateway device-id", required=True)
    argparser.add_argument("-r", "--rate", help="baudrate for serial device", type=int, default=115200)
    argparser.add_argument("-v", "--verbose", help="verbose", default=False, action="store_true")
    argparser.add_argument("-b", "--broker", help="mqtt broker hostname",
                             default="localhost")
    argparser.add_argument("-t", "--topic", help="mqtt topic",
                             default="/gw/{}")

    self.serial = None
    self.modem_uid = None
    self.bridge_count = 0
    self.next_report = 0

    self.config = argparser.parse_args()
    configure_default_logger(self.config.verbose)

    self.modem = Modem(self.config.device, self.config.rate, self.on_command_received, skip_alp_parsing=True)
    self.modem.connect()
    self.connect_to_mqtt()


  def connect_to_mqtt(self):
    self.connected_to_mqtt = False

    self.mq = mqtt.Client("", True, None, mqtt.MQTTv31)
    self.mq.on_connect = self.on_mqtt_connect
    self.mq.on_message = self.on_mqtt_message
    self.mqtt_topic_incoming = self.config.topic.format(self.config.device_id)
    #self.mqtt_topic_outgoing = self.config.topic.format(self.modem_uid)

    self.mq.connect(self.config.broker, 1883, 60)
    self.mq.loop_start()
    while not self.connected_to_mqtt: pass  # busy wait until connected
    logging.info("Connected to MQTT broker on {}, sending to topic {}".format(
      self.config.broker,
      self.mqtt_topic_incoming
    ))

  def on_mqtt_connect(self, client, config, flags, rc):
    #self.mq.subscribe(self.mqtt_topic_outgoing)
    self.connected_to_mqtt = True

  def on_mqtt_message(self, client, config, msg):
    logging.info("on_message") # TODO
    # try:    self.handle_msg(msg.topic, msg.payload)
    # except:
    #   exc_type, exc_value, exc_traceback = sys.exc_info()
    #   lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    #   trace = "".join(lines)
    #   self.log("failed to handle incoming message:", msg.payload, trace)

  def publish_to_mqtt(self, msg):
    self.mq.publish(self.mqtt_topic_incoming, msg)

  def __del__(self): # pragma: no cover
    try:
      self.mq.loop_stop()
      self.mq.disconnect()
    except: pass

  def on_command_received(self, cmd):
    try:
      logging.info("Command received: binary ALP (size {})".format(len(cmd)))

      # publish raw ALP command to incoming ALP topic, we will not parse the file contents here (since we don't know how)
      # so pass it as an opaque BLOB for parsing in backend
      self.publish_to_mqtt(bytearray(cmd))
    except:
      exc_type, exc_value, exc_traceback = sys.exc_info()
      lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
      trace = "".join(lines)
      logging.error("Exception while processing command: \n{}".format(trace))

  def run(self):
    logging.info("Started")
    keep_running = True
    while keep_running:
      try:
        if platform.system() == "Windows":
          time.sleep(1)
        else:
          signal.pause()
      except KeyboardInterrupt:
        logging.info("received KeyboardInterrupt... stopping processing")
        keep_running = False

      self.report_stats()

  def keep_stats(self):
    self.bridge_count += 1

  def report_stats(self):
    if self.next_report < time.time():
      if self.bridge_count > 0:
        logging.info("bridged %s messages" % str(self.bridge_count))
        self.bridge_count = 0
      self.next_report = time.time() + 15 # report at most every 15 seconds

if __name__ == "__main__":
  Modem2Mqtt().run()
