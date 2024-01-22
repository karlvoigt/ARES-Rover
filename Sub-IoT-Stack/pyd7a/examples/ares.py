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
import sys
import time
import traceback

#from examples.dormant_session import send_dormant_session

from d7a.alp.command import Command
from d7a.alp.interface import InterfaceType
from d7a.d7anp.addressee import Addressee, IdType
from d7a.sp.configuration import Configuration
from d7a.sp.qos import QoS, ResponseMode
from d7a.types.ct import CT
from modem.modem import Modem

import paho.mqtt.client as mqtt
import json
import struct

# This example can be used with a node running the gateway app included in Sub-IoT.
# The gateway is continuously listening for foreground frames.
# Messages pushed by other nodes (running for example the sensor_push app) will be received by the gateway node,
# transmitted over serial and the received_command_callback() function below will be called.
# It will then transmit a dormant session to the configured UID or to the first transmitter of the first received message.
# This dormant session has a timeout of 5 minutes and will write file 0x42 (which is not yet defined in most devices but should raise an error visible in RTTLog)
from util.logger import configure_default_logger

def decode_message(data):
    # Define the format string for struct.unpack
    # < for little endian, B for uint8_t, f for float, H for uint16_t, I for uint32_t
    # format_str = '<BfffHHHHIBB'

    # # Unpack the data
    # unpacked_data = struct.unpack(format_str, data)

    #extract data from byte array to STM32ToDash7Message using manual method
    startDelimiter = data[0]
    bytes_to_convert = bytes(data[1:5])  # Get the 4 bytes you want to convert
    xCoord = struct.unpack('<f', bytes_to_convert)[0]  # Convert the bytes to a float
    bytes_to_convert = bytes(data[5:9])  # Get the 4 bytes you want to convert
    yCoord = struct.unpack('<f', bytes_to_convert)[0]  # Convert the bytes to a float
    bytes_to_convert = bytes(data[9:13])  # Get the 4 bytes you want to convert
    angle = struct.unpack('<f', bytes_to_convert)[0]  # Convert the bytes to a float
    temperature = (data[14]<<8) | data[13]
    humidity = (data[16]<<8) | data[15]
    light = (data[18]<<8) | data[17]
    ir = (data[20]<<8) | data[19]
    uid = (data[24]<<24) | (data[23]<<16) | (data[22]<<8) | data[21]
    battery = data[25]
    endDelimiter = data[26]

    # Create a dictionary to store the data
    message = {
        'startDelimiter': startDelimiter,
        'xCoord': xCoord,
        'yCoord': yCoord,
        'angle': angle,
        'temperature': temperature,
        'humidity': humidity,
        'light': light,
        'ir': ir,
        'uid': uid,
        'battery': battery,
        'endDelimiter': endDelimiter
    }

    return message

def received_command_callback(cmd):
  # global addressee_id, try_dormant_session
  # try:
  #   logging.info("gotten message {}".format(cmd))
  #   addressee_id = cmd.interface_status.operand.interface_status.addressee.id
  #   mydata = [0x5b, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
  #             0x8, 0x9, 0x10, 0x5D]
  #   cmd = Command.create_with_write_file_action(
  #     file_id=0x62,
  #     data=mydata,
  #     interface_type=InterfaceType.D7ASP,
  #     interface_configuration=Configuration(
  #       qos=QoS(resp_mod=ResponseMode.RESP_MODE_ANY),
  #       addressee=Addressee(
  #         id_type=IdType.UID,
  #         id=addressee_id,
  #         access_class=0x11
  #       ),
  #       dorm_to=CT.compress(60 * 5)
  #     )
  #   )

  #   modem.execute_command(
  #     timeout_seconds=5,
  #     alp_command=cmd
  #   )
  #   logging.info("sent message")
  #   try_dormant_session = True
  # except Exception as e:
  #   logging.info("Error sending message: {}".format(e))
    
  #   return
  global addressee_id
  try:
    # Existing code
    addressee_id = cmd.interface_status.operand.interface_status.addressee.id
    data = cmd.actions[0].operation.operand.data

    if (data[0]==0x5B) and (data[26]==0x5D):
      # Decode the message
      message = decode_message(data)


      temp = message['temperature']*175/65535 - 45
      hum = message['humidity']*125/65535 - 6

      message['temperature'] = temp
      message['humidity'] = hum

      #Publish the values to MQTT
      client.publish(topic, json.dumps(message))

      # Print the message
      # Print the message
      logging.info("Command received: binary ALP (size {})".format(len(data)))
      logging.info("Data: {}".format(data))
      logging.info("Position: [{:.2f}, {:.2f}]".format(message['xCoord'], message['yCoord']))
      logging.info("Angle: {:.2f}".format(message['angle']))
      logging.info("Temperature: {:.2f} C".format(temp))
      logging.info("Humidity: {:.2f}%".format(hum))
      logging.info("Light: {}".format(message['light']))
      logging.info("IR: {}".format(message['ir']))
      logging.info("UID: {}".format(message['uid']))
      logging.info("Battery: {}".format(message['battery']))

      test_msg()
    else:
      logging.info("Command received: binary ALP (size {})".format(len(data)))
      logging.info("Data: {}".format(data))

  except (AttributeError, IndexError):
    # probably an answer on downlink we don't care about right now
    return
  except:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    trace = "".join(lines)
    logging.error("Exception while processing command: \n{}".format(trace))
  
def test_msg():
  global data
  if data == 1:
    data=0
  else:
    data=1

  msg_data = [data]
  logging.info("sending payload {}".format(data))

  logging.info("over {}".format("dormant session" if msg_data[0] == "D" else "low power listening"))
  
  cmd = Command.create_with_write_file_action(
    file_id=0x42,
    offset=2,
    data=msg_data,
    interface_type=InterfaceType.D7ASP,
    interface_configuration=Configuration(
      qos=QoS(resp_mod=ResponseMode.RESP_MODE_ANY),
      addressee=Addressee(
        id_type=IdType.UID,
        id=addressee_id,
        access_class=0x11
      ),
      dorm_to= CT()
    )
  )

  modem.execute_command(
    timeout_seconds=0,
    alp_command=cmd
  )
  logging.info("sent message")
  

def rebooted_callback(cmd):
  logging.info("rebooted with reason: {}".format(cmd))

def send_dormant_session():
  global once, addressee_id, try_dormant_session, modem
  try_dormant_session = False
  # addressee_id = 4050197526414295087 #The uid of the end node
  if once and (addressee_id == 4050197526414295087):
    logging.info("sending instruction to the end node")
    once = False
    data = [10,20,30,40,50,60,70,80,90,100]

    cmd = Command.create_with_write_file_action(
      file_id=0x62,
      data=data,
      interface_type=InterfaceType.D7ASP,
      interface_configuration=Configuration(
        qos=QoS(resp_mod=ResponseMode.RESP_MODE_ANY),
        addressee=Addressee(
          id_type=IdType.UID,
          id=addressee_id,
          access_class=0x11
        ),
        dorm_to=CT.compress(60 * 5)
      )
    )

    modem.execute_command(
      timeout_seconds=0,
      alp_command=cmd
    )

argparser = argparse.ArgumentParser()
argparser.add_argument("-d", "--device", help="serial device /dev file modem",
                            default="/dev/ttyUSB0")
argparser.add_argument("-r", "--rate", help="baudrate for serial device", type=int, default=115200)
argparser.add_argument("-v", "--verbose", help="verbose", default=False, action="store_true")
argparser.add_argument("-u", "--uid", help="UID of the receiver of the dormant session, if not given, it will wait for the first message it receives and respond on that UID. (in hexstring, for example 0xb57000009151d)", default=None)
config = argparser.parse_args()

configure_default_logger(config.verbose)

#MQTT setup
global client, broker_address, topic
broker_address = "test.mosquitto.org"
topic = "ares/transmissions"
client = mqtt.Client()
client.connect(broker_address, 1883, 60)
client.loop_start()

modem = Modem(config.device, config.rate, unsolicited_response_received_callback=received_command_callback, rebooted_callback=rebooted_callback)
modem.connect()
once = True
data = 0

if config.uid is not None:
  try_dormant_session = True
  addressee_id = int(config.uid, 16)
else:
  try_dormant_session = False
  addressee_id = 0

# send_dormant_session()

try:
  while True:
    # if try_dormant_session:
      # send_dormant_session()
    # if addressee_id is not None:
    #   test_msg()
    time.sleep(5)
    
    pass
except KeyboardInterrupt:
  sys.exit(0)


