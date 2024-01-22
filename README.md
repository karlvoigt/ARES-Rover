# ARES-Rover

Dash 7 module is flashed with ARES app.

The BLE module on the rover and plugged into the computer is flashed with bleuart.ino and central_bleuart.ino respectively,for the demo, however this the other way around will also work (as long as the debug serial output of ble central is disabled first)

The ares.py python program in the examples folder of pyd7a is used to communicate with the gateway and upload to MQTT and the telegraf.conf stores the telegraf configuration used.