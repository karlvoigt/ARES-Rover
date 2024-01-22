/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#define START_DELIMITER 0x5B    // '['
#define END_DELIMITER   0x5D    // ']'

typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    float xCoord;
    float yCoord;
    float angle; // In degrees
    uint16_t temperature;
    uint16_t humidity;
    uint16_t light;
    uint16_t ir;
    uint32_t timestamp;
    uint8_t battery;
    uint8_t endDelimiter;
} BLERoverMessage;

#define DATA_BUFFER_SIZE        sizeof(BLERoverMessage)     


// BLE Service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
bool ISR_flag;
bool BLE_flag;
bool ACK_flag;
bool newData;
bool FirstConnection;

char data_buffer[DATA_BUFFER_SIZE];
int data_buffer_index = 0;
bool transmissionStarted = false;

SoftwareTimer syncTimer;

uint16_t g_conn_handle;  // Handle of the current connection

void setup()
{
  BLE_flag = false;
  ACK_flag = false;
  newData = false;
  FirstConnection = true;
  
  // Set up D0 as an interrupt pin
  pinMode(D0, INPUT_PULLUP_SENSE);

  // Serial.begin(115200);
  Serial1.begin(115200);

  syncTimer.begin(10000, syncTimerCallback);

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial ) yield();
#endif
  // while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Seeed Studio");
  bledis.setModel("XIAO nRF52840");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Serial.println("Setup complete");

  // Set up and start advertising
  startAdv();
}

void syncTimerCallback(TimerHandle_t xTimer)
{
  (void) xTimer;

  if (!BLE_flag) {
    Bluefruit.Advertising.start(1);                  // 0 = Don't stop scanning after n seconds
    // startAdv();
  }      

  syncTimer.stop();
  syncTimer.reset();
  // Serial.println("Sync timer callback");
}

void loop()
{
  // Check for UART message
  while (Serial1.available()) {
    // CHeck the start delimiter
    if (Serial1.peek()==START_DELIMITER) {
      Serial1.readBytes(data_buffer, DATA_BUFFER_SIZE);
      //Check the end delimiter
      if (data_buffer[DATA_BUFFER_SIZE-1] == END_DELIMITER) {
        // Set newData flag
        // Serial.println("Received message");
        // bleuart.write(data_buffer, DATA_BUFFER_SIZE );
        newData = true;
      }
    }
  }

  if (BLE_flag) {
    ACK_flag = false;

    // //Wait for connection confirmation
    // while (!bleuart.available()) {
    //   bleuart.write('.');
    //   //Break out if BLE disconnects before transfer completes
    //   if (!BLE_flag) {
    //     break;
    //   }
    //   delay(5);
    // }
    // bleuart.write('.');
    // //Clear all the '.' from the queue
    // while (bleuart.peek()=='.') {
    //   bleuart.read();
    // }
    // Serial.println("Connection confirmed");
    
    //Wait for acknowledge from BLE
    while (!ACK_flag) {
      if (newData) {
        // Serial.println("Sending Data");
        // Send the received data
        bleuart.write(data_buffer, DATA_BUFFER_SIZE );
      } else {
        // Serial.println("Sending ACK");
        //Send a ACK message
        bleuart.write('#');
      }
      // Serial.println("Waiting for ACK");
      //Wait for acknowledge from BLE
      // while (!bleuart.available()) {
      //   delay(1);
      //   //Break out if BLE disconnects before transfer completes
      //   if (!BLE_flag) {
      //     break;
      //   }
      //   // bleuart.write('#');
      // }
      if (bleuart.available()){
        //Check for # which is Acknowledge
        if (bleuart.peek() == '#') {
          // Serial.println("Received ACK");
          bleuart.read(); //Clear the byte from the queue
          ACK_flag = true;
        } else if (bleuart.peek() == START_DELIMITER){
          // Serial.println("Received message");
          //Read the message from BLE
          BLERoverMessage message;
          bleuart.readBytes((uint8_t*)&message, sizeof(message));
          //Send the message to Serial
          Serial1.write((uint8_t*)&message, sizeof(message));
          // Serial.write((uint8_t*)&message, sizeof(message));
          //Wait for transmission to complete
          // Serial.flush();
          delay(30);  //Serial.flush() not working so using delay
          ACK_flag = true;
        } else {
          //If not start delimiter, clear BLE UART queue
          // Serial.println("Incorrect delimiter");
          // Serial.println(bleuart.read());
        }
      }
      if (!BLE_flag) break;
    }

    newData = false;

    Bluefruit.disconnect(g_conn_handle);
    BLE_flag=false;
    // Serial.println("Disconecting");
  }

  // power down nrf52.
  // sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  // sd_app_evt_wait();  // this function puts the nRF52 to sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
  // sd_power_system_off();  // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(false);  //TODO: set this depending on if we wish to connect or not
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  syncTimer.start();
  g_conn_handle=conn_handle;
  Bluefruit.Advertising.stop();
  // Get the reference to current connection
  // BLEConnection* connection = Bluefruit.Connection(conn_handle);

  // char central_name[32] = { 0 };
  // connection->getPeerName(central_name, sizeof(central_name));
  // Serial.println(central_name);
  BLE_flag = true;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  BLE_flag = false;
  bleuart.flush();

  // Serial.println();
  // Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
