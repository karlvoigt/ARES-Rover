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

/*
 * This sketch demonstrate the central API(). A additional bluefruit
 * that has bleuart as peripheral is required for the demo.
 */
#include <bluefruit.h>

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
BLEClientBas  clientBas;  // battery client
BLEClientDis  clientDis;  // device information client
BLEClientUart clientUart; // bleuart client

bool BLE_flag;
bool ACK_flag;
bool newData;
bool msgReceiveStarted;

char msg[sizeof(BLERoverMessage)];
int msg_index = 0;
char data_buffer[DATA_BUFFER_SIZE];
int data_buffer_index = 0;
bool transmissionStarted = false;

SoftwareTimer syncTimer;

uint16_t g_conn_handle;  // Handle of the current connection

void syncTimerCallback(TimerHandle_t xTimer)
{
  (void) xTimer;

  if (!BLE_flag) {
    Serial.println("Restarting scan");
    Bluefruit.Scanner.start(3);                  // 0 = Don't stop scanning after n seconds
    Bluefruit.Scanner.resume(); 
  }              

  syncTimer.stop(); 
  syncTimer.reset();
  Serial.println("Sync timer callback");
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  syncTimer.begin(10000, syncTimerCallback);

  //Set all the flags
  BLE_flag = false;
  ACK_flag = false;
  newData = false;

  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Seed Xiao Central");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }else
  {      
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{

  g_conn_handle=conn_handle;



  // Serial.print("Discovering BLE Uart Service ... ");
  if ( clientUart.discover(conn_handle) )
  {
  //   Serial.println("Found it");
    syncTimer.start();
    BLE_flag = true;
    ACK_flag = false;
    msg_index = 0;
    msgReceiveStarted = false;

  //   Serial.println("Enable TXD's notify");
    clientUart.enableTXD();
    Serial.println("Connected");

    // newConnection = true;

    // Serial.println("Ready to receive from peripheral");
  }else
  {
    // Serial.println("Found NONE");
    
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  // Bluefruit.Scanner.stop();
  BLE_flag = false;
  clientUart.flush();

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  if (reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION) {
    newData = false;
  }
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  Serial.println("Received data");
  if (BLE_flag) {
    Serial.println("BLE flag is true");

    //Wait for acknowledge from BLE
    if (!ACK_flag) {
      //Check for # which is Acknowledge
      if (!msgReceiveStarted) {
        if (clientUart.peek() == '#') {
          ACK_flag = true;
          clientUart.read();
          Serial.println("Received ACK");
        } else if (clientUart.peek() == START_DELIMITER){
          Serial.println("Received message");
          //Read the message from BLE
          msg[msg_index]=clientUart.read();
          msg_index++;
          //Send the message to Serial
          Serial1.write(msg, sizeof(BLERoverMessage));
          Serial.write(msg, sizeof(BLERoverMessage));
        } else {
          //If not start delimiter or ACK, clear BLE UART queue
          clientUart.read();
        }
      } else {
        if (msg_index == sizeof(BLERoverMessage)-1) {
          if (clientUart.peek() == END_DELIMITER) {
            //Correct message
            msg[msg_index]=clientUart.read();
            msgReceiveStarted = false;
            msg_index = 0;
            ACK_flag = true;

            //Send the message to Serial
            Serial1.write(msg, sizeof(BLERoverMessage));
            Serial.write(msg, sizeof(BLERoverMessage));

            // //Print data until other device disconnects
            // while (BLE_flag) {
            //   if (newData) {
            //     // Send the received data
            //     Serial.println("Sending data");
            //     clientUart.write(data_buffer, DATA_BUFFER_SIZE );
            //   } else {
            //     Serial.println("Sending ACK");
            //     //Send a ACK message
            //     clientUart.write('#');
            //   }
            //   delay(5);
            // }
            // newData = false;
          } else {
            //Incorrect message and clear the queue
            msgReceiveStarted = false;
            msg_index = 0;
            clientUart.read();
          }
        } else {
          msg[msg_index]=clientUart.read();
          msg_index++;
        }
      }
    } else {
      if (newData) {
        // Send the received data
        Serial.println("Sending data");
        clientUart.write(data_buffer, DATA_BUFFER_SIZE );
      } else {
        Serial.println("Sending ACK");
        //Send a ACK message
        clientUart.write('#');
      }
      delay(5);
    }
  }
  
}

void loop()
{

  while (Serial1.available()) {
    // CHeck the start delimiter
    if (Serial1.peek()==START_DELIMITER) {
      Serial.println("Received start delimiter");
      Serial1.readBytes(data_buffer, DATA_BUFFER_SIZE);
      //Check the end delimiter
      if (data_buffer[DATA_BUFFER_SIZE-1] == END_DELIMITER) {
        // Set newData flag
        Serial.println("Received message");
        // bleuart.write(data_buffer, DATA_BUFFER_SIZE );
        newData = true;
      }
    } else {
      Serial.println(Serial1.read());
    }
  }

  while (Serial.available() > 0) {
    // CHeck the start delimiter
    if (Serial.peek()==START_DELIMITER) {
      Serial.println("Received start delimiter");
      Serial.readBytes(data_buffer, DATA_BUFFER_SIZE);
      //Check the end delimiter
      if (data_buffer[DATA_BUFFER_SIZE-1] == END_DELIMITER) {
        // Set newData flag
        Serial.println("Received message");
        // bleuart.write(data_buffer, DATA_BUFFER_SIZE );
        newData = true;
      }
    } else {
      Serial.println(Serial.read());
    }
  }

  // if (BLE_flag) {
  //   Serial.println("BLE flag is true");
  //   ACK_flag = false;

  //   //Wait for acknowledge from BLE
  //   while (!ACK_flag) {
  //     //Check for # which is Acknowledge
  //     if (clientUart.peek() == '#') {
  //       Serial.println("Received ACK");
  //       ACK_flag = true;
  //     } else if (clientUart.peek() == START_DELIMITER){
  //       Serial.println("Received message");
  //       ACK_flag = true;
  //       //Read the message from BLE
  //       char msg[sizeof(BLERoverMessage)];
  //       clientUart.readBytes(msg, sizeof(BLERoverMessage));
  //       //Send the message to Serial
  //       Serial1.write(msg, sizeof(BLERoverMessage));
  //       Serial.write(msg, sizeof(BLERoverMessage));
  //     } else {
  //       //If not start delimiter, clear BLE UART queue
  //       clientUart.read();
  //     }
  //     //Break out if BLE disconnects before transfer completes
  //     if (!BLE_flag) break;
  //   }

  //   //Print data until other device disconnects
  //   while (BLE_flag) {
  //     if (newData) {
  //       // Send the received data
  //       Serial.println("Sending data");
  //       clientUart.write(data_buffer, DATA_BUFFER_SIZE );
  //     } else {
  //       Serial.println("Sending ACK");
  //       //Send a ACK message
  //       clientUart.write('#');
  //     }
  //     delay(5);
  //   }

  //   newData = false;
  // }
  delay(5);
}
