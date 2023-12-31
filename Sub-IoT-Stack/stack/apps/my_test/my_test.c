#include "hwuart.h"
#include "scheduler.h"
#include "CustomProtocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hwleds.h"
#include "hwsystem.h"
#include "hwlcd.h"

#include "scheduler.h"
#include "timer.h"
#include "debug.h"
#include "d7ap_fs.h"
#include "log.h"

#include "d7ap.h"
#include "alp_layer.h"
#include "dae.h"

#include "platform.h"

#define UART_PORT_IDX 0 // Adjust based on your hardware setup
#define UART_BAUDRATE 115200 // Adjust based on your requirements
#define UART_PINS 0 // Adjust based on your hardware setup
#define DATA_BUFFER_SIZE 64 // Adjust based on expected data size

#define SENSOR_FILE_ID 0x30 // Example file ID
#define SENSOR_FILE_SIZE 7  // Size of the sensor data in bytes

#define GLOBAL_ECHO_BUFFER_SIZE (DATA_BUFFER_SIZE * 3 + 1)
char global_echo_buffer[GLOBAL_ECHO_BUFFER_SIZE];

uart_handle_t* uart_handle;
uint8_t data_buffer[DATA_BUFFER_SIZE];
uint16_t data_buffer_index = 0;
uint8_t transmissionStarted = 0;

// Define the D7 interface configuration used for sending the ALP command on
static alp_interface_config_d7ap_t itf_config = (alp_interface_config_d7ap_t){
  .itf_id = ALP_ITF_ID_D7ASP,
  .d7ap_session_config = {
    .qos = {
        .qos_resp_mode = SESSION_RESP_MODE_PREFERRED,
        .qos_retry_mode = SESSION_RETRY_MODE_NO
    },
    .dormant_timeout = 0,
    .addressee = {
        .ctrl = {
            .nls_method = AES_NONE,
            .id_type = ID_TYPE_NOID,
        },
        .access_class = 0x01,
        .id = { 0 }
    }
  }
};

void process_received_data(uint8_t* buffer, uint16_t size) {
    if (size < CUSTOM_MESSAGE_SIZE) {
        // Echo the message back over UART for debugging
        char echo_message[40];
        snprintf(echo_message, sizeof(echo_message), "Expected size of %d but got size of %d\n", CUSTOM_MESSAGE_SIZE, size);
        uart_send_string(uart_handle, echo_message);
        uart_send_string(uart_handle, "Error: Data too small\r\n");
        return;
    }

    // Echo the entire data buffer for debugging (as hexadecimal)
    char* ptr = global_echo_buffer;
    for (int i = 0; i < size; i++) {
        ptr += snprintf(ptr, 4, "%02X ", buffer[i]);
    }
    uart_send_string(uart_handle, global_echo_buffer);
    uart_send_string(uart_handle, "\r\n");

    CustomMessage message;
    size_t offset = 0;

    // Manually copy data from buffer to struct fields
    message.startDelimiter = buffer[offset++];
    message.sensorType = buffer[offset++];
    message.timestamp = (uint32_t)buffer[offset] | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);
    offset += 4;
    message.sensorData = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
    offset += 2;
    message.checksum = buffer[offset++];
    message.endDelimiter = buffer[offset];

    // Validate the message
    if (message.startDelimiter != START_DELIMITER || message.endDelimiter != END_DELIMITER) {
        // Debugging: Print out the values of the start and end delimiters
        uart_send_string(uart_handle, "Error: Invalid delimiters\r\n");
        char debug_message[50];
        snprintf(debug_message, sizeof(debug_message), "Start Delimiter: 0x%02X, End Delimiter: 0x%02X\r\n", message.startDelimiter, message.endDelimiter);
        uart_send_string(uart_handle, debug_message);
        return;
    }

    uint8_t calculatedChecksum = START_DELIMITER + message.sensorType + (message.timestamp & 0xFF) + ((message.timestamp >> 8) & 0xFF) + ((message.timestamp >> 16) & 0xFF) + ((message.timestamp >> 24) & 0xFF) + (message.sensorData & 0xFF) + ((message.sensorData >> 8) & 0xFF);
    if (calculatedChecksum != message.checksum) {
        uart_send_string(uart_handle, "Error: Checksum mismatch\r\n");
        char debug_message[50];
        snprintf(debug_message, sizeof(debug_message), "CHECKSUM: 0x%02X, EXPECTED: 0x%02X\r\n", message.checksum, calculatedChecksum);
        uart_send_string(uart_handle, debug_message);
        return;
    }

    // Message is valid, proceed with Dash7 transmission
    uint8_t payload[7];
    payload[0] = message.sensorType;
    memcpy(&payload[1], &message.timestamp, 4);
    payload[5] = (message.sensorData >> 8) & 0xFF;
    payload[6] = message.sensorData & 0xFF;

    // Generate ALP command.
    // We will be sending a return file data action, without a preceding file read request.
    // This is an unsolicited message, where we push the sensor data to the gateway(s).
    
    // alloc command. This will be freed when the command completes
    alp_command_t* command = alp_layer_command_alloc(false, false);
    
    // forward to the D7 interface
    alp_append_forward_action(command, (alp_interface_config_t*)&itf_config, d7ap_session_config_length(&itf_config.d7ap_session_config));

    // add the return file data action
    alp_append_return_file_data_action(command, SENSOR_FILE_ID, 0, sizeof(payload), payload);

    // and finally execute this
    alp_layer_process(command);
}



void uart_rx_callback(uart_handle_t* uart_handler, uint8_t byte) {
    // Accumulate bytes in the buffer
    if (byte==START_DELIMITER) transmissionStarted=1;
    if (transmissionStarted) {
      if (data_buffer_index < DATA_BUFFER_SIZE) {
          data_buffer[data_buffer_index++] = byte;
          // Echo the entire data buffer for debugging (as hexadecimal)
          // char* ptr = global_echo_buffer;
          // for (int i = 0; i < data_buffer_index; i++) {
          //     ptr += snprintf(ptr, 4, "%02X ", data_buffer[i]);
          // }
          // uart_send_string(uart_handle, global_echo_buffer);
          // uart_send_string(uart_handle, "\r\n");

          // Check if we received the end delimiter
          if (byte == END_DELIMITER) {
              transmissionStarted=0;
              uart_send_string(uart_handle, global_echo_buffer);
              uart_send_string(uart_handle, "\r\n");
              uint8_t tempIndex= data_buffer_index;
              data_buffer_index = 0; // Reset buffer index for next message
              // Process the received data
              process_received_data(data_buffer, tempIndex);
          }
      } else {
          // Buffer overflow, reset buffer index
          data_buffer_index = 0;
          uart_send_string(uart_handle, "Error: Buffer overflow\r\n");
      }
    }
}

void on_alp_command_completed_cb(uint8_t tag_id, bool success)
{
    if(success)
      log_print_string("Command (%i) completed successfully", tag_id);
    else
      log_print_string("Command failed, no ack received");

    // reschedule sensor measurement
    // timer_post_task_delay(&execute_sensor_measurement, SENSOR_INTERVAL_SEC);
}

void on_alp_command_result_cb(alp_command_t *alp_command, alp_interface_status_t* origin_itf_status)
{
  if(origin_itf_status && (origin_itf_status->itf_id == ALP_ITF_ID_D7ASP) && (origin_itf_status->len > 0)) {
      d7ap_session_result_t* d7_result = ((d7ap_session_result_t*)origin_itf_status->itf_status);
      log_print_string("recv response @ %i dB link budget from:", d7_result->rx_level);
      log_print_data(d7_result->addressee.id, d7ap_addressee_id_length(d7_result->addressee.ctrl.id_type));
  }
  log_print_string("response payload:");
  log_print_data(alp_command->alp_command, fifo_get_size(&alp_command->alp_command_fifo));
  fifo_skip(&alp_command->alp_command_fifo, fifo_get_size(&alp_command->alp_command_fifo));
}

static alp_init_args_t alp_init_args;

void bootstrap() {
    log_print_string("Device booted\n");
    d7ap_fs_init();
    d7ap_init();

    alp_init_args.alp_command_completed_cb = &on_alp_command_completed_cb;
    alp_init_args.alp_command_result_cb = &on_alp_command_result_cb;
    alp_layer_init(&alp_init_args, false);

    uart_handle = uart_init(UART_PORT_IDX, UART_BAUDRATE, UART_PINS);
    if(uart_handle == NULL) {
        // Initialization failed, handle the error
    }

    bool enabled = uart_enable(uart_handle);
    if(!enabled) {
        // Enabling UART failed, handle the error
    }

    uart_set_rx_interrupt_callback(uart_handle, uart_rx_callback);
    uart_rx_interrupt_enable(uart_handle);

    // Send a startup message over UART
    uart_send_string(uart_handle, "UART Initialized Successfully\r\n");
}
