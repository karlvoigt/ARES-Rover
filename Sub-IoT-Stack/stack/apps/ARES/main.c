#include "hwuart.h"
#include "scheduler.h"
#include "CustomProtocol.h"
#include "serial_protocol.h"

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

#define UART_PORT_IDX           0       // Adjust based on your hardware setup
#define UART_BAUDRATE           115200  // Adjust based on your requirements
#define UART_PINS               0       // Adjust based on your hardware setup
#define DATA_BUFFER_SIZE        sizeof(STM32ToDash7Message)      // Adjust based on expected data size

#define SENSOR_FILE_ID          0x60    // Example file ID
#define SENSOR_FILE_SIZE        7       // Size of the sensor data in bytes
#define INSTRUCTION_FILE_ID     0x62    // File ID for the instruction file
#define INSTRUCTION_FILE_SIZE   7       // Size of the instruction file in bytes [Start Delimiter, X Coord upper, X Coord lower, Y Coord upper, Y Coord lower, Sensor Control, End Delimiter]

#define GLOBAL_ECHO_BUFFER_SIZE (DATA_BUFFER_SIZE * 3 + 1)
char global_echo_buffer[GLOBAL_ECHO_BUFFER_SIZE];

uart_handle_t* uart_handle;
uint8_t data_buffer[DATA_BUFFER_SIZE];
uint16_t data_buffer_index = 0;
uint8_t transmissionStarted = 0;
// serial_protocol_handle_t sp_handle;

//Function prototypes for all the functions
void process_received_data(uint8_t* buffer, uint16_t size);
void uart_rx_callback(uart_handle_t* uart_handler, uint8_t byte);
void on_alp_command_completed_cb(uint8_t tag_id, bool success);
void on_alp_command_result_cb(alp_command_t *alp_command, alp_interface_status_t* origin_itf_status);
static void file_modified_callback(uint8_t file_id);
void bootstrap();

static alp_init_args_t alp_init_args = {
  .alp_command_completed_cb = &on_alp_command_completed_cb,
  .alp_command_result_cb = &on_alp_command_result_cb
};

static const d7ap_fs_file_header_t instruction_file_header = { 
        .allocated_length = INSTRUCTION_FILE_SIZE,
        .length = INSTRUCTION_FILE_SIZE,
        .file_permissions
        = (file_permission_t) { .guest_read = true, .guest_write = true, .user_read = true, .user_write = true },
        .file_properties.storage_class = FS_STORAGE_PERMANENT };

static const d7ap_fs_file_header_t sensor_file_header = { 
        .allocated_length = SENSOR_FILE_SIZE,
        .length = SENSOR_FILE_SIZE,
        .file_permissions
        = (file_permission_t) { .guest_read = true, .guest_write = true, .user_read = true, .user_write = true },
        .file_properties.storage_class = FS_STORAGE_PERMANENT };

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
    if (size < sizeof(STM32ToDash7Message)) {
        // Echo the message back over UART for debugging
        char echo_message[40];
        snprintf(echo_message, sizeof(echo_message), "Expected size of %d but got size of %d\n", sizeof(STM32ToDash7Message), size);
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

    STM32ToDash7Message message;
    uint8_t offset = 0;

    // Manually copy data from buffer to struct fields
    message.startDelimiter = buffer[offset++];
    // message.yCoord = (float)buffer[offset] | ((float)buffer[offset + 1] << 8) | ((float)buffer[offset + 2] << 16) | ((float)buffer[offset + 3] << 24);
    // offset += 4;
    // message.xCoord = (float)buffer[offset] | ((float)buffer[offset + 1] << 8) | ((float)buffer[offset + 2] << 16) | ((float)buffer[offset + 3] << 24);
    // offset += 4;
    // message.angle = (float)buffer[offset] | ((float)buffer[offset + 1] << 8) | ((float)buffer[offset + 2] << 16) | ((float)buffer[offset + 3] << 24);
    // offset += 4;
    // message.temperature = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
    // offset += 2;
    // message.humidity = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
    // offset += 2;
    // message.light = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
    // offset += 2;
    // message.ir = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
    // offset += 2;
    // message.timestamp = (uint32_t)buffer[offset] | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);
    // offset += 4;
    // message.battery = buffer[offset++];
    message.endDelimiter = buffer[size - 1];

    // Validate the message
    if (message.startDelimiter != START_DELIMITER || message.endDelimiter != END_DELIMITER) {
        // Debugging: Print out the values of the start and end delimiters
        uart_send_string(uart_handle, "Error: Invalid delimiters\r\n");
        char debug_message[50];
        snprintf(debug_message, sizeof(debug_message), "Start Delimiter: 0x%02X, End Delimiter: 0x%02X\r\n", message.startDelimiter, message.endDelimiter);
        uart_send_string(uart_handle, debug_message);
        return;
    }

    // uint8_t calculatedChecksum = START_DELIMITER + message.sensorType + (message.timestamp & 0xFF) + ((message.timestamp >> 8) & 0xFF) + ((message.timestamp >> 16) & 0xFF) + ((message.timestamp >> 24) & 0xFF) + (message.sensorData & 0xFF) + ((message.sensorData >> 8) & 0xFF);
    // if (calculatedChecksum != message.checksum) {
    //     uart_send_string(uart_handle, "Error: Checksum mismatch\r\n");
    //     char debug_message[50];
    //     snprintf(debug_message, sizeof(debug_message), "CHECKSUM: 0x%02lX, EXPECTED: 0x%02X\r\n", message.checksum, calculatedChecksum);
    //     uart_send_string(uart_handle, debug_message);
    //     return;
    // }

    // Message is valid, proceed with Dash7 transmission
    // uint8_t payload[sizeof(message)];
    // payload[0] = message.startDelimiter;
    // memcpy(&payload[1], &message.timestamp, 4);
    // payload[5] = (message.sensorData >> 8) & 0xFF;
    // payload[6] = message.sensorData & 0xFF;

    // Generate ALP command.
    // We will be sending a return file data action, without a preceding file read request.
    // This is an unsolicited message, where we push the sensor data to the gateway(s).
    
    // alloc command. This will be freed when the command completes
    alp_command_t* command = alp_layer_command_alloc(false, false);
    
    // forward to the D7 interface
    alp_append_forward_action(command, (alp_interface_config_t*)&itf_config, d7ap_session_config_length(&itf_config.d7ap_session_config));

    // add the return file data action
    alp_append_return_file_data_action(command, SENSOR_FILE_ID, 0, size, buffer);

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
              uint8_t tempIndex= data_buffer_index;
              data_buffer_index = 0; // Reset buffer index for next message
              // Process the received data
              process_received_data(data_buffer, tempIndex);
          }
      } else {
          // Buffer overflow, reset buffer index
          data_buffer_index = 0;
          // Echo the entire data buffer for debugging (as hexadecimal)
          char* ptr = global_echo_buffer;
          for (int i = 0; i < data_buffer_index; i++) {
              ptr += snprintf(ptr, 4, "%02X ", data_buffer[i]);
          }
          uart_send_string(uart_handle, global_echo_buffer);
          uart_send_string(uart_handle, "Error: Buffer overflow\r\n");
          

      }
    } else {
        uart_send_string(uart_handle, "Error: Transmission not yet started\r\n");
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
      char logBuffer[96];
      //write the log print string and data ove ruart aswell
      sprintf(logBuffer, "recv response @ %i dB link budget from: \n%d", d7_result->rx_level, *(d7_result->addressee.id));
      uart_send_string(uart_handle, logBuffer);
  }
  log_print_string("response payload:");
  log_print_data(alp_command->alp_command, fifo_get_size(&alp_command->alp_command_fifo));
  fifo_skip(&alp_command->alp_command_fifo, fifo_get_size(&alp_command->alp_command_fifo));
}

static void file_modified_callback(uint8_t file_id)
{
    uart_send_string(uart_handle, "Instruction file modified\r\n");
    uint8_t instruction_file_data[INSTRUCTION_FILE_SIZE];
    uint32_t instruction_file_size = INSTRUCTION_FILE_SIZE;
    d7ap_fs_read_file(INSTRUCTION_FILE_ID, 0, instruction_file_data, &instruction_file_size, ROOT_AUTH);
    if(instruction_file_data[0] == START_DELIMITER && instruction_file_data[INSTRUCTION_FILE_SIZE - 1] == END_DELIMITER) {
        // Send the instruction file data over UART
        uart_send_bytes(uart_handle, (uint8_t*) instruction_file_data, INSTRUCTION_FILE_SIZE);
        // sp_handle.driver->serial_protocol_transfer_bytes(&sp_handle ,instruction_file_data, INSTRUCTION_FILE_SIZE, SERIAL_MESSAGE_TYPE_ALP_DATA);
    } else {
        uart_send_string(uart_handle, "Error: Invalid instruction file data\r\n");
    }
}

void bootstrap() {
    log_print_string("Device booted\n");
    d7ap_fs_init();
    d7ap_init();

    alp_layer_init(&alp_init_args, false);
    d7ap_fs_init_file(INSTRUCTION_FILE_ID, &instruction_file_header, NULL);
    // register a callback for when the sensor file is modified
    d7ap_fs_register_file_modified_callback(INSTRUCTION_FILE_ID, &file_modified_callback);
    // already trigger this callback to ensure we're already in the right state
    // file_modified_callback(INSTRUCTION_FILE_ID);

    d7ap_fs_init_file(SENSOR_FILE_ID, &sensor_file_header, NULL);


    // activate low power listening
    d7ap_fs_write_dll_conf_active_access_class(0x11); // use scanning AC, visible in d7ap_fs_data.c

    // serial_protocol_init(&sp_handle ,PLATFORM_MODEM_INTERFACE_UART, PLATFORM_MODEM_INTERFACE_BAUDRATE, true, PLATFORM_MODEM_INTERFACE_UART_STATE_PIN, PLATFORM_MODEM_INTERFACE_TARGET_UART_STATE_PIN, true, PLATFORM_MODEM_INTERFACE_DMA_RX ,PLATFORM_MODEM_INTERFACE_DMA_TX);

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
