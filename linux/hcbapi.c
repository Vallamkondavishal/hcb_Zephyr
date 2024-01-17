#include "hcbapi.h"

#define _POSIX_C_SOURCE 200809L

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "arpa/inet.h"
#include "errno.h"
#include "fcntl.h"
#include "pthread.h"
#include "signal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "strings.h"
#include "sys/ipc.h"
#include "sys/msg.h"
#include "sys/sem.h"
#include "sys/shm.h"
#include "sys/socket.h"
#include "sys/stat.h"
#include "sys/types.h"
#include "sys/wait.h"
#include "unistd.h"
#include <assert.h>
#include <math.h>

#include <linux/hiddev.h>
#include <sys/ioctl.h>

#include <libudev.h>

#include "hcb_protocol.h"
#include "hidapi/hidapi.h"
#include <time.h>

uint64_t current_ms(void)
{
  // Variables to store milliseconds (ms), seconds (s), and a timespec structure
  long ms;  // Milliseconds
  time_t s; // Seconds
  struct timespec spec;
  
  // Get the current time using CLOCK_REALTIME clock
  clock_gettime(CLOCK_REALTIME, &spec);

  s = spec.tv_sec;
  ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds

   // If milliseconds exceed 999, increment seconds and reset milliseconds to 0
  if (ms > 999)
  {
    s++;
    ms = 0;
  }

  // Return the total time in milliseconds (seconds * 1000 + milliseconds)
  return s * 1000 + ms;
}

/**
 * @brief CRC 8 Calculation Stuff
 *
 */
static const uint8_t CRC_TABLE[256] = { 0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B,
  0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77, 0x7E,
  0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4,
  0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF,
  0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5,
  0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE,
  0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27,
  0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04,
  0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61,
  0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
  0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5,
  0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E,
  0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43,
  0x44, 0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28,
  0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76,
  0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25,
  0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0,
  0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA,
  0xFD, 0xF4, 0xF3 };

static uint8_t crc8ccitt(const void* data, size_t size)
{
  // Initialize the CRC value to 0
  uint8_t val = 0;
  
  // Cast the data pointer to a uint8_t pointer
  uint8_t* pos = (uint8_t*)data;

  // Calculate the end pointer based on the size of the data
  uint8_t* end = pos + size;

  // Iterate over each byte in the data
  while (pos < end)
  {
    // XOR the current CRC value with the value from the CRC_TABLE
    val = CRC_TABLE[val ^ *pos];

    // Move to the next byte in the data
    pos++;
  }
  
  // Return the final CRC value
  return val;
}




//===============================================================================

/**
 * @brief Sends a "hello" command to the device.
 *
 * @param [in]  Handle File descriptor for the device
 * @param [out] Version the protocol version
 * @return int HCB_RC_OK if ok
 */
int hcb_send_hello(void* handle)
{
  // Create a hello command message
  hcb_msg_command_t message = { .header.report_id = 2,
    .header.message_id = HCB_ID_COMMAND,
    .header.byte_count = sizeof(hcb_msg_command_t),
    .header.crc = 0 };

  // Set the command type to HCB_CMD_HELLO
  message.command = HCB_CMD_HELLO;
  
  // Convert the message to a protocol structure
  hcb_protocol_t* ptr = (hcb_protocol_t*)&message;
  
  // Calculate CRC (Cyclic Redundancy Check) for the message
  uint8_t crc2 = crc8ccitt(ptr->raw, ptr->header.byte_count);
  message.header.crc = crc2;
  
  // Send the message to the device using the HID (Human Interface Device) write function
  return hid_write(handle, ptr->raw, message.header.byte_count);
}





int hcb_set(void* handle, uint32_t* val)
{
  // Step 1: Create a set point message
  hcb_msg_set_t message = { .header.report_id = 2,
    .header.message_id = HCB_ID_SET_POINT,
    .header.byte_count = sizeof(hcb_msg_set_t),
    .header.crc = 0 };

  // Step 2: Copy set point values to the message
  for (int i = 0; i < 8; ++i)
  {
    message.val[i] = val[i];
  }

  // Step 3: Calculate CRC (Cyclic Redundancy Check) for the message
  hcb_protocol_t* ptr = (hcb_protocol_t*)&message;

  uint8_t crc2 = crc8ccitt(ptr->raw, ptr->header.byte_count);
  message.header.crc = crc2;

  // Step 4: Send the set point message using the hid_write function
  return hid_write(handle, ptr->raw, message.header.byte_count);
}





/**
 * @brief                 File descriptor for the HID device.
 *
 * @param [in] handle     File descriptor for the HID device.
 * @return [out] int    
 */
int hcb_read_packet(void* handle, void* buf)
{

  // Step 1: Read a packet from the HID device
  int res
      = hid_read(handle, ((hcb_protocol_t*)buf)->raw, sizeof(hcb_protocol_t));
  if (res <= 0)
    return res;

  // Step 2: Validate CRC (Cyclic Redundancy Check)
  uint8_t crc8 = ((hcb_protocol_t*)buf)->header.crc;
  ((hcb_protocol_t*)buf)->header.crc = 0;
  ((hcb_protocol_t*)buf)->header.crc = crc8ccitt(
      ((hcb_protocol_t*)buf)->raw, ((hcb_protocol_t*)buf)->header.byte_count);

  // Step 3: Compare the calculated CRC with the received CRC
  if (crc8 == ((hcb_protocol_t*)buf)->header.crc)
    return 1;

  return 0;
}





/**
 * @brief         Opens a connection to the HID device.
 *
 * @param vendor_id   Vendor ID of the HID device.
 * @param product_id  Product ID of the HID device.
 * @return int
 */
void* hcb_open(unsigned short vendor_id, unsigned short product_id)
{
  return hid_open(0x2fe3, 0x0100, NULL);
}





/**
 * @brief        Closes the connection to the HID device.
 *
 * @param fd
 * @return int
 */

// Closes the connection to the HID device using the hid_close function.
void hcb_close(void* handle)
{
  hid_close(handle);
}





/**
 * @brief          Retrieves ADC (Analog-to-Digital Converter) data from the HID device.
 *
 * @param dev
 * @param adc      Array to store the ADC data.
 * @param len      Length of the ADC data array.
 * @return int     Returns 0 (not implemented).
 */
int hcb_get_adc(int fd, uint16_t* adc, int len)
{
  return 0;
}
