#ifndef HCB_API_H_
#define HCB_API_H_

#include <wchar.h>     // Include wide-character handling functions
#include <stdint.h>    // Include standard integer types
#include <stddef.h>    // Include definitions for NULL and size_t

#ifdef __cplusplus
extern "C"
{
#endif

// Define error codes
#define HCB_RC_OK           0   // Operation successful
#define HCB_RC_GENERIC_ERR -1   // Generic error
#define HCB_RC_CRC_ERR     -2   // Error related to cyclic redundancy check
#define HCB_RC_TIMEOUT_ERR -3   // Error due to timeout
#define HCB_RC_VERSION_ERR -4   // Error related to version mismatch

// Function declarations for interacting with an HCB device

// Open the HCB device and return a handle
void* hcb_open(uint16_t vendor_id, uint16_t product_id);

// Close the HCB device using the provided handle
void hcb_close(void* handle);

// Send a hello message to the HCB device using the provided handle
int hcb_send_hello(void* handle);

// Set a value on the HCB device using the provided handle and value
int hcb_set(void* handle, uint32_t* val);

// Read a packet from the HCB device using the provided handle and message buffer
int hcb_read_packet(void* handle, void* msg);

// Get ADC (Analog-to-Digital Converter) data from the HCB device using the provided file descriptor, ADC array, and length
int hcb_get_adc(int fd, uint16_t* adc, int len);

#ifdef __cplusplus
}
#endif

#endif // HCB_API_H_