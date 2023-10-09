#ifndef HCB_API_H_
#define HCB_API_H_

#include <wchar.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define HCB_RC_OK 0
#define HCB_RC_GENERIC_ERR -1
#define HCB_RC_CRC_ERR -2
#define HCB_RC_TIMEOUT_ERR -3
#define HCB_RC_VERSION_ERR -4

  /** opaque device structure */
  void* hcb_open(uint16_t vendor_id, uint16_t product_id);
  void hcb_close(void* handle);

  int hcb_send_hello(void* handle);
  int hcb_set(void* handle, uint32_t* val);
  int hcb_read_packet(void* handle, void* msg);
  int hcb_get_adc(int fd, uint16_t* adc, int len);

#ifdef __cplusplus
}
#endif

#endif // HCB_API_H_