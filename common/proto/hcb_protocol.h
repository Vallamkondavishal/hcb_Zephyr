#ifndef HCB_PROTOCOL_H_
#define HCB_PROTOCOL_H_

#include <stdint.h>

// ZEPHYR COMPATIBILITY STUFF
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

// ***

#define STATIC_MSG(msg, l) STATIC_MSG2(msg, l)
#define STATIC_MSG2(msg, l) on_line_##l##__##msg
#define STATIC_ASSERT(x, msg)                                                  \
  extern char STATIC_MSG(msg, __LINE__)[(x) ? 1 : -1]

#define HCB_REPORT_LEN 63

#define HCB_REPORT_ID_IN_REPLY 1
#define HCB_REPORT_ID_OUT_REQUEST 2
#define HCB_REPORT_ID_IN_UNSOLICITED 3

typedef enum hcb_commands_t
{
  HCB_CMD_HELLO,
  HCB_CMD_READ_ADC,
  HCB_FT_BIAS,
} hcb_commands_t;

typedef struct hcb_proto_header_t
{
  uint8_t report_id;
  uint8_t message_id;
  uint8_t byte_count;
  uint8_t crc;
} __packed hcb_proto_header_t;

#define HCB_REPORT_PAYLOAD (HCB_REPORT_LEN - sizeof(hcb_proto_header_t))

typedef struct hcb_msg_command_t
{
  hcb_proto_header_t header;
  uint32_t command;
} __packed hcb_msg_command_t;

typedef struct hcb_msg_hello_reply_t
{
  hcb_proto_header_t header;
  uint32_t ver; // PROTOCOL VERSION
} __packed hcb_msg_hello_reply_t;

typedef struct hcb_msg_adc_t
{
  hcb_proto_header_t header;
  uint32_t timestamp;
  uint16_t val[24];
} __packed hcb_msg_adc_t;
STATIC_ASSERT(sizeof(hcb_msg_adc_t) <= HCB_REPORT_PAYLOAD, MSG_TOO_BIG);


typedef struct hcb_msg_ft_t
{
  hcb_proto_header_t header;
  uint32_t timestamp;
  uint32_t val[6];
} __packed hcb_msg_ft_t;
STATIC_ASSERT(sizeof(hcb_msg_ft_t) <= HCB_REPORT_PAYLOAD, MSG_TOO_BIG);

typedef struct hcb_msg_set_t
{
  hcb_proto_header_t header;
  uint32_t val[8]; // Period usec
} __packed hcb_msg_set_t;
STATIC_ASSERT(sizeof(hcb_msg_set_t) <= HCB_REPORT_PAYLOAD, MSG_TOO_BIG);

typedef union hcb_protocol_t
{
  hcb_proto_header_t header;
  hcb_msg_command_t cmd;
  hcb_msg_hello_reply_t hello_reply;
  hcb_msg_adc_t adc;
  hcb_msg_set_t set;
  hcb_msg_ft_t ft;
  uint8_t raw[HCB_REPORT_LEN];
} __packed hcb_protocol_t;

typedef enum hcb_msg_id_t
{
  HCB_ID_COMMAND = 0x10,
  HCB_ID_HELLO_REPLY,
  HCB_ID_ADC,
  HCB_ID_UNSOLICITED_TIME,
  HCB_ID_SET_POINT,
  HCB_ID_UNSOLICITED_TIME_2,
  HCB_ID_FT,
  HCB_ID_MAX,
} hcb_msg_id_t;

#define HCB_PROTOCOL_VER 43

#endif /* HCB_PROTOCOL_H_ */