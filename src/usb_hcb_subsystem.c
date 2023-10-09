/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>

LOG_MODULE_REGISTER(usb_transport, LOG_LEVEL_INF);

#include <zephyr.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>
#include <drivers/gpio.h>
#include <sys/crc.h>

// SHELL STUFF
#include <shell/shell.h>
#include <stdlib.h>

#include "usb_hcb_subsystem.h"

typedef struct msg_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	hcb_protocol_t msg;
} msg_data_t;

// FIFO DEFINITION
K_FIFO_DEFINE(trasmission_fifo);
// SEMAPHORE
K_SEM_DEFINE(my_sem, 0, 1);

/* callback function list */
static void usb_transport_host_ready(const struct device *dev);

void int_out_ready_cb(const struct device *dev);

int usb_transport_send_reply(hcb_protocol_t *proto);

static const struct hid_ops usb_transport_callbacks = {
	.int_in_ready = usb_transport_host_ready,
	.int_out_ready = int_out_ready_cb,
};

/* create an HID report descriptor with input and output reports */
static const uint8_t usb_transport_hid_report_desc[] = {
	0x06, 0x00, 0xFF, // (GLOBAL) USAGE_PAGE         0xFF00 Vendor-defined
	0x09, 0x01, //   (LOCAL)  USAGE              0xFF000001
	0xA1, 0x01, // (MAIN)   COLLECTION         0x01 Application (Usage=0x0: Page=, Usage=, Type=) <-- Warning: USAGE type should be CA (Application)
	0x15, 0x00, //   (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0
	0x26, 0xFF, 0x00, //   (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)
	0x75, 0x08, //   (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field
	0x85, 0x01, //   (GLOBAL) REPORT_ID          0x01 (1)
	0x95, 0x50, //   (GLOBAL) REPORT_COUNT       0x40 (64) Number of fields
	0x09, 0x01, //   (LOCAL)  USAGE              0xFF000001
	0x81, 0x02, //   (MAIN)   INPUT              0x00000002 (64 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
	0x85, 0x02, //   (GLOBAL) REPORT_ID          0x02 (2)
	0x09, 0x01, //   (LOCAL)  USAGE              0xFF000001
	0x95, 0x20, //   (GLOBAL) REPORT_COUNT       0x20 (32) Number of fields
	0x91, 0x02, //   (MAIN)   OUTPUT             0x00000002 (64 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
	0xC0, // (MAIN)   END_COLLECTION     Application
};

static const struct device *hid_device;
static hcb_protocol_t hb1 = {};

// PROTOTYPES
int usb_transfer_msg(hcb_protocol_t *proto);

//
int send_msg_hello_reply()
{
	hb1.header.report_id = HCB_REPORT_ID_IN_REPLY;
	hb1.header.message_id = HCB_ID_HELLO_REPLY;
	hb1.header.crc = 0;
	hb1.header.byte_count = sizeof(hcb_msg_hello_reply_t);

	hb1.hello_reply.ver = HCB_PROTOCOL_VER;

	hb1.header.crc = crc8_ccitt(0x0, hb1.raw, hb1.header.byte_count);

	LOG_ERR("send_msg_hello_reply !");

	int ret = usb_transport_send_reply(&hb1);

	if (ret) {
		LOG_ERR("send_msg_hello_reply error %d", ret);
	}
	return ret;
}

int send_msg_adc()
{
	hb1.header.report_id = HCB_REPORT_ID_IN_REPLY;
	hb1.header.message_id = HCB_ID_ADC;
	hb1.header.crc = 0;
	hb1.header.byte_count = sizeof(hcb_msg_adc_t);

	for (int i = 0; i < 24; ++i) {
		hb1.adc.val[i] = i;
	}

	hb1.header.crc = crc8_ccitt(0x0, hb1.raw, hb1.header.byte_count);

	int ret = usb_transport_send_reply(&hb1);
	if (ret) {
		LOG_ERR("send_msg_adc error %d", ret);
	}

	return ret;
}

int send_adc_values()
{
	hb1.header.report_id = HCB_REPORT_ID_IN_UNSOLICITED;
	hb1.header.message_id = HCB_ID_ADC;
	hb1.header.crc = 0;
	hb1.header.byte_count = sizeof(hcb_msg_adc_t);

	get_adc_values(hb1.adc.val);

	hb1.header.crc = crc8_ccitt(0x0, hb1.raw, hb1.header.byte_count);

	int ret = usb_transport_send_reply(&hb1);
	return ret;
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_CONFIGURED:
		LOG_INF(" #USB_DC_CONFIGURED");
		k_sem_give(&my_sem);
		break;
	default:
		LOG_ERR("status CB %u", status);
		break;
	}
}

int send_ati_values()
{
	hb1.header.report_id = HCB_REPORT_ID_IN_UNSOLICITED;
	hb1.header.message_id = HCB_ID_FT;
	hb1.header.crc = 0;
	hb1.header.byte_count = sizeof(hcb_msg_adc_t);

	get_ft_values(hb1.ft.val);

	hb1.header.crc = crc8_ccitt(0x0, hb1.raw, hb1.header.byte_count);

	int ret = usb_transport_send_reply(&hb1);
	return ret;
}

/**
 * @brief
 *
 * @return int
 */
int usb_transport_init()
{
	const struct device *adc_rst = device_get_binding("GPIOA");
	int res = gpio_pin_configure(adc_rst, 10, GPIO_OUTPUT_ACTIVE);
	res = gpio_pin_set(adc_rst, 10, 1);

	int ret;

	hid_device = device_get_binding("HID_0");

	if (hid_device == NULL) {
		LOG_ERR("USB HID Device not found");
		return -ENODEV;
	}

	/* register a HID device and the callback functions */
	usb_hid_register_device(hid_device, usb_transport_hid_report_desc,
				sizeof(usb_transport_hid_report_desc),
				&usb_transport_callbacks);

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return -EIO;
	}
	/* initialize USB interface and HID class */
	return usb_hid_init(hid_device);
}

/**
 * @brief enqueue message for the
 *
 * @param proto
 * @return int
 */
int usb_transfer_msg(hcb_protocol_t *proto)
{
	size_t size = sizeof(struct msg_data_t);

	msg_data_t *mem_ptr = (msg_data_t *)k_malloc(size);
	__ASSERT_NO_MSG(mem_ptr != 0);

	memcpy(&mem_ptr->msg, proto, size);

	k_fifo_put(&trasmission_fifo, mem_ptr);
	return 0;
}

/**
 * @brief
 *
 */
void usb_transport_thread(void)
{
	while (1) {
		struct msg_data_t *rx_data =
			k_fifo_get(&trasmission_fifo, K_FOREVER);

		if (rx_data == NULL) {
			LOG_ERR(" Failed get");
			continue;
		}

		hcb_protocol_t *buf = &rx_data->msg;

		uint8_t message_id = ((hcb_proto_header_t *)buf)->message_id;

		// TODO add CRC check
		switch (message_id) {
		case HCB_ID_COMMAND: {
			uint32_t command = ((hcb_protocol_t *)buf)->cmd.command;
			if (command == HCB_CMD_HELLO) {
				send_msg_hello_reply();
			}
			if (command == HCB_CMD_READ_ADC) {
				send_msg_adc();
			}
		} break;
		case HCB_ID_UNSOLICITED_TIME: {
			send_adc_values();
		} break;
		case HCB_ID_UNSOLICITED_TIME_2: {
			send_ati_values();
		} break;
		case HCB_ID_SET_POINT: {
			set_setpoints(buf->set.val);
		} break;
		default:
			LOG_ERR(" RECEIVED other %d\n", message_id);
			break;
		}

		k_free(rx_data);
	}
}

void usb_unsolicited_stuff(void)
{
	static hcb_protocol_t buf = {};
	while (1) {
		buf.header.report_id = 2;
		buf.header.message_id = HCB_ID_UNSOLICITED_TIME;
		buf.header.crc = 0;
		buf.header.byte_count = sizeof(hcb_proto_header_t);

		int ret = usb_transfer_msg(&buf);
		if (ret) {
			LOG_ERR("usb_transfer_msg error %d", ret);
		}
		k_sleep(K_MSEC(100));
	
		buf.header.report_id = 2;
		buf.header.message_id = HCB_ID_UNSOLICITED_TIME_2;
		buf.header.crc = 0;
		buf.header.byte_count = sizeof(hcb_proto_header_t);
		 ret = usb_transfer_msg(&buf);
		if (ret) {
			LOG_ERR("usb_transfer_msg error %d", ret);
		}
		k_sleep(K_MSEC(100));
	}
}

int usb_transport_send_reply(hcb_protocol_t *proto)
{
	uint32_t written = 0;

	if (k_sem_take(&my_sem, K_USEC(333)) != 0) {
		return -1;
	}

	// dump_buf("Send:", proto->raw, proto->header.byte_count);
	/* send reply in one or more HID reports */
	// ************
	// 0x41 because windows drops report of the wrong size.
	// output reports are of the size of 0x40 plus the report number
	int ret = hid_int_ep_write(hid_device, (uint8_t *)proto->raw, proto->header.byte_count,
				   &written);

	if (ret) {
		LOG_INF("usb_write failed with error %d", ret);
	}

	if (written != proto->header.byte_count) {
		/* usb_write is expected to send all data */
		LOG_DBG("usb_write: requested %u sent %u",
			proto->header.byte_count, written);
		//ret = -EIO;
	}
	return ret;
}

static void usb_transport_host_ready(const struct device *dev)
{
	// Give the semaphore back to reenable transmission
	k_sem_give(&my_sem);
}

static uint8_t buf[128];

void int_out_ready_cb(const struct device *dev)
{
	uint32_t len = 0;
	hid_int_ep_read(hid_device, buf, 64, &len);
	usb_transfer_msg((hcb_protocol_t *)buf);
}
