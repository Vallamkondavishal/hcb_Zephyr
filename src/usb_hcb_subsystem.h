/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TUNING_USB_TRANSPORT_H
#define TUNING_USB_TRANSPORT_H

#include "hcb_protocol.h"

typedef void (*usb_transport_receive_callback_t)(uint8_t *data, uint32_t len);

int usb_transport_init();
int usb_transport_send_reply(hcb_protocol_t *proto);
void usb_transport_thread(void);
void usb_unsolicited_stuff(void);
int send_msg_test();

#endif /* TUNING_USB_TRANSPORT_H */
