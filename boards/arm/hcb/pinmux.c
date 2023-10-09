/*
 * Copyright (c) 2017 I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for HCB board */
static const struct pin_config pinconf[] = {
	{ STM32_PIN_PA0, (STM32_PINMUX_ALT_FUNC_1) },
	{ STM32_PIN_PA1, (STM32_PINMUX_ALT_FUNC_1) },

	{ STM32_PIN_PA2, (STM32_PINMUX_ALT_FUNC_1) },
	{ STM32_PIN_PA3, (STM32_PINMUX_ALT_FUNC_1) },


	{ STM32_PIN_PA8, (STM32_PINMUX_ALT_FUNC_6) },
	{ STM32_PIN_PA9, (STM32_PINMUX_ALT_FUNC_6) },

	{ STM32_PIN_PC6, (STM32_PINMUX_ALT_FUNC_4) },
	{ STM32_PIN_PC7, (STM32_PINMUX_ALT_FUNC_4) },


	{ STM32_PIN_PA5, (STM32_PINMUX_ALT_FUNC_5) }, // SPI1 - CLK
	{ STM32_PIN_PA6, (STM32_PINMUX_ALT_FUNC_5) }, // SPI1 - MISO
	{ STM32_PIN_PA7, (STM32_PINMUX_ALT_FUNC_5) }, // SPI1 - MOSI
	{ STM32_PIN_PB6,
	  (STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP) }, // UASRT1-TX
	{ STM32_PIN_PB7,
	  (STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP) }, // UASRT1-RX
	{ STM32_PIN_PB8, (STM32_PINMUX_ALT_FUNC_9) }, // CAN_RX
	{ STM32_PIN_PB9, (STM32_PINMUX_ALT_FUNC_9) }, // CAN_TX
#ifdef CONFIG_USB_DC_STM32
	{ STM32_PIN_PA11, (STM32_PINMUX_ALT_FUNC_14 | STM32_PUSHPULL_NOPULL) },
	{ STM32_PIN_PA12, (STM32_PINMUX_ALT_FUNC_14 | STM32_PUSHPULL_NOPULL) },
#endif /* CONFIG_USB_DC_STM32 */
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
	 CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
