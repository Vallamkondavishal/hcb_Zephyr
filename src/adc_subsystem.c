#include "adc_subsystem.h"

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/printk.h>

#include <version.h>
#include <assert.h>

// SHELL STUFF
#include <shell/shell.h>
#include <stdlib.h>

// GPIO for chip select
// PIN defines: should be in DTS
#define MCU_ADC_CS1_PORT "GPIOC"
#define MCU_ADC_CS1_PIN	 5

#define MCU_ADC_CS2_PORT "GPIOB"
#define MCU_ADC_CS2_PIN	 10

#define MCU_ADC_CS3_PORT "GPIOB"
#define MCU_ADC_CS3_PIN	 4

#define MCU_DAC_CS_PORT "GPIOC"
#define MCU_DAC_CS_PIN	4

#define MCU_ADC_RST_PORT "GPIOA"
#define MCU_ADC_RST_PIN	 4

// ADS8668 COMMANDS
#define ADS8668_CMD_NO_OP    0x00
#define ADS8668_CMD_STANDBY  0x82
#define ADS8668_CMD_PWR_DOWN 0x83
#define ADS8668_CMD_RESET    0x85
#define ADS8668_CMD_AUTO_RST 0xA0
#define ADS8668_CMD_MAN_CH_0 0xC0
#define ADS8668_CMD_MAN_CH_1 0xC4
#define ADS8668_CMD_MAN_CH_2 0xC8
#define ADS8668_CMD_MAN_CH_3 0xCC
#define ADS8668_CMD_MAN_CH_4 0xD0
#define ADS8668_CMD_MAN_CH_5 0xD4
#define ADS8668_CMD_MAN_CH_6 0xD8
#define ADS8668_CMD_MAN_CH_7 0xDC

// Manual Channel Selection Command
static const uint8_t chMap[] = {
	/* CH_0*/ ADS8668_CMD_MAN_CH_0,
	/* CH_1*/ ADS8668_CMD_MAN_CH_1,
	/* CH_2*/ ADS8668_CMD_MAN_CH_2,
	/* CH_3*/ ADS8668_CMD_MAN_CH_3,
	/* CH_4*/ ADS8668_CMD_MAN_CH_4,
	/* CH_5*/ ADS8668_CMD_MAN_CH_5,
	/* CH_6*/ ADS8668_CMD_MAN_CH_6,
	/* CH_7*/ ADS8668_CMD_MAN_CH_7,
};

static inline uint8_t getManualCommand(channel_t ch)
{
	return chMap[ch];
}

// CHannel Ranges
#define ADS8668_CH_RANGE_0 0x05
#define ADS8668_CH_RANGE_1 0x06
#define ADS8668_CH_RANGE_2 0x07
#define ADS8668_CH_RANGE_3 0x08
#define ADS8668_CH_RANGE_4 0x09
#define ADS8668_CH_RANGE_5 0x0A
#define ADS8668_CH_RANGE_6 0x0B
#define ADS8668_CH_RANGE_7 0x0C

// Manual Channel Selection Command
static const uint8_t chRangeMap[] = {
	/* CH_0*/ ADS8668_CH_RANGE_0,
	/* CH_1*/ ADS8668_CH_RANGE_1,
	/* CH_2*/ ADS8668_CH_RANGE_2,
	/* CH_3*/ ADS8668_CH_RANGE_3,
	/* CH_4*/ ADS8668_CH_RANGE_4,
	/* CH_5*/ ADS8668_CH_RANGE_5,
	/* CH_6*/ ADS8668_CH_RANGE_6,
	/* CH_7*/ ADS8668_CH_RANGE_7,
};

static inline uint8_t getRangeReg(channel_t ch)
{
	return chRangeMap[ch];
}

/***/

static const struct device *ad1_cs = 0;
static const struct device *ad2_cs = 0;
static const struct device *ad3_cs = 0;
static const struct device *dac_cs = 0;
static const struct device *adc_rst = 0;

#define ADS8668_WRITE 0x1

struct adc_description {
	const struct device *device;
	uint32_t cs_pin;
};

static struct adc_description get_device(enum adc_t adc)
{
	struct adc_description desc = { NULL, 0 };
	switch (adc) {
	case AD_1:
		desc.cs_pin = MCU_ADC_CS1_PIN;
		desc.device = ad1_cs;
		break;
	case AD_2:
		desc.cs_pin = MCU_ADC_CS2_PIN;
		desc.device = ad2_cs;
		break;
	case AD_3:
		desc.cs_pin = MCU_ADC_CS3_PIN;
		desc.device = ad3_cs;
		break;
	default:
		break;
	}
	return desc;
}

//**********************************************************
// SPI
//**********************************************************
static int stm32_spi_send(struct device *spi, const struct spi_config *spi_cfg,
			  const uint8_t *data, size_t len)
{
	const struct spi_buf_set tx = {
		.buffers =
			&(const struct spi_buf){
				.buf = (uint8_t *)data,
				.len = len,
			},
		.count = 1,
	};

	return spi_write(spi, spi_cfg, &tx);
}

static uint8_t rxmsg[2];
static struct spi_buf rx = {
	.buf = (uint8_t *)rxmsg,
	.len = 2,
};
static const struct spi_buf_set rx_bufs = {
	.buffers = &rx,
	.count = 1,
};

struct spi_config spi_cfg = {};
struct device *spi;

//**********************************************************

static int adc_command(enum adc_t adc, uint8_t command)
{
	struct adc_description adc_dev = get_device(adc);
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);
	uint8_t buf[] = { command, 0x00 };
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);
	return err;
}

/**
 * @brief
 *
 */
void adc_init()
{
	// Power ON devices
	adc_rst = device_get_binding(MCU_ADC_RST_PORT);
	gpio_pin_configure(adc_rst, MCU_ADC_RST_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(adc_rst, MCU_ADC_RST_PIN, 1);

	// CONFIG CHIP SELECTS
	ad1_cs = device_get_binding(MCU_ADC_CS1_PORT);
	gpio_pin_configure(ad1_cs, MCU_ADC_CS1_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(ad1_cs, MCU_ADC_CS1_PIN, 1);

	ad2_cs = device_get_binding(MCU_ADC_CS2_PORT);
	gpio_pin_configure(ad2_cs, MCU_ADC_CS2_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(ad2_cs, MCU_ADC_CS2_PIN, 1);

	ad3_cs = device_get_binding(MCU_ADC_CS3_PORT);
	gpio_pin_configure(ad3_cs, MCU_ADC_CS3_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(ad3_cs, MCU_ADC_CS3_PIN, 1);

	dac_cs = device_get_binding(MCU_DAC_CS_PORT);
	gpio_pin_configure(dac_cs, MCU_DAC_CS_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	// SPI initialization

	spi = (struct device *)device_get_binding("SPI_1");
	if (!spi) {
		printk("Could not find SPI driver\n");
		return;
	}

	spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA |
			    SPI_OP_MODE_MASTER;

	spi_cfg.frequency = 1200000U;

	k_sleep(K_MSEC(10));

	spi_cfg.cs = NULL;
}

/**
 * @brief
 *
 */
void adc_hw_reset()
{
	gpio_pin_set(adc_rst, MCU_ADC_RST_PIN, 0);
}

/**
 * @brief
 *
 * @param adc
 * @param reg
 * @return uint8_t
 */
uint8_t ads8668_read_reg(adc_t adc, uint8_t reg)
{
	struct adc_description adc_dev = get_device(adc);

	uint8_t buf[] = { (reg << 1) & 0xfe, 0xaa };
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));
	int res = spi_read(spi, &spi_cfg, &rx_bufs);
	(void) res;
	(void) err;
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);

	return ((uint8_t *)(rx_bufs.buffers->buf))[0];
}

/**
 * @brief Acquire a channel in manual mode
 * 
 * @param adc the adc
 * @param ch the channel
 * @return int 0 if ok 
 */
int ads8668_acquire(enum adc_t adc, uint8_t ch)
{
	struct adc_description adc_dev = get_device(adc);
	uint8_t channelAddress = getManualCommand(ch);

	uint8_t buf[] = { channelAddress, 0x00 };
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));
	(void) err; // TODO
	int res = spi_read(spi, &spi_cfg, &rx_bufs);
	(void) res; // TODO
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);

	return (((((uint8_t *)(rx_bufs.buffers->buf))[0] << 4) & 0x0ff0) |
		((0x00f0 & ((uint8_t *)(rx_bufs.buffers->buf))[1]) >> 4));
}

/**
 * @brief 
 * 
 * @param adc 
 * @param ch 
 * @param ch_range 
 * @return int 
 */
int ads8668_set_range(adc_t adc, uint8_t ch, ch_range_t ch_range)
{
	return ads8668_write_reg(adc, getRangeReg(ch), ch_range);
}

/**
 * @brief
 * 
 * @param adc
 * @param reg
 * @param data
 */
int ads8668_write_reg(enum adc_t adc, uint8_t reg, uint8_t data)
{
	struct adc_description adc_dev = get_device(adc);

	uint8_t buf[] = { (reg << 1) | ADS8668_WRITE, data };
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));
	int res = spi_read(spi, &spi_cfg, &rx_bufs);
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);

	(void) res;
	return err;
}

int ads8668_reset_command(enum adc_t adc)
{
	return adc_command(adc, ADS8668_CMD_RESET);
}

int ads8668_standby_command(enum adc_t adc)
{
	return adc_command(adc, ADS8668_CMD_STANDBY);
}

// SHELL
static int cmd_write(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int reg = strtoul(argv[2], NULL, 0);
	unsigned long int data = strtoul(argv[3], NULL, 0);

	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;
	}

	ads8668_write_reg(ad, reg, data);

	return 0;
}

static int cmd_range(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int ch = strtoul(argv[2], NULL, 0);
	unsigned long int range = strtoul(argv[3], NULL, 0);

	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;
	}

	ads8668_set_range(ad, ch, range);

	return 0;
}

static int cmd_read(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;

	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int reg = strtoul(argv[2], NULL, 0);

	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;
	}

	uint8_t result = ads8668_read_reg(ad, reg);

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "ad[%d] - reg 0x%02x: 0x%02x\n", ad, reg, result);

	return 0;
}

static int cmd_acq(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int ch = strtoul(argv[2], NULL, 0);

	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;
	}

	int result = ads8668_acquire(ad, ch);

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "acquire ad[%d] - ch 0x%02x: 0x%02x\n", ad, ch, result);

	return 0;
}

static int cmd_reset(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;

	unsigned long int ad = strtoul(argv[1], NULL, 0);

	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;
	}

	uint8_t result = ads8668_reset_command(ad);

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "Reset ad[%d] - 0x%02x\n", ad, result);

	return 0;
}

// DAC STUFF

#define MAX5714_CLEAR 0x50
#define MAX5714_RESET 0x51

int max5714_set_value(max5714_cmd_mode_t mode, max5714_output_t output,
		      int value)
{
	if (value > 4095)
		return -1;

	uint8_t lsn = (value & 0xf) << 4;
	uint8_t msb = value >> 4 & 0xff;

	uint8_t cmd = mode | output;

	char buf[4] = {};
	buf[0] = cmd;
	buf[1] = msb;
	buf[2] = lsn;
	buf[3] = 0x0;
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 0);
	int err = stm32_spi_send(spi, &spi_cfg, buf, 4);
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	return err;
}

int max5714_clear()
{
	char buf[4] = {};
	buf[0] = MAX5714_CLEAR;
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 0);
	int err = stm32_spi_send(spi, &spi_cfg, buf, 4);
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	return err;
}

int max5714_reset()
{
	char buf[4] = {};
	buf[0] = MAX5714_RESET;
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 0);
	int err = stm32_spi_send(spi, &spi_cfg, buf, 4);
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	return err;
}

/**
 * @brief drive a DAC output
 * uses MAX5714_CMD_CODEn_LOADn to mode to do it
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_dac_set(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;

	unsigned long int output = strtoul(argv[1], NULL, 0);

	unsigned long int value = strtoul(argv[2], NULL, 0);

	int err = max5714_set_value(MAX5714_CMD_CODEn_LOADn, output, value);

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "Set output %d using %d - argc %d\n", output, value);

	return err;
}

static int cmd_clear(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	int err = max5714_clear();

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Dac Clear\n");

	return err;
}

static int cmd_dac_reset(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	int err = max5714_reset();

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Dac Reset\n");

	return err;
}

SHELL_STATIC_SUBCMD_SET_CREATE(adc_cmds,
			       SHELL_CMD_ARG(read, NULL,
					     "<ad_num> <reg>"
					     "",
					     cmd_read, 3, 1),
			       SHELL_CMD_ARG(write, NULL,
					     "<ad_num> <reg> <value>"
					     "",
					     cmd_write, 4, 1),
			       SHELL_CMD_ARG(reset, NULL,
					     "<ad_num> <reg>"
					     "",
					     cmd_reset, 2, 1),
			       SHELL_CMD_ARG(range, NULL,
					     "<ad_num> <ch> <range>"
					     "",
					     cmd_range, 3, 1),
			       SHELL_CMD_ARG(acq, NULL,
					     "<ad_num> <ch>"
					     "",
					     cmd_acq, 3, 1),

			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(adc, &adc_cmds, "AD6886 shell commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(dac_cmds,
			       SHELL_CMD_ARG(set, NULL,
					     "<output> <val>"
					     "",
					     cmd_dac_set, 3, 1),
			       SHELL_CMD_ARG(clear, NULL,
					     ""
					     "",
					     cmd_clear, 1, 0),
			       SHELL_CMD_ARG(reset, NULL,
					     ""
					     "",
					     cmd_dac_reset, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dac, &dac_cmds, "DAC shell commands", NULL);
