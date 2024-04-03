/*
 * This section includes necessary headers for the ADC subsystem, GPIO, SPI, and the Zephyr RTOS.
 * It also includes headers for shell-related functionalities and standard C libraries.
 */
#include "adc_subsystem.h"  // Include header file for ADC subsystem

#include <zephyr.h>  // Include Zephyr OS header
#include <device.h>  // Include device header for device management
#include <drivers/gpio.h>  // Include GPIO driver header
#include <drivers/spi.h>  // Include SPI driver header
#include <sys/printk.h>  // Include printk header for printing

// SHELL STUFF
#include <version.h>  // Include version header for version information
#include <assert.h>  // Include assert header for assertion functionality

// SHELL STUFF
#include <shell/shell.h>  // Include shell header for shell functionality
#include <stdlib.h>  // Include stdlib header for standard library functions

/*
 * GPIO configurations for chip select pins and control pins.
 * These pin assignments are expected to be defined in the Device Tree Source (DTS).
 */
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
/*
 * Definitions of ADS8668 ADC commands, including NO_OP, STANDBY, POWER_DOWN, RESET,
 * AUTO_RESET, and manual channel selection commands (MAN_CH_0 to MAN_CH_7).
 */
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


/*
 * Mapping of manual commands to ADC channels (CH_0 to CH_7) and channel ranges.
 */
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

/*
 * Function to retrieve the manual command based on the ADC channel.
 */
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

/*
 * Function to retrieve the manual command based on the ADC channel range.
 */
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

/*
 * Function: get_device
 * ---------------------
 * Retrieves the device description based on the specified ADC type.
 *
 * adc: The enum representing the ADC type (AD_1, AD_2, or AD_3).
 *
 * returns: A struct adc_description containing the device pointer and chip select pin.
 */
static struct adc_description get_device(enum adc_t adc)
{
	// Initialize the device description with default values
	struct adc_description desc = { NULL, 0 };

	// Switch case to handle different ADC types and assign corresponding device and chip select pin
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
	    // Handle unexpected ADC type
		break;
	}
	// Return the device description
	return desc;
}

//**********************************************************
// SPI
//**********************************************************

/*
 * Function: stm32_spi_send
 * -------------------------
 * Sends data over SPI communication using the specified SPI device and configuration.
 *
 * spi: Pointer to the SPI device structure.
 * spi_cfg: Pointer to the SPI configuration structure.
 * data: Pointer to the data buffer to be transmitted.
 * len: Length of the data buffer.
 *
 * returns: Result of the SPI write operation.
 */
static int stm32_spi_send(struct device *spi, const struct spi_config *spi_cfg,
			  const uint8_t *data, size_t len)
{
    // Define a SPI buffer set for transmitting data
    const struct spi_buf_set tx = {
        .buffers =
            &(const struct spi_buf){
                .buf = (uint8_t *)data,  // Data buffer
                .len = len,              // Length of the data buffer
            },
        .count = 1,  // Number of buffers in the set (only one buffer in this case)
    };

    // Call the SPI write function to transmit the data
    return spi_write(spi, spi_cfg, &tx);
}

// Define a receive buffer and set for receiving data
static uint8_t rxmsg[2];  // Receive buffer
static struct spi_buf rx = {
    .buf = (uint8_t *)rxmsg,  // Data buffer for receiving
    .len = 2,                  // Length of the data buffer
};
static const struct spi_buf_set rx_bufs = {
    .buffers = &rx,  // Pointer to the receive buffer
    .count = 1,      // Number of buffers in the set (only one buffer in this case)
};

// Define a SPI configuration structure and SPI device pointer
struct spi_config spi_cfg = {};  // Initialize SPI configuration structure
struct device *spi;              // Pointer to the SPI device structure

//**********************************************************

/**
 * @brief Send a command to the ADC device.
 *
 * @param adc The ADC device to send the command to.
 * @param command The command byte to be sent.
 * @return 0 on success, negative errno code on failure.
 */
static int adc_command(enum adc_t adc, uint8_t command)
{
	// Get the ADC device description based on the specified ADC.
	struct adc_description adc_dev = get_device(adc);

	// Set the chip select pin to low to initiate communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);

	// Prepare the command byte to be sent along with a dummy byte for receiving data.
	uint8_t buf[] = { command, 0x00 };

	// Send the command bytes via SPI.
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));

	// Set the chip select pin back to high to end communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);

	// Return the result of the SPI transmission.
	return err;
}

/**
 * @brief Initializes the ADC and DAC devices.
 *
 * This function initializes the ADC and DAC devices by powering them on,
 * configuring their chip selects, and initializing the SPI communication.
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

    // Delay for SPI initialization
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
 * @brief Reads a register from the ADS8668 ADC.
 *
 * This function reads a register from the ADS8668 ADC specified by the `adc` parameter.
 *
 * @param adc The ADC device to read from.
 * @param reg The register address to read.
 * @return The value read from the specified register.
 */
uint8_t ads8668_read_reg(adc_t adc, uint8_t reg)
{
	// Get the ADC device description based on the specified ADC.
	struct adc_description adc_dev = get_device(adc);

    // Prepare the command byte and a dummy byte for receiving data.
	uint8_t buf[] = { (reg << 1) & 0xfe, 0xaa };

	// Set the chip select pin to low to initiate communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);

    // Send the command bytes via SPI and read the response.
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));
	int res = spi_read(spi, &spi_cfg, &rx_bufs);


	(void) res;
	(void) err;

	// Set the chip select pin back to high to end communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);

    // Return the value read from the specified register.
	return ((uint8_t *)(rx_bufs.buffers->buf))[0];
}

/**
 * @brief Acquires a channel in manual mode.
 *
 * This function acquires data from the specified channel in manual mode.
 *
 * @param adc The ADC device.
 * @param ch The channel number to acquire.
 * @return 0 if acquisition is successful, otherwise an error code.
 */
int ads8668_acquire(enum adc_t adc, uint8_t ch)
{   
	// Get the ADC device description based on the specified ADC.
	struct adc_description adc_dev = get_device(adc);

	// Get the address for the specified channel in manual mode.
	uint8_t channelAddress = getManualCommand(ch);
    
	// Prepare the command byte and a dummy byte for receiving data.
	uint8_t buf[] = { channelAddress, 0x00 };

	// Set the chip select pin to low to initiate communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);

	// Send the command bytes via SPI and read the response.
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));

	// Ignore the error for now (TODO: handle errors properly)
	(void) err; // TODO
	int res = spi_read(spi, &spi_cfg, &rx_bufs);

	// Ignore the result for now (TODO: handle data properly)
	(void) res; // TODO

	// Set the chip select pin back to high to end communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);
    
	// Extract and return the acquired data from the response.
	return (((((uint8_t *)(rx_bufs.buffers->buf))[0] << 4) & 0x0ff0) |
		((0x00f0 & ((uint8_t *)(rx_bufs.buffers->buf))[1]) >> 4));
}

/**
 * @brief Sets the range for a specified channel.
 *
 * This function sets the range for the specified channel of the ADC.
 *
 * @param adc The ADC device.
 * @param ch The channel number to set the range for.
 * @param ch_range The range to set for the channel.
 * @return 0 if setting the range is successful, otherwise an error code.
 */
int ads8668_set_range(adc_t adc, uint8_t ch, ch_range_t ch_range)
{
	// Call the ads8668_write_reg function to write the range to the register.
    // Passes the ADC device, the register corresponding to the channel, and the specified range.
	return ads8668_write_reg(adc, getRangeReg(ch), ch_range);
}

/**
 * @brief Writes data to a register of the ADS8668 ADC.
 *
 * This function writes data to the specified register of the ADS8668 ADC.
 *
 * @param adc The ADC device.
 * @param reg The register address to write to.
 * @param data The data to write to the register.
 * @return 0 if writing to the register is successful, otherwise an error code.
 */
int ads8668_write_reg(enum adc_t adc, uint8_t reg, uint8_t data)
{
	// Get the ADC device description based on the specified ADC.
	struct adc_description adc_dev = get_device(adc);

    // Prepare the command byte and data byte for writing to the register.
	uint8_t buf[] = { (reg << 1) | ADS8668_WRITE, data };

	// Set the chip select pin to low to initiate communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 0);

	// Send the command bytes via SPI to write to the register.
	int err = stm32_spi_send(spi, &spi_cfg, buf, sizeof(buf));

	// Read the response (not used for writing operation).
	int res = spi_read(spi, &spi_cfg, &rx_bufs);

	// Set the chip select pin back to high to end communication.
	gpio_pin_set(adc_dev.device, adc_dev.cs_pin, 1);
    
	// Ignore the response and return the result of the SPI transmission.
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

/**
 * @brief Command handler for writing data to a register.
 *
 * This function handles the shell command to write data to a register of the ADS8668 ADC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the command is executed successfully, otherwise an error code.
 */
static int cmd_write(const struct shell *shell, size_t argc, char **argv)
{
	// Check the number of arguments passed to the command.
	(void)argc;

	// Parse the arguments to obtain the ADC, register, and data values.
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int reg = strtoul(argv[2], NULL, 0);
	unsigned long int data = strtoul(argv[3], NULL, 0);
    
	// Check if the ADC value is within the valid range.
	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;// Return error code if the ADC value is invalid.
	}
    
	// Write data to the specified register of the ADS8668 ADC.
	ads8668_write_reg(ad, reg, data);

	return 0;// Return success code.
}

/**
 * @brief Command handler for setting the range of a channel.
 *
 * This function handles the shell command to set the range of a channel for the ADS8668 ADC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the command is executed successfully, otherwise an error code.
 */
static int cmd_range(const struct shell *shell, size_t argc, char **argv)
{

	// Check the number of arguments passed to the command.
	(void)argc;

	// Parse the arguments to obtain the ADC, channel, and range values.
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int ch = strtoul(argv[2], NULL, 0);
	unsigned long int range = strtoul(argv[3], NULL, 0);

    // Check if the ADC value is within the valid range.
	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;// Return error code if the ADC value is invalid.
	}
    
	// Set the range of the specified channel for the ADS8668 ADC.
	ads8668_set_range(ad, ch, range);

	return 0;// Return success code.
}

/**
 * @brief Command handler for reading data from a register.
 *
 * This function handles the shell command to read data from a register of the ADS8668 ADC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the command is executed successfully, otherwise an error code.
 */
static int cmd_read(const struct shell *shell, size_t argc, char **argv)
{
	// Check the number of arguments passed to the command.
	(void)argc;

	// Parse the arguments to obtain the ADC and register values.     
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int reg = strtoul(argv[2], NULL, 0);
    
	// Check if the ADC value is within the valid range.
	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;// Return error code if the ADC value is invalid.
	}
    
	// Read data from the specified register of the ADS8668 ADC.
	uint8_t result = ads8668_read_reg(ad, reg);
    
	// Print the result to the shell.
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "ad[%d] - reg 0x%02x: 0x%02x\n", ad, reg, result);

	return 0;// Return success code.
}

/**
 * @brief Command handler for acquiring data from a channel.
 *
 * This function handles the shell command to acquire data from a channel of the ADS8668 ADC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the command is executed successfully, otherwise an error code.
 */
static int cmd_acq(const struct shell *shell, size_t argc, char **argv)
{
	// Check the number of arguments passed to the command.
	(void)argc;

	// Parse the arguments to obtain the ADC and channel values.
	unsigned long int ad = strtoul(argv[1], NULL, 0);
	unsigned long int ch = strtoul(argv[2], NULL, 0);
    
	// Check if the ADC value is within the valid range.
	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL;// Return error code if the ADC value is invalid.
	}
    
	// Acquire data from the specified channel of the ADS8668 ADC.
	int result = ads8668_acquire(ad, ch);
    
	// Print the result to the shell.
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "acquire ad[%d] - ch 0x%02x: 0x%02x\n", ad, ch, result);

	return 0;// Return success code.
}

/**
 * @brief Command handler for resetting the ADS8668 ADC.
 *
 * This function handles the shell command to reset the ADS8668 ADC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the command is executed successfully, otherwise an error code.
 */
static int cmd_reset(const struct shell *shell, size_t argc, char **argv)
{ 
	// Check the number of arguments passed to the command.
	(void)argc;
    
	// Parse the argument to obtain the ADC value.
	unsigned long int ad = strtoul(argv[1], NULL, 0);
    
	// Check if the ADC value is within the valid range.
	if (ad >= AD_MAX) {
		shell_error(shell, "Wrong ad %d", ad);
		return -EINVAL; // Return error code if the ADC value is invalid.
	}

    // Issue the reset command to the specified ADC.
	uint8_t result = ads8668_reset_command(ad);

    // Print the result to the shell.
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "Reset ad[%d] - 0x%02x\n", ad, result);

	return 0; // Return success code.
}

// DAC STUFF

#define MAX5714_CLEAR 0x50
#define MAX5714_RESET 0x51

/**
 * @brief Set the output value of the MAX5714 DAC.
 *
 * This function sets the output value of the MAX5714 DAC based on the provided parameters.
 *
 * @param mode Command mode for the DAC.
 * @param output Output channel of the DAC.
 * @param value Value to be set as the output voltage.
 * @return 0 if the operation is successful, otherwise an error code.
 */
int max5714_set_value(max5714_cmd_mode_t mode, max5714_output_t output,
		      int value)
{
	// Check if the value exceeds the maximum allowed value.
	if (value > 4095)
		return -1; // Return error if the value is out of range.
    
	// Extract the MSB and LSN from the value.
	uint8_t lsn = (value & 0xf) << 4;
	uint8_t msb = value >> 4 & 0xff;
    
	// Construct the command byte.
	uint8_t cmd = mode | output;
    
	// Prepare the data buffer.
	char buf[4] = {};
	buf[0] = cmd;
	buf[1] = msb;
	buf[2] = lsn;
	buf[3] = 0x0;

	// Select the DAC chip.
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 0);

	// Send the data over SPI.
	int err = stm32_spi_send(spi, &spi_cfg, buf, 4);

	// Deselect the DAC chip.
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	return err;// Return the result of the SPI transmission.
}


/**
 * @brief Clear the output of the MAX5714 DAC.
 *
 * This function clears the output of the MAX5714 DAC by sending a clear command over SPI.
 *
 * @return 0 if the operation is successful, otherwise an error code.
 */
int max5714_clear()
{
	// Prepare the command buffer.
	char buf[4] = {};
	buf[0] = MAX5714_CLEAR;

	// Select the DAC chip.
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 0);

	// Send the clear command over SPI.
	int err = stm32_spi_send(spi, &spi_cfg, buf, 4);

	// Deselect the DAC chip.
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	return err;// Return the result of the SPI transmission.
}

/**
 * @brief Reset the MAX5714 DAC.
 *
 * This function resets the MAX5714 DAC by sending a reset command over SPI.
 *
 * @return 0 if the operation is successful, otherwise an error code.
 */
int max5714_reset()
{
	// Prepare the command buffer.
	char buf[4] = {};
	buf[0] = MAX5714_RESET;

	 // Select the DAC chip.
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 0);

	// Send the reset command over SPI.
	int err = stm32_spi_send(spi, &spi_cfg, buf, 4);

	// Deselect the DAC chip.
	gpio_pin_set(dac_cs, MCU_DAC_CS_PIN, 1);

	return err;// Return the result of the SPI transmission.
}

/**
 * @brief Command handler to set a DAC output using the MAX5714 DAC.
 *
 * This function sets a DAC output using the MAX5714 DAC based on the provided parameters.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the operation is successful, otherwise an error code.
 */
static int cmd_dac_set(const struct shell *shell, size_t argc, char **argv)
{
	// Check the number of arguments passed to the command.
	(void)argc;
    // Parse the arguments to obtain the output and value.
	unsigned long int output = strtoul(argv[1], NULL, 0);
    unsigned long int value = strtoul(argv[2], NULL, 0);

    // Set the DAC output using the MAX5714 DAC.
	int err = max5714_set_value(MAX5714_CMD_CODEn_LOADn, output, value);
    
	// Print the result to the shell.
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
		      "Set output %d using %d - argc %d\n", output, value);

	return err; // Return the result of the operation.
}

/**
 * @brief Command handler to clear the DAC output.
 *
 * This function clears the output of the MAX5714 DAC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the operation is successful, otherwise an error code.
 */
static int cmd_clear(const struct shell *shell, size_t argc, char **argv)
{
	// Check the number of arguments passed to the command.
	(void)argc;
	(void)argv;
    
	// Clear the DAC output.
	int err = max5714_clear();
    
	// Print the result to the shell.
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Dac Clear\n");

	return err;  // Return the result of the operation.
}


/**
 * @brief Command handler to reset the MAX5714 DAC.
 *
 * This function resets the MAX5714 DAC.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of strings containing the arguments passed to the command.
 * @return 0 if the operation is successful, otherwise an error code.
 */
static int cmd_dac_reset(const struct shell *shell, size_t argc, char **argv)
{
	// Check the number of arguments passed to the command.
	(void)argc;
	(void)argv;
    
	// Reset the MAX5714 DAC.
	int err = max5714_reset();
    
	// Print the result to the shell.
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Dac Reset\n");

	return err; // Return the result of the operation.
}

/**
 * @brief Definition of subcommands for ADC shell commands.
 *
 * This macro creates a set of subcommands for ADC-related operations, including reading,
 * writing, resetting, setting range, and acquiring channels.
 */
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

/**
 * @brief Registration of ADC shell commands.
 *
 * This macro registers the ADC shell commands along with their corresponding subcommands.
 */
SHELL_CMD_REGISTER(adc, &adc_cmds, "AD6886 shell commands", NULL);

/**
 * @brief Definition of subcommands for DAC shell commands.
 *
 * This macro creates a set of subcommands for DAC-related operations, including setting
 * output, clearing, and resetting.
 */
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

/**
 * @brief Registration of DAC shell commands.
 *
 * This macro registers the DAC shell commands along with their corresponding subcommands.
 */
SHELL_CMD_REGISTER(dac, &dac_cmds, "DAC shell commands", NULL);
