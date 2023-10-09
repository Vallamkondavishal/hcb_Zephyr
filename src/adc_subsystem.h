// Stuff to control the ADC - for now just user space
#ifndef _ADC_SUBSYSTEM_H_
#define _ADC_SUBSYSTEM_H_

#include <stdint.h>

typedef enum adc_t { AD_1, AD_2, AD_3, AD_MAX } adc_t;

typedef enum channel_t {
	ADS868_CH_1=0,
	ADS868_CH_2,
	ADS868_CH_3,
	ADS868_CH_4,
	ADS868_CH_5,
	ADS868_CH_6,
	ADS868_CH_7,
	ADS868_CH_8,
	ADS868_CH_MAX
} channel_t;

typedef enum ch_range_t {
	ADS868_RANGE_2_50_B = 0,
	ADS868_RANGE_1_25_B = 1,
	ADS868_RANGE_0_62_B = 2,
	ADS868_RANGE_0_31_B = 3,
	ADS868_RANGE_0_15_B = 11,
	ADS868_RANGE_2_50_M = 5,
	ADS868_RANGE_1_25_M = 6,
	ADS868_RANGE_0_62_M = 7,
	ADS868_RANGE_0_31_M = 15,
} ch_range_t;

void adc_init();
void adc_hw_reset();

// ADS8668 commands
int ads8668_reset_command(enum adc_t adc);
int ads8668_standby_command(enum adc_t adc);

int ads8668_acquire(adc_t adc, uint8_t ch);
int ads8668_set_range(adc_t adc, uint8_t ch, ch_range_t ch_range);

uint8_t ads8668_read_reg(adc_t adc, uint8_t reg);
int ads8668_write_reg(adc_t adc, uint8_t reg, uint8_t data);

// MAX5714 command
typedef enum max5714_output_t {
	MAX5714_DAC_A = 0x00,
	MAX5714_DAC_B = 0x01,
	MAX5714_DAC_C = 0x02,
	MAX5714_DAC_D = 0x03,
	MAX5714_DAC_ALL = 0x0f,
} max5714_output_t;

typedef enum max5714_cmd_mode_t {
	MAX5714_CMD_CODEn = 0x00,
	MAX5714_CMD_LOADn = 0x10,
	MAX5714_CMD_CODEn_LOAD_ALL = 0x20,
	MAX5714_CMD_CODEn_LOADn = 0x30,
} max5714_cmd_mode_t;

int max5714_set_value(max5714_cmd_mode_t mode, max5714_output_t output,
		      int value);
int max5714_clear();
int max5714_reset();

#endif // _ADC_SUBSYSTEM_H_