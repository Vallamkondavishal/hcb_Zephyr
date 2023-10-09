#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>

LOG_MODULE_REGISTER(p, LOG_LEVEL_INF);

#include "adc_subsystem.h"

#include <drivers/can.h>
#include <drivers/gpio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/pwm.h>

#include "usb_hcb_subsystem.h"

#define LED_PORT "GPIOC"
#define LED_PIN	 8

#define MIN_PERIOD_USEC (USEC_PER_SEC / 64U)
#define MAX_PERIOD_USEC USEC_PER_SEC
#define FILTER_LEN 8

#define Kp 0.3
#define Ki 0.7

#define K1 (Kp + Ki)
#define K2 -Kp

K_MUTEX_DEFINE(adcMutex);
K_MUTEX_DEFINE(pidMutex);
uint16_t adc_vals[24];
uint16_t set_points[8];
/**
 * @brief 
 * 
 */
static const uint8_t ch2axis[AD_MAX][ADS868_CH_MAX] = {{1,2,3,4,5,6,7,0},{1,2,3,4,5,6,7,0},{1,2,3,4,5,6,7,0}};
static const uint8_t enabled_adc[AD_MAX] = {1,0,1};

void acquire_adc()
{
	static uint16_t internal_adc_vals[FILTER_LEN][24]; // Lots of SPI transactions
	static int filter_idx = 0;
	for (adc_t ad = AD_1; ad < AD_MAX; ++ad){
		if(enabled_adc[ad] == 1){
			for (channel_t ch = ADS868_CH_1; ch < ADS868_CH_MAX; ++ch) {
				int idx = (ad * 8) + ch2axis[ad][ch];
				internal_adc_vals[filter_idx][idx] = ads8668_acquire(ad, ch);
			}
		}
	}
	filter_idx++;
	if (filter_idx >= FILTER_LEN)
		filter_idx = 0;
	k_mutex_lock(&adcMutex, K_FOREVER);
	//only first ADC for the pression and third fot the current
	for (int i  = 0; i < 24 ; i++){
		int acc = 0;
		if(i < 8 || i > 15){
			for( int j = 0 ; j < FILTER_LEN ; j ++){	
				acc += internal_adc_vals[j][i]; 
				
			}
		adc_vals[i] = acc/ FILTER_LEN;
		}
	}
	k_mutex_unlock(&adcMutex);
}

typedef struct pid{
  float n_1_u;
  float n_1_error;
  float error;
  float control;
}pid;


/**
 * @brief 
 * 
 */

void PID_Controller(pid *pid_current){
	
	uint16_t local_setpoints[8];
	uint16_t local_adcvals[24];
	
	//Copy in the local variables
	k_mutex_lock(&adcMutex, K_FOREVER);
	memcpy(local_setpoints,set_points,sizeof(set_points));
	memcpy(local_adcvals,adc_vals,sizeof(adc_vals));
	k_mutex_unlock(&adcMutex);
    
	for(int i = 0; i < ADS868_CH_MAX; i++) {

		
		if(local_setpoints[i] < 410) {
			pid_current[i].control = 0;
			continue;
		}
		
		//General sensor , constant value of on/off
		if(i == 6) pid_current[i].control = 5000;
		
		else{
			pid_current[i].error = local_setpoints[i] - local_adcvals[i];
			pid_current[i].control = pid_current[i].n_1_u + K1*pid_current[i].error + K2*pid_current[i].n_1_error;

			//Limite dei PWM settato 3000
			if(pid_current[i].control > 3000){
				pid_current[i].control = 3000;
			}
			if(pid_current[i].control <= 0){
				pid_current[i].control = 0;
			}
			pid_current[i].n_1_u = pid_current[i].control;
			pid_current[i].n_1_error = pid_current[i].error;
		}
		
	}

}


/**
 * @brief 
 * 
 */
void default_adc_range()
{
	for (adc_t ad = AD_1; ad < AD_MAX; ++ad) {
		for (channel_t ch = ADS868_CH_1; ch < ADS868_CH_MAX; ++ch) {
			ads8668_set_range(ad, ch, ADS868_RANGE_1_25_M);
		}
	}
}

/**
 * @brief Get the adc values object
 * 
 * @param dst 
 */
void get_adc_values(uint16_t *dst)
{
	k_mutex_lock(&adcMutex, K_FOREVER);
	memcpy(dst, adc_vals, sizeof(adc_vals));
	k_mutex_unlock(&adcMutex);
}

static const struct device *pwm_dev_1;
static const struct device *pwm_dev_2;
static const struct device *pwm_dev_8;

typedef enum pwm_t { PWM_1, PWM_2, PWM_8, PWM_MAX } pwm_t;

const struct device *get_pwm_dev(pwm_t pwm)
{
	switch (pwm) {
	case PWM_1:
		return pwm_dev_1;
	case PWM_2:
		return pwm_dev_2;
	case PWM_8:
		return pwm_dev_8;
	case PWM_MAX:
		return NULL;
	}

	return NULL;
}

typedef struct pwm_descriptor_t {
	pwm_t pwm;
	uint32_t channel;
	uint32_t period;
} pwm_descriptor_t;

#define PWM_MAX_CHANNELS 8
const pwm_descriptor_t pwm_descriptor[PWM_MAX_CHANNELS] = {
	{ PWM_2, 1, 8333 }, // SA1
	{ PWM_2, 2, 8333 }, // SA2
	{ PWM_1, 2, 8333 }, // SB1
	{ PWM_1, 1, 8333 }, // SB2
	{ PWM_8, 2, 8333 }, // SC1
	{ PWM_8, 1, 8333 }, // SC2
	{ PWM_2, 3, 8333 }, // SD1 -- 
	{ PWM_2, 4, 8333 }, // SD2
};

void pwm_init()
{
	pwm_dev_1 = device_get_binding("PWM_1");
	pwm_dev_2 = device_get_binding("PWM_2");
	pwm_dev_8 = device_get_binding("PWM_8");
}

void set_pwm_ranges(pid *pid_current)
{
	for (int i = 0; i < PWM_MAX_CHANNELS; ++i) {
		pwm_pin_set_usec(get_pwm_dev(pwm_descriptor[i].pwm), pwm_descriptor[i].channel, pwm_descriptor[i].period, (uint32_t) pid_current[i].control, 0);
		adc_vals[i+ 8] = (uint16_t)pid_current[i].control;
	}
}
uint16_t fromBartoBananas(uint32_t val){
	return (uint16_t) ((val*1600)/350) + 400;
}
void set_setpoints(uint32_t *vals){
	k_mutex_lock(&adcMutex, K_FOREVER);
	for(int i = 0; i < ADS868_CH_MAX ; i++){
		set_points[i] = fromBartoBananas(vals[i]);
	}
	k_mutex_unlock(&adcMutex);
}
void pid_current_thread(){
	uint32_t target = k_uptime_get_32();
	uint32_t delay = 0;
	pid pid_current[8];
	while (1) {	
		// Current control loop and set pwm
		k_mutex_lock(&pidMutex, K_FOREVER);
		PID_Controller(pid_current);
		set_pwm_ranges(pid_current);
		k_mutex_unlock(&pidMutex);
		do {
			target += 10;
			delay = target - k_uptime_get_32();
		} while (delay > 10);
		k_sleep(K_MSEC(delay));
	}

}
void main(void)
{
	const struct device *alive_dev = device_get_binding(LED_PORT);
	gpio_pin_configure(alive_dev, LED_PIN, GPIO_OUTPUT_ACTIVE);

	printk("\nHCB V:%s \n", HCB_FW_VERSION);

	adc_init();
	pwm_init();
	usb_transport_init();
	default_adc_range();

	acquire_adc();

	uint32_t target = k_uptime_get_32();
	uint32_t delay = 0;
	while (1) {	
		acquire_adc();
		do {
			target += 5;
			delay = target - k_uptime_get_32();
		} while (delay > 5);
		k_sleep(K_MSEC(delay));
		gpio_pin_set(alive_dev, LED_PIN, 1);
	}
}

#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

K_THREAD_DEFINE(send_id, STACKSIZE, usb_transport_thread, NULL, NULL, NULL,
		PRIORITY, 0, 10);

K_THREAD_DEFINE(unsolicited_id, STACKSIZE, usb_unsolicited_stuff, NULL, NULL,
		NULL, PRIORITY, 0, 10);

K_THREAD_DEFINE(pid_thread, STACKSIZE, pid_current_thread, NULL, NULL,
		NULL, PRIORITY, 0, 10);
