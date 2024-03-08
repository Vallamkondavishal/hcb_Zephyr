// Set the log level to INFO
#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>

// Register the module as "p" with INFO log level
LOG_MODULE_REGISTER(p, LOG_LEVEL_INF);

#include "adc_subsystem.h"

#include <drivers/can.h>
#include <drivers/gpio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/pwm.h>

#include "usb_hcb_subsystem.h"

// Define GPIO for LED
#define LED_PORT "GPIOC"
#define LED_PIN	 8

// Define minimum and maximum period for PWM
#define MIN_PERIOD_USEC (USEC_PER_SEC / 64U)
#define MAX_PERIOD_USEC USEC_PER_SEC
#define FILTER_LEN 8

// PID Controller Constants
#define Kp 0.3
#define Ki 0.7

#define K1 (Kp + Ki)
#define K2 -Kp

// Define mutexes for ADC and PID
K_MUTEX_DEFINE(adcMutex);
K_MUTEX_DEFINE(pidMutex);

// Arrays to store ADC values and set points
uint16_t adc_vals[24];
uint16_t set_points[8];
/**
 * @brief Mapping of ADC channels to axis
 * 
 * This array maps ADC channels to their corresponding axis.
 * Each row represents an ADC, and each column represents a channel.
 * The values indicate the order in which the channels are read for each ADC.
 * For example, ch2axis[0][0] represents the first channel to read for ADC 1.
 */
static const uint8_t ch2axis[AD_MAX][ADS868_CH_MAX] = {{1,2,3,4,5,6,7,0},{1,2,3,4,5,6,7,0},{1,2,3,4,5,6,7,0}};

/**
 * @brief Array indicating enabled ADC channels
 * 
 * This array indicates whether each ADC is enabled (1) or disabled (0).
 * Each element corresponds to an ADC, with 1 indicating enabled and 0 disabled.
 */
static const uint8_t enabled_adc[AD_MAX] = {1,0,1};

/**
 * @brief Function to acquire ADC values
 * 
 * This function acquires ADC values for enabled channels.
 * It uses a static array to filter ADC values over multiple samples.
 * Mutex is used to ensure thread safety.
 */
void acquire_adc()
{
	// Static variables for filtering ADC values
	static uint16_t internal_adc_vals[FILTER_LEN][24]; // Lots of SPI transactions
	static int filter_idx = 0;

	// Iterate through ADCs and channels to acquire values
	for (adc_t ad = AD_1; ad < AD_MAX; ++ad){
		if(enabled_adc[ad] == 1){
			for (channel_t ch = ADS868_CH_1; ch < ADS868_CH_MAX; ++ch) {
				int idx = (ad * 8) + ch2axis[ad][ch];
				internal_adc_vals[filter_idx][idx] = ads8668_acquire(ad, ch);
			}
		}
	}
	// Update filter index
	filter_idx++;
	if (filter_idx >= FILTER_LEN)
		filter_idx = 0;

	// Apply filter and update adc_vals array
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


/**
 * @brief Structure for PID Controller
 * 
 * This structure holds the variables required for the PID controller.
 */
typedef struct pid{
    float n_1_u;        // Previous control signal
    float n_1_error;    // Previous error
    float error;        // Current error
    float control;      // Current control signal
}pid;


/**
 * @brief PID Controller Function
 * 
 * This function implements the PID controller algorithm.
 * It calculates the control signal based on current errors and previous control signals.
 */

void PID_Controller(pid *pid_current){
	
	// Local variables for setpoints and ADC values
	uint16_t local_setpoints[8];
	uint16_t local_adcvals[24];
	
	// Copy setpoints and ADC values into local variables
	k_mutex_lock(&adcMutex, K_FOREVER);
	memcpy(local_setpoints,set_points,sizeof(set_points));
	memcpy(local_adcvals,adc_vals,sizeof(adc_vals));
	k_mutex_unlock(&adcMutex);
    
	// Iterate through ADC channels
	for(int i = 0; i < ADS868_CH_MAX; i++) {

		// Check if setpoint is below threshold
		if(local_setpoints[i] < 410) {
			pid_current[i].control = 0;
			continue;
		}
		
		//General sensor , constant value of on/off
		if(i == 6) pid_current[i].control = 5000;
		
		else{
			// Calculate error and control signal
			pid_current[i].error = local_setpoints[i] - local_adcvals[i];
			pid_current[i].control = pid_current[i].n_1_u + K1*pid_current[i].error + K2*pid_current[i].n_1_error;

			//Limite dei PWM settato 3000
			if(pid_current[i].control > 3000){
				pid_current[i].control = 3000;
			}
			if(pid_current[i].control <= 0){
				pid_current[i].control = 0;
			}
			// Update previous control signal and error
			pid_current[i].n_1_u = pid_current[i].control;
			pid_current[i].n_1_error = pid_current[i].error;
		}
		
	}

}


/**
 * @brief Set the default ADC range
 * 
 * This function sets the default range for all ADC channels.
 * It iterates through each ADC and channel, setting the range to 1.25V.
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
 * @brief Get the ADC values
 * 
 * This function copies ADC values to the destination buffer.
 * It locks the ADC mutex to ensure thread safety while copying.
 * 
 * @param dst Destination buffer to store ADC values
 */

void get_adc_values(uint16_t *dst)
{
	k_mutex_lock(&adcMutex, K_FOREVER);
	memcpy(dst, adc_vals, sizeof(adc_vals));
	k_mutex_unlock(&adcMutex);
}

// Declaration of static variables to store PWM devices
static const struct device *pwm_dev_1;
static const struct device *pwm_dev_2;
static const struct device *pwm_dev_8;

// Enumeration for PWM channels
typedef enum pwm_t { PWM_1, PWM_2, PWM_8, PWM_MAX } pwm_t;

// Function to get the PWM device based on the PWM channel
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

// Structure to hold PWM descriptor
typedef struct pwm_descriptor_t {
	pwm_t pwm;
	uint32_t channel;
	uint32_t period;
} pwm_descriptor_t;

// Maximum number of PWM channels
#define PWM_MAX_CHANNELS 8

// PWM descriptors initialization
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

/**
 * @brief Initialize PWM devices
 * 
 * This function initializes the PWM devices by getting their bindings.
 * It assigns the device pointers to static variables for later use.
 */
void pwm_init()
{
	pwm_dev_1 = device_get_binding("PWM_1");
	pwm_dev_2 = device_get_binding("PWM_2");
	pwm_dev_8 = device_get_binding("PWM_8");
}

/**
 * @brief Set PWM ranges
 * 
 * This function sets the PWM ranges based on PID controller outputs.
 * It iterates through the PWM channels and sets their duty cycles accordingly.
 * Additionally, it updates the ADC values corresponding to PWM channels.
 * 
 * @param pid_current Array of PID controller structures
 */
void set_pwm_ranges(pid *pid_current)
{
	for (int i = 0; i < PWM_MAX_CHANNELS; ++i) {
		pwm_pin_set_usec(get_pwm_dev(pwm_descriptor[i].pwm), pwm_descriptor[i].channel, pwm_descriptor[i].period, (uint32_t) pid_current[i].control, 0);
		adc_vals[i+ 8] = (uint16_t)pid_current[i].control;
	}
}

/**
 * @brief Convert value from bar to bananas
 * 
 * This function converts a value from the bar scale to the bananas scale.
 * 
 * @param val Value in the bar scale
 * @return Converted value in the bananas scale
 */
uint16_t fromBartoBananas(uint32_t val){
	return (uint16_t) ((val*1600)/350) + 400;
}

/**
 * @brief Set setpoints
 * 
 * This function sets the setpoints for the PID controller.
 * It converts the values from the bar scale to the bananas scale and updates the set_points array.
 * 
 * @param vals Array of setpoints in the bar scale
 */
void set_setpoints(uint32_t *vals){
	k_mutex_lock(&adcMutex, K_FOREVER);
	for(int i = 0; i < ADS868_CH_MAX ; i++){
		set_points[i] = fromBartoBananas(vals[i]);
	}
	k_mutex_unlock(&adcMutex);
}

/**
 * @brief PID current thread
 * 
 * This thread continuously runs the PID controller algorithm.
 * It locks the PID mutex, executes the controller, sets PWM ranges, and unlocks the mutex.
 * It then waits for a specified delay before repeating the process.
 */
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

/**
 * @brief Main function
 * 
 * This function is the entry point of the application.
 * It initializes necessary components, such as ADC, PWM, USB transport, and default ADC range.
 * It then enters an infinite loop where it continuously acquires ADC values, sets LED status, and sleeps for a specified delay.
 */
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

/* Definition of thread stack size */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

/* Thread definitions */
K_THREAD_DEFINE(send_id, STACKSIZE, usb_transport_thread, NULL, NULL, NULL,
		PRIORITY, 0, 10);

K_THREAD_DEFINE(unsolicited_id, STACKSIZE, usb_unsolicited_stuff, NULL, NULL,
		NULL, PRIORITY, 0, 10);

K_THREAD_DEFINE(pid_thread, STACKSIZE, pid_current_thread, NULL, NULL,
		NULL, PRIORITY, 0, 10);