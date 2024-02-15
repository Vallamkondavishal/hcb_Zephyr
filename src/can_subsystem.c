#include "can_subsystem.h"

#include <zephyr.h>
#include <kernel.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/can.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>

// Define the stack size and priority for the receiving thread
#define RX_THREAD_STACK_SIZE	     512
#define RX_THREAD_PRIORITY	     2

// Define the stack size and priority for the state polling thread
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY   2

// Define message IDs for LED control and counter
#define LED_MSG_ID		     0x10
#define COUNTER_MSG_ID		     0x12345

// Define commands for LED control
#define SET_LED			     1
#define RESET_LED		     0

// Define sleep time
#define SLEEP_TIME		     K_MSEC(250)

// Define the ID for the Fault Tolerance (FT) system
#define FT_ID 0x1b0

// Define a stack for the receive thread with size RX_THREAD_STACK_SIZE
K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);

// Define a stack for the poll state thread with size STATE_POLL_THREAD_STACK_SIZE
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);

// Declare pointers to devices for CAN and LED GPIO
const struct device *can_dev;
const struct device *led_gpio_dev;

// Declare thread data structures for receive and poll state threads
struct k_thread rx_thread_data;
struct k_thread poll_state_thread_data;

// Declare work structure for receive work
struct zcan_work rx_work;

// Declare work structure for state change work
struct k_work state_change_work;

// Declare an enumeration to represent the current state of the CAN bus
enum can_state current_state;

// Declare a structure to hold the current error counts on the CAN bus
struct can_bus_err_cnt current_err_cnt;

#ifdef CANFESTIVAL_ZEPHYR_DEBUG

// Function to print the contents of a Zephyr CAN frame
static void print_frame(struct zcan_frame *frame)
{
	// Print frame ID, RTR flag, and DLC
	printk("|0x%3x|%s|%d|", frame->id, frame->rtr ? "RTR" : "   ",
	       frame->dlc);
	
    // Iterate through each byte of data in the frame
	for (int i = 0; i < CAN_MAX_DLEN; i++) {
		if (i < frame->dlc) {

			// Print data byte in hexadecimal format
			printk(" 0x%02x", frame->data[i]);
		} else {
			// If data length is less than the maximum, pad with spaces
			printk("     ");
		}
	}
    // End the line
	printk("|\n");
}
#endif

// Callback function invoked upon transmission error interrupt
void tx_irq_callback(uint32_t error_flags, void *arg)
{   
	// Cast arg to a character pointer representing the sender
	char *sender = (char *)arg;
    // If error flags are non-zero, print error details
	if (error_flags) {
		printk("Callback! error-code: %d\nSender: %s\n", error_flags,
		       sender);
	}
}

// Define a mutex for protecting access to force and torque arrays
K_MUTEX_DEFINE(ftMutex);

// Arrays to store force and torque values
static int32_t forces[3];
static int32_t torques[3];

/**
 * @brief Get the adc values object
 * 
 * @param dst   Destination array to store force and torque values
 */
void get_ft_values(int32_t *dst)
{
	// Lock the mutex to prevent concurrent access to force and torque arrays
	k_mutex_lock(&ftMutex, K_FOREVER);

	// Copy force and torque arrays into destination array
	memcpy(dst, forces, sizeof(forces));
	memcpy(dst+3, torques, sizeof(torques));

	// Unlock the mutex to allow other threads to access force and torque arrays
	k_mutex_unlock(&ftMutex);
}

/**
 * @brief Callback function to receive and update force and torque values
 * 
 * @param frame Pointer to the received CAN frame
 * @param unused Unused argument
 */
void receive_change_led(struct zcan_frame *frame, void *unused)
{
	ARG_UNUSED(unused);
    
	// Iterate over the array indices to check if the received frame corresponds to force or torque data
	for (int i = 1; i < 4; ++i)
	{
		// Check if the frame ID matches the expected ID for force or torque data
		if (frame->id == (FT_ID + i)) {

			// Lock the mutex to prevent concurrent access to force and torque arrays
			k_mutex_lock(&ftMutex, K_FOREVER);

			// Extract force and torque values from the received frame and store them in the respective arrays
			uint32_t f = frame->data_32[0];
			forces[i - 1] = (int32_t)__bswap_32(f);
			uint32_t t = frame->data_32[1];
			torques[i - 1] = (int32_t)__bswap_32(t);

			// Unlock the mutex to allow other threads to access force and torque arrays
			k_mutex_unlock(&ftMutex);
		}
	}

// Optional debug printing if CANFESTIVAL_ZEPHYR_DEBUG is defined
#ifdef CANFESTIVAL_ZEPHYR_DEBUG
	printk(" CAN_RECEIVE: ");
	print_frame(frame);
#endif
}

/**
 * @brief Converts a CAN state enum value to its string representation.
 * 
 * @param state The CAN state enum value to convert.
 * @return A string representation of the CAN state.
 */
char *state_to_str(enum can_state state)
{
	switch (state) {
	case CAN_ERROR_ACTIVE:
		return "error-active";
	case CAN_ERROR_PASSIVE:
		return "error-passive";
	case CAN_BUS_OFF:
		return "bus-off";
	default:
		return "unknown";
	}
}

/**
 * @brief Thread function to continuously poll the CAN bus state and error counts.
 * 
 * @param unused1 Unused parameter (required by Zephyr thread API).
 * @param unused2 Unused parameter (required by Zephyr thread API).
 * @param unused3 Unused parameter (required by Zephyr thread API).
 */
void poll_state_thread(void *unused1, void *unused2, void *unused3)
{   
	// Initialize error count structures
	struct can_bus_err_cnt err_cnt = { 0, 0 };
	struct can_bus_err_cnt err_cnt_prev = { 0, 0 };

	// Initialize previous state to error active
	enum can_state state_prev = CAN_ERROR_ACTIVE;

	// Variable to hold current CAN state
	enum can_state state;
    
	// Continuous loop to poll CAN state and error counts
	while (1) {

		// Get current CAN state and error counts
		state = can_get_state(can_dev, &err_cnt);

		// Check if there's a change in state or error counts
		if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
		    err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt ||
		    state_prev != state) {
			// Update previous error counts and state
			err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
			err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
			state_prev = state;

			// Print current state and error counts
			printk("state: %s\n"
			       "rx error count: %d\n"
			       "tx error count: %d\n",
			       state_to_str(state), err_cnt.rx_err_cnt,
			       err_cnt.tx_err_cnt);
		} else {

			// Sleep for a short duration to avoid continuous polling
			k_sleep(K_MSEC(100));
		}
	}
}

/**
 * @brief Handler function for state change work.
 * 
 * @param work Pointer to the work item.
 */
void state_change_work_handler(struct k_work *work)
{
	// Print current state and error counts
	printk("State Change ISR\nstate: %s\n"
	       "rx error count: %d\n"
	       "tx error count: %d\n",
	       state_to_str(current_state), current_err_cnt.rx_err_cnt,
	       current_err_cnt.tx_err_cnt);

// Perform recovery from bus-off state if auto recovery is not enabled
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	if (current_state == CAN_BUS_OFF) {
		printk("Recover from bus-off\n");

        // Attempt to recover from bus-off state
		if (can_recover(can_dev, K_MSEC(100) != 0)) {
			printk("Recovery timed out\n");
		}
	}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */
}

/**
 * @brief Interrupt service routine for CAN state change.
 * 
 * @param state The new CAN state.
 * @param err_cnt Error counts for the CAN bus.
 */
void state_change_isr(enum can_state state, struct can_bus_err_cnt err_cnt)
{
	// Update current state and error counts
	current_state = state;
	current_err_cnt = err_cnt;

	// Submit state change work for handling
	k_work_submit(&state_change_work);
}

/**
 * @brief Main thread for handling CAN communication and LED change requests.
 * 
 * @param _1 Unused parameter (required by Zephyr thread API).
 * @param _2 Unused parameter (required by Zephyr thread API).
 * @param _3 Unused parameter (required by Zephyr thread API).
 */
void main_can_ati_thread(void *_1, void *_2, void *_3)
{
	// Define a CAN filter for LED change messages
	static const struct zcan_filter my_filter = {
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr = CAN_DATAFRAME,
		.id = 0x1,
		.rtr_mask = 0,
		.id_mask = 0x0
	};
    // Define a CAN frame for requesting LED changes
	struct zcan_frame req_long_data_frame = {
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr = CAN_DATAFRAME,
		.id = FT_ID,
		.dlc = 1
	};
    // Variable to toggle LED change data
    uint8_t toggle = 1;
    
    // Counter for LED change requests
    uint16_t counter = 0;
    
    // Thread IDs for polling state and receiving CAN frames
    k_tid_t rx_tid, get_state_tid;
    
    // Return value for function calls
    int ret;

    // Get the CAN device
    can_dev = device_get_binding(DT_CHOSEN_ZEPHYR_CAN_PRIMARY_LABEL);

    // Check if CAN device is found
    if (!can_dev) {
        printk("CAN: Device driver not found.\n");
        return;
    }

    // Initialize state change work handler
    k_work_init(&state_change_work, state_change_work_handler);

    // Attach a filter for receiving LED change messages
    ret = can_attach_workq(can_dev, &k_sys_work_q, &rx_work,
                           receive_change_led, NULL, &my_filter);
    if (ret == CAN_NO_FREE_FILTER) {
        printk("Error, no filter available!\n");
        return;
    }

    // Create a thread for polling CAN bus state
	printk("Change LED filter ID: %d\n", ret);
	get_state_tid =
		k_thread_create(&poll_state_thread_data, poll_state_stack,
				K_THREAD_STACK_SIZEOF(poll_state_stack),
				poll_state_thread, NULL, NULL, NULL,
				STATE_POLL_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!get_state_tid) {
		printk("ERROR spawning poll_state_thread\n");
	}

    // Register CAN state change ISR
    can_register_state_change_isr(can_dev, state_change_isr);

    // Print initialization completion message
    printk("Finished init.\n");

    // Main loop for sending LED change requests
    while (1) {
        // Set LED change request data
        req_long_data_frame.data[0] = 0x01;
        
        // Send LED change request
        can_send(can_dev, &req_long_data_frame, K_FOREVER,
                 tx_irq_callback, "LED change");
        
        // Sleep for a defined duration
        k_sleep(SLEEP_TIME);
    }
}

// Thread Definition

// Define stack size and priority for main CAN ATI thread
#define CANATI_STACK_SIZE 512
#define CANATI_PRIORITY	  5

// Define main CAN ATI thread
K_THREAD_DEFINE(CANATI, CANATI_STACK_SIZE, main_can_ati_thread, NULL, NULL,
                NULL, CANATI_PRIORITY, 0, 0);
