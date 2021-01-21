/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <kernel.h>
#include <drivers/can.h>
#include <sys/byteorder.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

#define LED_MSG_ID 0x10


/* scheduling priority used by each thread */
#define PRIORITY 7
 
/*Defining the LEDs used */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

// Number of nodes in the network
#define num_nodes 3

/* Defines the ID of each node. The neighbors and node filters are computed based on node ID. 
So, make sure it is unique for each node on the network. */
#define NODE_ID 0

// Neigbhbors of each node in the network.
#define NEIGHBOR (NODE_ID + 1)% num_nodes

// CAN Filter IDs that each of the nodes on the network listens to
const int nodeFilters[num_nodes] = {0x123, 0x234, 0x345};
 
// Boolean variable that are used to indicate that the node holds the token
bool isTokenAvailable = false;

// Boolean variable to indicate that the node is done executing the task and ready to relinquish the token
bool releaseToken = false;


const struct device *can_dev;

// CAN frame that is sent over the network 
struct zcan_frame can_frame = {
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr = CAN_DATAFRAME,
		.std_id = nodeFilters[NEIGHBOR],
		.dlc = 1,
		.data[0] = 12
	};

struct printk_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint32_t led;
	uint32_t cnt;
};

K_FIFO_DEFINE(printk_fifo);

// Struct that defines the LED
struct led {
	const char *gpio_dev_name;
	const char *gpio_pin_name;
	unsigned int gpio_pin;
	unsigned int gpio_flags;
};

// Callback function for the CAN thread. 
// It sets the token to true to indicate that the node holds the token.
void can_callback_function(struct zcan_frame *frame, bool *isTokenAvailable)
{
	*isTokenAvailable = true;
};

/* The task for the app thread that lights up an LED upon the availability of a token 
 Later, it sets the release token toindicate to the can thread that the token can be released */
void AppTask(const struct led *led, uint32_t sleep_ms, uint32_t id, bool *isTokenAvailable, bool *releaseToken){
	struct device *gpio_dev;
	int ret;

	if(*isTokenAvailable){
	gpio_dev = device_get_binding(led->gpio_dev_name);
	if (gpio_dev == NULL) {
		printk("Error: didn't find %s device\n",
		       led->gpio_dev_name);
		return;
	}

	ret = gpio_pin_configure(gpio_dev, led->gpio_pin, led->gpio_flags);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n",
			ret, led->gpio_pin, led->gpio_pin_name);
		return;
	}

		ret = gpio_pin_get_raw(gpio_dev, led->gpio_pin);
		if(ret == 0){
			gpio_pin_set(gpio_dev, led->gpio_pin, 1);
		}else{
			gpio_pin_set(gpio_dev, led->gpio_pin, 0);
		}

		*releaseToken = true;
	}
		
	k_msleep(sleep_ms);

}
 
/* Releases the token when the node is done executing the tasks */
void ReleaseToken(bool *isTokenAvailable, bool *releaseToken, const struct led *led0){
	 	struct device *gpio_dev;


	int ret;
	if(*isTokenAvailable && *releaseToken){
		ret = can_send(can_dev, &can_frame, K_FOREVER,
			 NULL,
			 "LED change");
		if(ret != CAN_TX_OK){
			 gpio_dev = device_get_binding(led0->gpio_dev_name);
			 gpio_pin_set(gpio_dev, led0->gpio_pin, 1);

		}
		*isTokenAvailable = false;
		*releaseToken = false;
	}

	k_msleep(1000);
}

/* CAN thread that is responsible for configuring and initializing the nodes on CAN bus. 
The thread also sets an interrupt request to receive incoming CAN messages */ 
void CanThread(void)
{ 
	int filter_id;
	const struct led led0 = {
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
		.gpio_dev_name = DT_GPIO_LABEL(LED0_NODE, gpios),
		.gpio_pin_name = DT_LABEL(LED0_NODE),
		.gpio_pin = DT_GPIO_PIN(LED0_NODE, gpios),
		.gpio_flags = GPIO_OUTPUT | DT_GPIO_FLAGS(LED0_NODE, gpios),
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#endif
	};

	const struct zcan_filter my_filter = {
	        .id_type = CAN_STANDARD_IDENTIFIER,
	        .rtr = CAN_DATAFRAME,
	        .std_id = nodeFilters[NODE_ID],
	        .rtr_mask = 1,
	        .std_id_mask = CAN_STD_ID_MASK
	};

	can_dev = device_get_binding(DT_CHOSEN_ZEPHYR_CAN_PRIMARY_LABEL);

	if (!can_dev) {
		printk("CAN: Device driver not found.\n");
		return;
	}

	filter_id = can_attach_isr(can_dev, can_callback_function, &isTokenAvailable, &my_filter);
	if (filter_id < 0) {
		return;
	}

	can_configure(can_dev, CAN_NORMAL_MODE, 125000);

	while(true){
		ReleaseToken(&isTokenAvailable, &releaseToken, &led0);	
	}
	
	
}

/* The App thread that executes the App task. The task though is executed based upon the availability of the token */
void AppThread(void)
{
	const struct led led1 = {
#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
		.gpio_dev_name = DT_GPIO_LABEL(LED1_NODE, gpios),
		.gpio_pin_name = DT_LABEL(LED1_NODE),
		.gpio_pin = DT_GPIO_PIN(LED1_NODE, gpios),
		.gpio_flags = GPIO_OUTPUT | DT_GPIO_FLAGS(LED1_NODE, gpios),
#else
#error "Unsupported board: led1 devicetree alias is not defined"
#endif
	};
	
	while(true){
		AppTask(&led1, 1000, 1, &isTokenAvailable, &releaseToken);
	}
	
}

void main(void){}
	

K_THREAD_DEFINE(can_thread, STACKSIZE, CanThread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(app_thread, STACKSIZE, AppThread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
