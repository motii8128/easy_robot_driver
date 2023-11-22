#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <stdio.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t subscriber_fl;
rcl_subscription_t subscriber_fr;
rcl_subscription_t subscriber_rl;
rcl_subscription_t subscriber_rr;
std_msgs__msg__Float32 *get_fl_msg;
std_msgs__msg__Float32 *get_fr_msg;
std_msgs__msg__Float32 *get_rl_msg;
std_msgs__msg__Float32 *get_rr_msg;
std_msgs__msg__Float32 fl_msg;
std_msgs__msg__Float32 fr_msg;
std_msgs__msg__Float32 rl_msg;
std_msgs__msg__Float32 rr_msg;

bool fl_ready;
bool fr_ready;
bool rl_ready;
bool rr_ready;

void fl_callback(const void * msgin)
{
	get_fl_msg = (std_msgs__msg__Float32 *)msgin;
	fl_ready = true;
}
void fr_callback(const void * msgin)
{
	get_fr_msg = (std_msgs__msg__Float32 *)msgin;
	fr_ready = true;
}
void rl_callback(const void * msgin)
{
	get_rl_msg = (std_msgs__msg__Float32 *)msgin;
	rl_ready = true;
}
void rr_callback(const void * msgin)
{
	get_rr_msg = (std_msgs__msg__Float32 *)msgin;
	rr_ready = true;
}

void set_motor(bool fl_flag, bool fr_flag, bool rl_flag, bool rr_flag)
{
	if(fl_flag == true && fr_flag == true && rl_flag == true && rr_flag == true)
	{

	}
}

void appMain(void * arg)
{
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	fl_ready = false;
	fr_ready = false;
	rl_ready = false;
	rr_ready = false;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "int32_subscriber_rclc", "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber_fl,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/wheel/fl"));

	RCCHECK(rclc_subscription_init_default(
		&subscriber_fr,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/wheel/fr"));
	
	RCCHECK(rclc_subscription_init_default(
		&subscriber_rl,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/wheel/rl"));
	
	RCCHECK(rclc_subscription_init_default(
		&subscriber_rr,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/wheel/rr"));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fl, &fl_msg, &fl_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fr, &fr_msg, &fr_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_rl, &rl_msg, &rl_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_rr, &rr_msg, &rr_callback, ON_NEW_DATA));

	while(1){
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			usleep(100000);
			set_motor(fl_ready, fr_ready, rl_ready, rr_ready);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subscriber_fl, &node));
	RCCHECK(rcl_node_fini(&node));
	
	vTaskDelete(NULL);
}
