#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensors.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>

#include <cmath>

DMAMEM rcl_node_t node;
DMAMEM rclc_support_t support;
DMAMEM rcl_allocator_t allocator;


// subscriber
DMAMEM rcl_subscription_t subscriber;
DMAMEM std_msgs__msg__Int32 fsm_msg;
DMAMEM rclc_executor_t executor_sub;

// capacity
DMAMEM rcl_publisher_t capacity_publisher;
DMAMEM std_msgs__msg__Int32 capacity_msg;
DMAMEM rclc_executor_t executor_pub;
DMAMEM rcl_timer_t timer;

#define LED_PIN LED_BUILTIN

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }

/**
 * @brief loop to indicate error with blinking LED
 *
 */
void error_loop() {
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

/**
 * @brief subscription callback executed at receiving a message
 *
 * @param msgin
 */
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *message = (const std_msgs__msg__Int32 *)msgin;

    int val = message->data;

    int data1 = val & 0xF;
    int data2 = val >> 4;

    // START BRUSH
    if (data1 == 0) {
        activateBrush();
    } else if (data1 == 1) {
        deactivateBrush();
    } else if (data1 == 2) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        activateVerticalSmoothing();
    } else if (data1 == 3) {
        activateHorizontalSmoothing();
    } else if (data1 == 4) {
        deactivateVerticalSmoothing();
    } else if (data1 == 5) {
        deactivateHorizontalSmoothing();
    } else if (data1 == 6) {
        if (data2 == 0) {
            goForward(1000);
        } else {
            goForward(data2);
        }
    } else if (data1 == 7) {
        if (data2 == 0) {
            goBack(1000);
        } else {
            goBack(data2);
        }
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

        capacity_msg.data++;
        RCSOFTCHECK(rcl_publish(&capacity_publisher, &capacity_msg, NULL));
    }
}

void micro_ros_setup() {
    Serial.println("Setting up micro-ROS...");

    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 86);
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(&subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                           "/mason_fsm_publisher/state"));

    // capacity publisher set-up
    RCCHECK(rclc_publisher_init_default(
        &capacity_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/capacity"));

    const unsigned int timer_timeout = 300;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &fsm_msg,
                                           &subscription_callback, ON_NEW_DATA));

    capacity_msg.data = 0;

    Serial.println("Micro-ROS setup done!");
}

void micro_ros_loop() {
    delay(100);
    RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
    RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
}

void setup() {
    Serial.begin(115200);
    // pinMode(LED_PIN, OUTPUT);
    while (!Serial) {
        // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(250);
    }

    // digitalWrite(LED_PIN, HIGH);

    setupSmoothingServos();

    setupBrushServo();

    setupBrushMotor();

    setupExtruderMotor();

    setupAS5047P();

    micro_ros_setup();
}

void loop() { micro_ros_loop(); }