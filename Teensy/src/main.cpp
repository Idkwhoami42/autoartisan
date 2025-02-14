// this code is example of publisher and subscriber
// https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_subscriber/micro-ros_subscriber.ino
// https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_publisher/micro-ros_publisher.ino
// run micro_ros_agent before connecting arduino
// docker run -it --rm -v /dev:/dev --privileged --net=host
// microros/micro-ros-agent:galactic serial --dev /dev/cu.usbmodem1422201 -v6

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

DMAMEM rcl_node_t node;
DMAMEM rclc_support_t support;
DMAMEM rcl_allocator_t allocator;

DMAMEM rcl_timer_t timer;

// subscriber
DMAMEM rcl_subscription_t subscriber;
DMAMEM std_msgs__msg__Int32 msg;
DMAMEM rclc_executor_t executor_sub;

#define LED_PIN LED_BUILTIN

#define RCCHECK(fn, msg)               \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop(msg);           \
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
void error_loop(String msg) {
    Serial.printf("Error: %s\n", msg.c_str());
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
    const std_msgs__msg__Int32 *msg_led = (const std_msgs__msg__Int32 *)msgin;

    // START BRUSH
    if (msg_led->data == 0) {
        activateBrush();
    } else if (msg_led->data == 1) {
        deactivateBrush();
    } else if (msg_led->data == 2) {
        activateVerticalSmoothing();
    } else if (msg_led->data == 3) {
        activateHorizontalSmoothing();
    } else if (msg_led->data == 4) {
        deactivateVerticalSmoothing();
    } else if (msg_led->data == 5) {
        deactivateHorizontalSmoothing();
    } else if (msg_led->data == 6) {
        goForward(1000);
    } else if (msg_led->data == 7) {
        goBack(1000);
    } else if (msg_led->data == 8) {
        activateExtruderMotor();
    } else if (msg_led->data == 9) {
        deactivateExtruderMotor();
    }
}

void micro_ros_setup() {
    Serial.println("Setting up micro-ROS...");

    set_microros_serial_transports(Serial);

    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 86);
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator),
            "Failed to initialize support");

    // create node
    RCCHECK(rclc_node_init_default(&node, "teensy", "", &support), "Failed to initialize node");

    // create subscriber
    RCCHECK(rclc_subscription_init_default(&subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                           "/mason_fsm_publisher/state"),
            "Failed to initialize subscriber");

    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator),
            "Failed to initialize executor");
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback,
                                           ON_NEW_DATA),
            "Failed to add subscription to executor");

    Serial.println("Micro-ROS setup done!");
}

void micro_ros_loop() {
    RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)),
            "Failed to spin pub executor");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    while (!Serial) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(250);
    }

    digitalWrite(LED_PIN, LOW);

    setupSmoothingServos();

    setupBrushServo();

    setupBrushMotor();

    setupExtruderMotor();

    micro_ros_setup();
}

void loop() { micro_ros_loop(); }