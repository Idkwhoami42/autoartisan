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

DMAMEM rcl_timer_t timer;

// subscriber
DMAMEM rcl_subscription_t subscriber;
DMAMEM std_msgs__msg__Int32 msg;
DMAMEM rclc_executor_t executor_sub;

// capacity
DMAMEM rcl_publisher_t capacity_publisher;
DMAMEM rcl_publisher_t capacity_publisher2;
float percentage = std::nanf("");

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
    
    // capacity publisher set-up
    RCCHECK(rclc_publisher_init_default(&capacity_publisher,
                                        &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                        "capacity"),
            "Failed to initialize capacity publisher");
    RCCHECK(rclc_publisher_init_default(&capacity_publisher2,
                                        &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                        "capacity_percentage"),
            "Failed to initialize capacity percentage publisher");

    Serial.println("Micro-ROS setup done!");
}

int get_section(float p) {
    if (p >= 0.0f && p < 10.0f) return 1;
    else if (p >= 10.0f && p < 20.0f) return 2;
    else if (p >= 20.0f && p < 30.0f) return 3;
    else if (p >= 30.0f && p < 40.0f) return 4;
    else if (p >= 40.0f && p < 50.0f) return 5;
    else if (p >= 50.0f && p < 60.0f) return 6;
    else if (p >= 60.0f && p < 70.0f) return 7;
    else if (p >= 70.0f && p < 80.0f) return 8;
    else if (p >= 80.0f && p < 90.0f) return 9;
    else return 10;
}

void update_capacity(float angle) {
    std_msgs__msg__Int32 cap_msg;
    std_msgs__msg__String cap_perc_msg;
    std_msgs__msg__String__init(&cap_perc_msg);

    float d = (angle / static_cast<float>(360)) * (static_cast<float>(60) / static_cast<float>(34)) * 2.0;
    float prev_percentage = std::isnan(percentage) ? std::nanf("") : percentage;
    percentage = 100 * (17.75 - d) / 17.75;

    // Log values
    Serial.printf("Prev_percentage: %.1f%%", prev_percentage);
    Serial.printf("Percentage: %.1f%%", percentage);

    if (std::isnan(prev_percentage) || (get_section(prev_percentage) != get_section(percentage))) {
        cap_msg.data = get_section(percentage);
        char buffer[32];
        // sprintf(buffer, "Capacity: %.1f%", mynumber);
        // cap_perc_msg.data = buffer;

        sprintf(buffer, "Capacity: %.1f%%", percentage);
        rosidl_runtime_c__String__assign(&cap_perc_msg.data, buffer);

        RCSOFTCHECK(rcl_publish(&capacity_publisher, &cap_msg, NULL));
        RCSOFTCHECK(rcl_publish(&capacity_publisher2, &cap_perc_msg, NULL));
    }

    std_msgs__msg__String__fini(&cap_perc_msg);
}

void micro_ros_loop() {
    RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)),
            "Failed to spin sub executor");
    float angle = getAngle();
    update_capacity(angle);
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

    setupAS5047P();

    micro_ros_setup();
}

void loop() { micro_ros_loop(); }