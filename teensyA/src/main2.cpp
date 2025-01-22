#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// #include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define EXTRUSION_MOTOR_PIN 29

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define SERIAL_PORT Serial

#define SPI_PORT SPI
#define CS_PIN 2

#define WIRE_PORT Wire
#define AD0_VAL 1

rcl_publisher_t pos_publisher;
rcl_publisher_t capacity_publisher;
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t path_subscriber;
sensor_msgs__msg__JointState js_msg;
std_msgs__msg__Float64MultiArray path_msg;

// struct CurrentVel {
//     float x_vel = -1.0;
//     float y_vel = -1.0;
// };

struct CurrentPos {
    float x_pos = -1.0;
    float y_pos = -1.0;
};

// CurrentVel vel; 
CurrentPos pos;

void js_subscription_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

    float y = (msg->name[0] == "MotorJointLeft") ? msg->position[0] : msg->position[1];
    // float y_vel = (msg->name[0] == "MotorJointLeft") ? msg->velocity[0] : msg->velocity[1];
    float x = (msg->name[0] == "MotorTop") ? msg->position[0] : msg->position[1];
    // float x_vel = (msg->name[0] == "MotorTop") ? msg->velocity[0] : msg->velocity[1];
    pos.x_pos = x;
    pos.y_pos = y;
    // pos.x_vel = x_vel;
    // pos.y_vel = y_vel;
}

void path_subscription_callback(const void* msgin) {
    const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    std::pair<float, float> start = std::make_pair(msg->data[0], msg->data[1]);
    std::pair<float, float> end = std::make_pair(msg->data[2], msg->data[3]);

    if (abs(pos.x_pos - start.first) > 1 && abs(pos.y_pos - start.second) > 1) {
        std_msgs__msg__Float64MultiArray pos_msg;
        pos_msg.data = {start.first, start.second};
        RCSOFTCHECK(rcl_publish(&pos_publisher, &pos_msg, NULL));
    }
    int extendPin = 31;
    int retractPin = 32;
    // Take max. value in case one of them is 0
    // float duty = std::max(vel.x_vel, vel.y_vel);
    // int value = static_cast<int>(20.0 + 180.0*duty);

    // Assume avg duty cycle to be 0.2 - 0.3
    digitalWrite(extendPin, HIGH);
    digitalWrite(retractPin, LOW);
    analogWrite(EXTRUSION_MOTOR_PIN, 20);

    pos_msg.data = {end.first, end.second};
    RCSOFTCHECK(rcl_publish(&pos_publisher, &pos_msg, NULL));

    digitalWrite(retractPin, HIGH);
    digitalWrite(extendPin, LOW);
    analogWrite(EXTRUSION_MOTOR_PIN, 20);
}



void setup() {
    SERIAL_PORT.begin(115200);
    SPI.begin();
    as5047p.initSPI();
    pinMode(EXTRUSION_MOTOR_PIN, OUTPUT);

    while(!Serial){
        delay(100);
    }

    set_microros_serial_transports(SERIAL_PORT);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 
    
    delay(1000);

    allocator = rcl_get_default_allocator();

    //create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 86);
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);)
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));

        RCCHECK(rclc_publisher_init_default(
            &pos_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "forward_position_controller/commands",
            &rmw_qos_profile_default
        ));

        RCCHECK(rclc_publisher_init_default(
            &capacity_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "capacity",
            &rmw_qos_profile_default
        ));

        RCCHECK(rclc_subscriber_init_default(
            &joint_state_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "joint_state",
            &rmw_qos_profile_default
        ));

        RCCHECK(rclc_subscriber_init_default(
            &path_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "path",
            &rmw_qos_profile_default
        ));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &js_msg, &js_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &path_subscriber, &path_msg, &path_subscription_callback, ON_NEW_DATA));
    

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  
}

float calculate_capacity(float angle) {
    float d = (angle / static_cast<float>(360)) * (static_cast<float>(60) / static_cast<float>(34)) * 2.0;
    return (17.75 - d) / 17.75;
}

void loop() {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) { // If the agent is not reachable, reset the board
        digitalWrite(LED_PIN, LOW);
        asm volatile("bx %0" :: "r" (0x00000000));
    }

    float angle = as5047p.readAngleDegree();
    float percentage = calculate_capacity(angle);

    if (percentage < 0.2) {
        std_msgs__msg__String warning_msg;
        warning_msg.data = "CAPACITY IS LESS THAN 20%, REFILL THE RESERVOIR.";
        RCSOFTCHECK(rcl_publish(&capacity_publisher, &warning_msg, NULL));
    }
}