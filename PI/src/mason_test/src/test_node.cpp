#include <chrono>
#include <float32_multi_array__struct.hpp>

#include "odrive_base/include/odrive_enums.h"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
   public:
    TestNode() : Node("test_node") {
        RCLCPP_INFO(this->get_logger(), "Test node created");

        this->request_axis_state_client0 =
            this->create_client<odrive_can::srv::AxisState>("odrive_axis0/request_axis_state");
        this->request_axis_state_client1 =
            this->create_client<odrive_can::srv::AxisState>("odrive_axis1/request_axis_state");
        this->request_axis_state_client2 =
            this->create_client<odrive_can::srv::AxisState>("odrive_axis2/request_axis_state");

        RCLCPP_INFO(this->get_logger(), "State clients created; Setting closed loop control...");

        set_axis_state(this->request_axis_state_client0, 0,
                       ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        set_axis_state(this->request_axis_state_client1, 1,
                       ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        set_axis_state(this->request_axis_state_client2, 2,
                       ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        RCLCPP_INFO(this->get_logger(), "Closed loop control set");

        // a sub wihch takes float array
        sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odrive_position", 10,
            std::bind(&TestNode::position_callback, this, std::placeholders::_1));

        this->pub0 = this->create_publisher<odrive_can::msg::ControlMessage>(
            "odrive_axis0/control_message", 10);
        this->pub1 = this->create_publisher<odrive_can::msg::ControlMessage>(
            "odrive_axis1/control_message", 10);
        this->pub2 = this->create_publisher<odrive_can::msg::ControlMessage>(
            "odrive_axis2/control_message", 10);

        this->sub1 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis0/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                axis0_pos = msg->pos_estimate;
            });
        this->sub2 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis1/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                axis1_pos = msg->pos_estimate;
            });
        this->sub0 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis2/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                axis2_pos = msg->pos_estimate;
            });

        // create a timer every 100ms
        this->timer = this->create_wall_timer(100ms, std::bind(&TestNode::timer_callback, this));
    }

    void position_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Position callback called");
        if (msg->data.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Invalid message size");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", msg->data[0], msg->data[1],
                    msg->data[2]);

        float x = msg->data[0];
        float y = msg->data[1];
        float z = msg->data[2];

        // ros2 topic pub /odrive_axis0/control_message odrive_can/msg/ControlMessage
        // "{control_mode: 2, input_mode: 1, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"

        odrive_can::msg::ControlMessage control_msg0, control_msg1, control_msg2;
        control_msg0.control_mode = 3;  // position control
        control_msg0.input_mode = 5;    // trajectory mode
        control_msg0.input_pos = x;

        control_msg1.control_mode = 3;  // position control
        control_msg1.input_mode = 5;    // trajectory mode
        control_msg1.input_pos = y;

        control_msg2.control_mode = 3;  // position control
        control_msg2.input_mode = 5;    // trajectory mode
        control_msg2.input_pos = z;

        // publish control messages

        this->pub0->publish(control_msg0);
        this->pub1->publish(control_msg1);
        this->pub2->publish(control_msg2);
    }

    void timer_callback() {
        if (std::abs(this->axis0_pos - 20) <= 1e-2) {
            RCLCPP_INFO(this->get_logger(), "Axis 0 reached position 20");
            odrive_can::msg::ControlMessage control_msg0;
            control_msg0.control_mode = 3;  // position control
            control_msg0.input_mode = 5;    // trajectory mode
            control_msg0.input_pos = 20;
            this->pub0->publish(control_msg0);
        }
    }

    ~TestNode() {
        set_axis_state(this->request_axis_state_client0, 0, ODriveAxisState::AXIS_STATE_IDLE);
        set_axis_state(this->request_axis_state_client1, 1, ODriveAxisState::AXIS_STATE_IDLE);
        set_axis_state(this->request_axis_state_client2, 2, ODriveAxisState::AXIS_STATE_IDLE);
    }

   private:
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client0;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client1;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client2;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub;

    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub0;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub1;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub2;

    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub0;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub2;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub1;

    rclcpp::TimerBase::SharedPtr timer;

    float axis0_pos, axis1_pos, axis2_pos;

    void set_axis_state(rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client, int axis,
                        ODriveAxisState state) {
        while (!client->wait_for_service(500ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for the service for axis %d. Exiting.",
                             axis);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service for axis %d not available, waiting again...",
                        axis);
        }

        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = static_cast<int32_t>(state);
        client->async_send_request(request).wait();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}