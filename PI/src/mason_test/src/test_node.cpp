#include <chrono>

#include "../include/mason_test/odrive_enums.h"
#include "mason_test/srv/float.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"

#define POS_EPSILON 0.15f

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

        bool ok_axis0 = set_axis_state(this->request_axis_state_client0, 0,
                                       ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        bool ok_axis1 = set_axis_state(this->request_axis_state_client1, 1,
                                       ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        bool ok_axis2 = set_axis_state(this->request_axis_state_client2, 2,
                                       ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        if (!ok_axis0 || !ok_axis1 || !ok_axis2) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set closed loop control for all axes");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Closed loop control set");

        this->position_y_service = this->create_service<mason_test::srv::Float>(
            "odrive_position_y",
            std::bind(&TestNode::position_y_callback, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3));

        this->position_x_service = this->create_service<mason_test::srv::Float>(
            "odrive_position_x",
            std::bind(&TestNode::position_x_callback, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3));

        this->stop_y_service = this->create_service<std_srvs::srv::Empty>(
            "odrive_stop_y", std::bind(&TestNode::stop_y_callback, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3));

        this->stop_x_service = this->create_service<std_srvs::srv::Empty>(
            "odrive_stop_x", std::bind(&TestNode::stop_x_callback, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3));

        this->pub0 = this->create_publisher<odrive_can::msg::ControlMessage>(
            "odrive_axis0/control_message", 10);
        this->pub1 = this->create_publisher<odrive_can::msg::ControlMessage>(
            "odrive_axis1/control_message", 10);
        this->pub2 = this->create_publisher<odrive_can::msg::ControlMessage>(
            "odrive_axis2/control_message", 10);

        this->pub_position =
            this->create_publisher<std_msgs::msg::Float32MultiArray>("odrive_position", 10);

        this->sub0 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis0/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                if (std::isnan(axis0_offset)) axis0_offset = msg->pos_estimate;
                axis0_pos = msg->pos_estimate;
                this->axis0_error = msg->active_errors;
                if (msg->active_errors != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Axis 0 error detected %d", this->axis0_error);
                    this->stop(0);
                }
            });
        this->sub1 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis1/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                if (std::isnan(axis1_offset)) axis1_offset = msg->pos_estimate;
                axis1_pos = msg->pos_estimate;
                this->axis1_error = msg->active_errors;

                if (msg->active_errors != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Axis 1 error detected %d", this->axis1_error);
                    this->stop(1);
                }
            });
        this->sub2 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis2/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                if (std::isnan(axis2_offset)) axis2_offset = msg->pos_estimate;
                axis2_pos = msg->pos_estimate;
                this->axis2_error = msg->active_errors;
                if (msg->active_errors != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Axis 2 error detected %d", this->axis2_error);
                    this->stop(2);
                }
            });

        this->position_timer = this->create_wall_timer(50ms, [this]() {
            std_msgs::msg::Float32MultiArray msg;
            if (std::isnan(axis0_offset) || std::isnan(axis1_offset) || std::isnan(axis2_offset))
                return;
            msg.data.push_back(-1 * (axis0_pos - axis0_offset));

            float axis1_corrected_pos = axis1_pos - axis1_offset;
            float axis2_corrected_pos = -1 * (axis2_pos - axis2_offset);

            if (!float_compare(axis1_corrected_pos, axis2_corrected_pos, 1.0f)) {
                RCLCPP_ERROR(this->get_logger(), "Axis 1 and Axis 2 are not in sync %f %f",
                             axis1_corrected_pos, axis2_corrected_pos);
                this->axis1_error = 1;
                this->axis2_error = 1;
                this->stop(1);
                this->stop(2);
            }

            msg.data.push_back((axis1_corrected_pos + axis2_corrected_pos) / 2);
            this->pub_position->publish(msg);
        });
    }

    void position_y_callback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                             const std::shared_ptr<mason_test::srv::Float::Request> request,
                             const std::shared_ptr<mason_test::srv::Float::Response> /*response*/) {
        RCLCPP_INFO(this->get_logger(), "Position Y callback called");
        RCLCPP_INFO(this->get_logger(), "Request: %f", request->input_pos);

        if (this->axis1_error != 0 || this->axis2_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Axis 1 or 2 error detected");
            return;
        }

        float axis1_newpos = request->input_pos + this->axis1_offset;
        float axis2_newpos = -1 * request->input_pos + this->axis2_offset;
        this->move(1, axis1_newpos);
        this->move(2, axis2_newpos);

        // while (!float_compare(this->axis1_pos, axis1_newpos, POS_EPSILON) ||
        //        !float_compare(this->axis2_pos, axis2_newpos, POS_EPSILON)) {
        // }

        return;
    }

    void position_x_callback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                             const std::shared_ptr<mason_test::srv::Float::Request> request,
                             const std::shared_ptr<mason_test::srv::Float::Response> /*response*/) {
        RCLCPP_INFO(this->get_logger(), "Position X callback called");
        RCLCPP_INFO(this->get_logger(), "Request: %f", request->input_pos);

        if (this->axis0_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Axis 0 error detected");
            return;
        }

        float axis0_newpos = -1 * request->input_pos + this->axis0_offset;
        move(0, axis0_newpos);

        // while (!float_compare(this->axis1_pos, axis0_newpos, POS_EPSILON)) {
        // }

        return;
    }

    void move(int axis, float pos) {
        odrive_can::msg::ControlMessage msg;
        msg.control_mode = CONTROL_MODE_POSITION_CONTROL;
        msg.input_mode = INPUT_MODE_TRAP_TRAJ;

        switch (axis) {
            case 0:
                msg.input_pos = pos;
                this->pub0->publish(msg);
                break;
            case 1:
                msg.input_pos = pos;
                this->pub1->publish(msg);
                break;
            case 2:
                msg.input_pos = pos;
                this->pub2->publish(msg);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid axis");
                break;
        }
    }

    void stop_x_callback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                         const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                         const std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/) {
        odrive_can::msg::ControlMessage control_msg;
        control_msg.control_mode = CONTROL_MODE_POSITION_CONTROL;
        control_msg.input_mode = INPUT_MODE_PASSTHROUGH;

        control_msg.input_pos = this->axis0_pos;
        this->pub0->publish(control_msg);
    }

    void stop_y_callback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                         const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                         const std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/) {
        odrive_can::msg::ControlMessage control_msg;
        control_msg.control_mode = CONTROL_MODE_POSITION_CONTROL;
        control_msg.input_mode = INPUT_MODE_PASSTHROUGH;

        control_msg.input_pos = this->axis1_pos;
        this->pub1->publish(control_msg);

        control_msg.input_pos = this->axis2_pos;
        this->pub2->publish(control_msg);
    }

    ~TestNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down node");
        // set_axis_state(this->request_axis_state_client0, 0, ODriveAxisState::AXIS_STATE_IDLE,
        // false); set_axis_state(this->request_axis_state_client1, 1,
        // ODriveAxisState::AXIS_STATE_IDLE, false);
        // set_axis_state(this->request_axis_state_client2, 2, ODriveAxisState::AXIS_STATE_IDLE,
        // false);
        RCLCPP_INFO(this->get_logger(), "Node destroyed");
    }

   private:
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client0;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client1;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client2;

    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub0;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub1;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub2;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_position;

    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub0;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub2;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub1;

    rclcpp::Service<mason_test::srv::Float>::SharedPtr position_y_service;
    rclcpp::Service<mason_test::srv::Float>::SharedPtr position_x_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_y_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_x_service;

    rclcpp::TimerBase::SharedPtr position_timer;

    float axis0_pos = 0, axis1_pos = 0, axis2_pos = 0;
    float axis0_offset = nanf(""), axis1_offset = nanf(""), axis2_offset = nanf("");
    uint32_t axis0_error = 0, axis1_error = 0, axis2_error = 0;

    bool set_axis_state(rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client, int axis,
                        ODriveAxisState state, bool wait = true) {
        while (wait && !client->wait_for_service(500ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for the service for axis %d. Exiting.",
                             axis);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service for axis %d not available, waiting again...",
                        axis);
        }

        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = static_cast<int32_t>(state);

        auto result = client->async_send_request(request);

        if (!wait) return true;

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 5s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "successfully got response state: %d",
                        response->axis_state);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for response for axis %d", axis);
            return false;
        }

        return true;
    }

    void stop(int axis) {
        odrive_can::msg::ControlMessage control_msg;
        control_msg.control_mode = CONTROL_MODE_POSITION_CONTROL;
        control_msg.input_mode = INPUT_MODE_PASSTHROUGH;

        if (axis == 0) {
            control_msg.input_pos = axis0_pos;
            this->pub0->publish(control_msg);
        } else if (axis == 1) {
            control_msg.input_pos = axis1_pos;
            this->pub1->publish(control_msg);
        } else if (axis == 2) {
            control_msg.input_pos = axis2_pos;
            this->pub2->publish(control_msg);
        }
    }

    bool float_compare(float a, float b, float epsilon) { return std::fabs(a - b) <= epsilon; }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}