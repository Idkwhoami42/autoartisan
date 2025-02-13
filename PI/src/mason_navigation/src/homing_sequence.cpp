#include "mason_navigation/srv/homing_sequence.hpp"
#include "mason_test/srv/float.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "odrive_can/msg/controller_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <string>

using namespace std::chrono_literals;
std::vector<std::string> joint_names = {"motor_horizontal", "motor_left", "motor_right"};

class HomingSequenceService : public rclcpp::Node {

public:
    HomingSequenceService() : Node("homing_sequence_service") {
        this->homing_service_ = this->create_service<mason_navigation::srv::HomingSequence>(
            "home_mason", 
            std::bind(&HomingSequenceService::handle_homing, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );

        this->return_to_start_service_ = this->create_service<mason_navigation::srv::HomingSequence>(
            "return_mason_to_start", 
            std::bind(&HomingSequenceService::handle_return_to_start, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );

        this->contact_sensor_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/contact_sensor_triggered", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->sensor = msg->data;
                RCLCPP_INFO(this->get_logger(), "%s CONTACT SENSOR TRIGGERED.", msg->data.c_str());
            }
        );

        this->sub0 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis0/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                if (std::isnan(axis0_offset)) axis0_offset = msg->pos_estimate;
                this->joint_positions[joint_names[0]] = msg->pos_estimate;
                RCLCPP_INFO(this->get_logger(), "%s position: %f", joint_names[0].c_str(), msg->pos_estimate);
            });
        this->sub1 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis1/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                if (std::isnan(axis1_offset)) axis1_offset = msg->pos_estimate;
                this->joint_positions[joint_names[1]] = msg->pos_estimate;
                RCLCPP_INFO(this->get_logger(), "%s position: %f", joint_names[1].c_str(), msg->pos_estimate);
            });
        this->sub2 = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "odrive_axis2/controller_status", 10,
            [this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
                if (std::isnan(axis2_offset)) axis2_offset = msg->pos_estimate;
                this->joint_positions[joint_names[2]] = msg->pos_estimate;

                RCLCPP_INFO(this->get_logger(), "%s position: %f", joint_names[2].c_str(), msg->pos_estimate);
            });

        this->position_y_client_ = this->create_client<mason_test::srv::Float>("odrive_position_y");
        this->position_x_client_ = this->create_client<mason_test::srv::Float>("odrive_position_x");
        this->stop_y_client_ = this->create_client<std_srvs::srv::Empty>("odrive_stop_y");
        this->stop_x_client_ = this->create_client<std_srvs::srv::Empty>("odrive_stop_x");

        RCLCPP_INFO(this->get_logger(), "Service is ready to receive requests.");
    }

    bool positionServiceIsAvailable(rclcpp::Client<mason_test::srv::Float>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for position service. Exiting...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Position service not available, waiting again...");
        }
        return true;
    }

    bool stoppingServiceIsAvailable(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for stopping service. Exiting...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Stopping service not available, waiting again...");
        }
        return true;
    }

private:
    void handle_homing(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<mason_navigation::srv::HomingSequence::Request> /*request*/,
      const std::shared_ptr<mason_navigation::srv::HomingSequence::Response> response) {

        bool homed_horizontally = false;
        bool homed_vertically = false;

        auto position_request = std::make_shared<mason_test::srv::Float::Request>();
        auto stop_request = std::make_shared<std_srvs::srv::Empty::Request>();
        position_request->input_pos = 1000.0f;

        bool ready = false;
        while(!ready) {
            ready = this->positionServiceIsAvailable(this->position_y_client_);
            ready = ready && this->stoppingServiceIsAvailable(this->stop_y_client_);
        }

        this->position_y_client_->async_send_request(position_request);

        while (!homed_vertically) {
            if (this->sensor == "TOP") {
                this->stop_y_client_->async_send_request(stop_request);
                homed_vertically = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (joint_positions.find("motor_left") == joint_positions.end() || joint_positions.find("motor_right") == joint_positions.end()) {
            RCLCPP_ERROR(this->get_logger(),
                "Error occurred during homing: Joint \"motor_left\" and/or \"motor_right\" not found.");
        }

        if (std::abs((this->joint_positions["motor_left"] - this->joint_positions["motor_right"])) > 1.0f) {
            RCLCPP_ERROR(this->get_logger(),
                "Error occurred during homing: Left and right vertical motors seem misaligned.");
        }

        response->y_limit = std::min(this->joint_positions["motor_left"], this->joint_positions["motor_right"]);

        ready = false;
        while(!ready) {
            ready = this->positionServiceIsAvailable(this->position_x_client_);
            ready = ready && this->stoppingServiceIsAvailable(this->stop_x_client_);
        }

        this->position_x_client_->async_send_request(position_request);

        while (!homed_horizontally) {
            if (this->sensor == "RIGHT") {
                this->stop_x_client_->async_send_request(stop_request);
                homed_horizontally = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (joint_positions.find("motor_horizontal") == joint_positions.end()) {
            RCLCPP_ERROR(this->get_logger(),
                "Error occurred during homing: Joint \"motor_horizontal\" not found.");
        }

        response->x_limit = this->joint_positions["motor_horizontal"];
    }

    // Can change to a std_srvs/srv/Trigger type of Service
    // If getting the values of min_x and min_y proves unnecessary
    void handle_return_to_start(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<mason_navigation::srv::HomingSequence::Request> /*request*/,
      const std::shared_ptr<mason_navigation::srv::HomingSequence::Response> response) {

        auto position_request = std::make_shared<mason_test::srv::Float::Request>();
        auto stop_request = std::make_shared<std_srvs::srv::Empty::Request>();
        position_request->input_pos = -1000.0f;

        bool ready = false;
        while(!ready) {
            ready = this->positionServiceIsAvailable(this->position_y_client_);
            ready = ready && this->stoppingServiceIsAvailable(this->stop_y_client_);
        }

        this->position_y_client_->async_send_request(position_request);

        while (true) {
            if (this->sensor == "BOTTOM") {
                this->stop_y_client_->async_send_request(stop_request);
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (joint_positions.find("motor_left") == joint_positions.end() || joint_positions.find("motor_right") == joint_positions.end()) {
            RCLCPP_ERROR(this->get_logger(),
                "Error occurred during homing: Joint \"motor_left\" and/or \"motor_right\" not found.");
        }

        if (std::abs((this->joint_positions["motor_left"] - this->joint_positions["motor_right"])) > 1.0f) {
            RCLCPP_ERROR(this->get_logger(),
                "Error occurred during homing: Left and right vertical motors seem misaligned.");
        }

        response->y_limit = std::min(this->joint_positions["motor_left"], this->joint_positions["motor_right"]);

        ready = false;
        while(!ready) {
            ready = this->positionServiceIsAvailable(this->position_x_client_);
            ready = ready && this->stoppingServiceIsAvailable(this->stop_x_client_);
        }

        this->position_x_client_->async_send_request(position_request);

        while (true) {
            if (this->sensor == "LEFT") {
                this->stop_x_client_->async_send_request(stop_request);
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (joint_positions.find("motor_horizontal") == joint_positions.end()) {
            RCLCPP_ERROR(this->get_logger(),
                "Error occurred during homing: Joint \"motor_horizontal\" not found.");
        }

        response->x_limit = this->joint_positions["motor_horizontal"];
    }

    rclcpp::Service<mason_navigation::srv::HomingSequence>::SharedPtr homing_service_;
    rclcpp::Service<mason_navigation::srv::HomingSequence>::SharedPtr return_to_start_service_;
    rclcpp::Client<mason_test::srv::Float>::SharedPtr position_x_client_;
    rclcpp::Client<mason_test::srv::Float>::SharedPtr position_y_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_x_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_y_client_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub0;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub2;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub1;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contact_sensor_sub_;
    std::unordered_map<std::string, float> joint_positions;
    float axis0_offset = nanf(""), axis1_offset = nanf(""), axis2_offset = nanf("");
    std::string sensor = "";
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<HomingSequenceService> service_node = std::make_shared<HomingSequenceService>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready to be called.");

    rclcpp::spin(service_node);
    rclcpp::shutdown();

    return 0;
} 