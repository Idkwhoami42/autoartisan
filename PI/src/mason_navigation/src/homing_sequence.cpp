#include "mason_navigation/srv/homing_sequence.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "mason_test/srv/float.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class HomingSequenceService : public rclcpp::Node {
   public:
    HomingSequenceService() : Node("homing_sequence_service") {
        this->homing_service = this->create_service<mason_navigation::srv::HomingSequence>(
            "home_mason",
            std::bind(&HomingSequenceService::handle_homing, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3));

        this->return_to_start_service_ =
            this->create_service<mason_navigation::srv::HomingSequence>(
                "return_mason_to_start",
                std::bind(&HomingSequenceService::handle_return_to_start, this,
                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        this->contact_sensor_sub = this->create_subscription<std_msgs::msg::String>(
            "/contact_sensor_triggered", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                this->sensor = msg->data;
                RCLCPP_INFO(this->get_logger(), "%s CONTACT SENSOR TRIGGERED.", msg->data.c_str());
            });

        this->position_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odrive_position", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                this->pos_x = msg->data[0];
                this->pos_y = msg->data[1];
            });

        this->position_y_client_ = this->create_client<mason_test::srv::Float>("odrive_position_y");
        this->position_x_client = this->create_client<mason_test::srv::Float>("odrive_position_x");
        this->stop_y_client_ = this->create_client<std_srvs::srv::Empty>("odrive_stop_y");
        this->stop_x_client = this->create_client<std_srvs::srv::Empty>("odrive_stop_x");

        RCLCPP_INFO(this->get_logger(), "Service is ready to receive requests.");
    }

    bool positionServiceIsAvailable(rclcpp::Client<mason_test::srv::Float>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for position service. Exiting...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Position service not available, waiting again...");
        }
        return true;
    }

    bool stoppingServiceIsAvailable(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for stopping service. Exiting...");
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
        while (!ready) {
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

        std::this_thread::sleep_for(200ms);

        response->y_limit = this->pos_y;

        ready = false;
        while (!ready) {
            ready = this->positionServiceIsAvailable(this->position_x_client);
            ready = ready && this->stoppingServiceIsAvailable(this->stop_x_client);
        }

        this->position_x_client->async_send_request(position_request);

        while (!homed_horizontally) {
            if (this->sensor == "RIGHT") {
                this->stop_x_client->async_send_request(stop_request);
                homed_horizontally = true;
            }
        }

        std::this_thread::sleep_for(200ms);

        response->x_limit = this->pos_x;
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
        while (!ready) {
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

        std::this_thread::sleep_for(200ms);

        response->y_limit = this->pos_y;

        ready = false;
        while (!ready) {
            ready = this->positionServiceIsAvailable(this->position_x_client);
            ready = ready && this->stoppingServiceIsAvailable(this->stop_x_client);
        }

        this->position_x_client->async_send_request(position_request);

        while (true) {
            if (this->sensor == "LEFT") {
                this->stop_x_client->async_send_request(stop_request);
                break;
            }
        }

        std::this_thread::sleep_for(200ms);

        response->x_limit = this->pos_x;
    }

    rclcpp::Service<mason_navigation::srv::HomingSequence>::SharedPtr homing_service;
    rclcpp::Service<mason_navigation::srv::HomingSequence>::SharedPtr return_to_start_service_;
    rclcpp::Client<mason_test::srv::Float>::SharedPtr position_x_client;
    rclcpp::Client<mason_test::srv::Float>::SharedPtr position_y_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_x_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_y_client_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contact_sensor_sub;

    float pos_x = 0, pos_y;

    std::string sensor = "";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<HomingSequenceService> service_node = std::make_shared<HomingSequenceService>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready to be called.");

    rclcpp::spin(service_node);
    rclcpp::shutdown();

    return 0;
}