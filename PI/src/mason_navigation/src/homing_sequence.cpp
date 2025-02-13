// #include "srv/homing_sequence.hpp"// 
#include "mason_navigation/srv/homing_sequence.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "mason_hardware/contact_sensors.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <string>

// const char *chip = "gpiochip4";
std::vector<std::string> joint_names = {"motor_horizontal", "motor_left", "motor_right"};
// std::vector<std::pair<unsigned int, std::string>> contactSensorPins = {
//         {27, "RIGHT"}, {22, "TOP"}, {23, "BOTTOM"}, {24, "LEFT"}};
// std::mutex activeSensorMutex;
// std::string activeSensor = "";
// std::chrono::nanoseconds t = std::chrono::nanoseconds{10000};

// void pollPins(ContactSensors *contactSensors) {
//     while (contactSensors->keepPolling.load()) {
//         if (contactSensors) {
//             contactSensors->event_lines = contactSensors->bulk.event_wait(t);
//             if (!contactSensors->event_lines.empty()) {
//                 for (const auto &event : contactSensors->event_lines) {
//                     std::unique_lock<std::mutex> lock(activeSensorMutex);
//                     activeSensor =
//                         contactSensors->buttonCallback(event.offset(), event.event_read());
//                 }
//             }
//         }
//     }
// }

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
            std::bind(&HomingSequenceService::contact_sensor_callback, this, std::placeholders::_1)
        );

        // this->subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10,
        //     std::bind(&HomingSequenceService::joint_state_callback, this, std::placeholders::_1)
        // );

        this->publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10
        );

        RCLCPP_INFO(this->get_logger(), "Service is ready to receive requests.");
    }

private:
    void handle_homing(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<mason_navigation::srv::HomingSequence::Request> /*request*/,
      const std::shared_ptr<mason_navigation::srv::HomingSequence::Response> response) {

        bool homed_horizontally = false;
        bool homed_vertically = false;

        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {0.0, 0.5, 0.5};

        this->publisher_->publish(message);

        // while (!homed_vertically) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor == "TOP") {
        //         homed_vertically = true;
        //         message.data = {0.0, 0.0, 0.0};
        //         this->publisher_->publish(message);
        //     }
        // }

        while (!homed_vertically) {
            if (this->sensor == "TOP") {
                homed_vertically = true;
                message.data = {0.0, 0.0, 0.0};
                this->publisher_->publish(message);
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

        message.data = {0.5, 0.0, 0.0};
        this->publisher_->publish(message);

        // while (!homed_horizontally) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor == "RIGHT") {
        //         homed_horizontally = true;
        //         message.data = {0.0, 0.0, 0.0};
        //         this->publisher_->publish(message);
        //     }
        // }

        while (!homed_horizontally) {
            if (this->sensor == "RIGHT") {
                homed_horizontally = true;
                message.data = {0.0, 0.0, 0.0};
                this->publisher_->publish(message);
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

        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {0.0, -0.5, -0.5};

        this->publisher_->publish(message);

        // while (true) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor == "BOTTOM") {
        //         message.data = {0.0, 0.0, 0.0};
        //         this->publisher_->publish(message);
        //         break;
        //     }
        // }

        while (true) {
            if (this->sensor == "BOTTOM") {
                message.data = {0.0, 0.0, 0.0};
                this->publisher_->publish(message);
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

        message.data = {-0.5, 0.0, 0.0};
        this->publisher_->publish(message);

        // while (true) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor == "LEFT") {
        //         message.data = {0.0, 0.0, 0.0};
        //         this->publisher_->publish(message);
        //         break;
        //     }
        // }

        while (true) {
            if (this->sensor == "LEFT") {
                message.data = {0.0, 0.0, 0.0};
                this->publisher_->publish(message);
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

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            this->joint_positions[msg->name[i]] = msg->position[i];
        }

        for (auto joint_name : joint_names) {
            if (joint_positions.find(joint_name) != joint_positions.end()) {
                RCLCPP_INFO(this->get_logger(), "Position of %s: %.4f", joint_name.c_str(), joint_positions[joint_name]);
            }
        }
    }

    void contact_sensor_callback(const std_msgs::msg::String::SharedPtr msg) {
        this->sensor = msg->data;
    }

    rclcpp::Service<mason_navigation::srv::HomingSequence>::SharedPtr homing_service_;
    rclcpp::Service<mason_navigation::srv::HomingSequence>::SharedPtr return_to_start_service_;
    // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contact_sensor_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std::unordered_map<std::string, double> joint_positions;
    std::string sensor = "";
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    // std::unique_ptr<ContactSensors> contactSensors = std::make_unique<ContactSensors>(chip, contactSensorPins);
    // std::thread polling_thread = std::thread(pollPins, contactSensors.get());

    std::shared_ptr<HomingSequenceService> service_node = std::make_shared<HomingSequenceService>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready to be called.");

    rclcpp::spin(service_node);
    rclcpp::shutdown();

    // if (polling_thread.joinable()) polling_thread.join();

    return 0;
} 