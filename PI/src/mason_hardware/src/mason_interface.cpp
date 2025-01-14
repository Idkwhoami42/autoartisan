#include "mason_hardware/mason_interface.hpp"

#include <chrono>
#include <limits>

#include "cmath"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// I have given it the same radius as the motor as a placeholder
#define WHEEL_RADIUS 0.0315

namespace mason_hardware {

MasonInterface::MasonInterface() {}

MasonInterface::~MasonInterface() {
    if (this->cfg_.port->is_open()) {
        this->cfg_.port->close();
    }
}

void MasonInterface::homingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // try {
    //     this->motorL_->doHoming(this->cfg_.port);
    //     this->motorR_->doHoming(this->cfg_.port);
    //     this->motorT_->doHoming(this->cfg_.port);

    //     while (this->contactSensors->mostRecentlyActivated() != "right") {
    //         this->motorT_->goToPos(this->cfg_.port, 360);
    //     }
    //     this->motorT_->setDuty(this->cfg_.port, 0);

    //     this->max_x_ = this->motorT_->rotations;

    //     while (this->contactSensors->mostRecentlyActivated() != "top_right") {
    //         this->motorR_->goToPos(this->cfg_.port, 360);
    //         this->motorL_->goToPos(this->cfg_.port, 360);
    //     }
    //     this->motorR_->setDuty(this->cfg_.port, 0);
    //     this->motorL_->setDuty(this->cfg_.port, 0);

    //     this->max_y_ = (this->motorR_->rotations + this->motorL_->rotations) / 2.0;
    // } catch (std::exception &e) {
    //     std::cout << "Error during homing: " << e.what() << std::endl;
    // }
}

void MasonInterface::returnToStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // try {
    //     this->motorT_->direction = 1;
    //     this->motorR_->direction = 1;
    //     this->motorL_->direction = 1;

    //     while (this->contactSensors->mostRecentlyActivated() != "left") {
    //         // 0 = CW, 1 = CCW
    //         this->motorT_->goToPos(this->cfg_.port, 360);
    //     }
    //     this->motorT_->setDuty(this->cfg_.port, 0);

    //     while (this->contactSensors->mostRecentlyActivated() != "bottom_left") {
    //         this->motorR_->goToPos(this->cfg_.port, 360);
    //         this->motorL_->goToPos(this->cfg_.port, 360);
    //     }
    //     this->motorR_->setDuty(this->cfg_.port, 0);
    //     this->motorL_->setDuty(this->cfg_.port, 0);

    //     response->success = true;
    //     response->message = "Successfully returned to start.\n";

    // } catch (std::exception &e) {
    //     response->success = false;
    //     const char *msg;
    //     scanf(msg, "Error whilst returning to start: %s.\n", e.what());
    //     response->message = std::string(msg);
    // }
}

void MasonInterface::initializeServices(rclcpp::Node::SharedPtr node) {
    this->homing_service_ = node->create_service<std_srvs::srv::Trigger>(
        "home_robot", std::bind(&MasonInterface::homingCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->reset_service_ = node->create_service<std_srvs::srv::Trigger>(
        "back_to_start", std::bind(&MasonInterface::returnToStart, this, std::placeholders::_1, std::placeholders::_2));
}

hardware_interface::CallbackReturn MasonInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // std::vector<std::pair<int, std::string>> contactSensorPins = {};
    // std::vector<std::string> sensorNames = {"bottom_left", "left", "top_left", "top_right", "right", "bottom_right"};
    this->cfg_.device = info_.hardware_parameters["device"].c_str();

    std::string left_motor_name = info_.hardware_parameters["left_motor_name"].c_str();
    std::string right_motor_name = info_.hardware_parameters["right_motor_name"].c_str();
    std::string horizontal_motor_name = info_.hardware_parameters["horizontal_motor_name"].c_str();

    int left_motor_id = std::stoi(info_.hardware_parameters["left_motor_id"]);
    int right_motor_id = std::stoi(info_.hardware_parameters["right_motor_id"]);
    int horizontal_motor_id = std::stoi(info_.hardware_parameters["horizontal_motor_id"]);

    int baudRate = std::stoi(info_.hardware_parameters["baud_rate"]);

    // // double timeout_ms = std::stod(info_.hardware_parameters["timeout_ms"]);
    // for (auto name : sensorNames) {
    //     contactSensorPins.push_back(std::make_pair(std::stoi(info_.hardware_parameters[name]), name));
    // }
    // ContactSensors comms(contactSensorPins);
    // this->contactSensors = &comms;
    // // angular_velocity_max = Power_max / Torque_max

    double angular_velocity = static_cast<double>(2450) / static_cast<double>(7);

    // // RPM_max = angular_velocity * 30 / pi
    // // v_max = angular_velocity * radius

    // // double rpmMax = angular_velocity * (static_cast<double>(30) / static_cast<double>(M_PI));
    this->cfg_.max_velocity_ = angular_velocity * WHEEL_RADIUS;

    // // conversion_factor = 1 / v_max
    this->cfg_.duty_conversion_factor_ = static_cast<double>(1) / this->cfg_.max_velocity_;

    try {
        // Create and open serial port
        boost::asio::io_context io;
        boost::asio::serial_port port(io, this->cfg_.device);

        port.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        this->cfg_.port = &port;
    } catch (std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Error occurred: %s.", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    Motor motor_left(left_motor_name, left_motor_id, true);
    Motor motor_right(right_motor_name, right_motor_id, true);
    Motor motor_horizontal(horizontal_motor_name, horizontal_motor_id, false);

    this->motorL_ = &motor_left;
    this->motorR_ = &motor_right;
    this->motorH_ = &motor_horizontal;

    Controller controller(this->cfg_.port, {left_motor_id, right_motor_id});

    this->position_states_.resize(2, std::numeric_limits<double>::quiet_NaN());
    this->position_commands_.resize(2, std::numeric_limits<double>::quiet_NaN());

    auto node = rclcpp::Node::make_shared("mason_interface_node");
    initializeServices(node);

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Hardware interface activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MasonInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces = {};

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "x_position_state", hardware_interface::HW_IF_POSITION, &this->position_states_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "y_position_state", hardware_interface::HW_IF_POSITION, &this->position_states_[1]));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MasonInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces = {};

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "x_position_cmd", hardware_interface::HW_IF_POSITION, &this->position_commands_[0]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "y_position_cmd", hardware_interface::HW_IF_POSITION, &this->position_commands_[1]));

    return command_interfaces;
}

hardware_interface::CallbackReturn MasonInterface::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Activating ...please wait...");

    if (!this->cfg_.port->is_open()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Deactivating ...please wait...");

    // Close port if open
    if (this->cfg_.port->is_open()) {
        this->cfg_.port->close();
    }

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MasonInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (!this->cfg_.port->is_open()) {
        return hardware_interface::return_type::ERROR;
    }

    double delta_seconds = period.seconds();

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Getting values for left motor . . .");
    double prev_pos = this->motorL_->pos;
    bool res = this->motorL_->getValues(this->cfg_.port);
    if (!res) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"),
                    "getValues returned false for left motor. Errors may cause inaccuracies in values printed");
        return hardware_interface::return_type::ERROR;
    }
    this->motorL_->vel = (this->motorL_->pos - prev_pos) / delta_seconds;
    this->motorL_->printValues();

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Getting values for right motor . . .");
    prev_pos = this->motorR_->pos;
    res = this->motorR_->getValues(this->cfg_.port);
    if (!res) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"),
                    "getValues returned false for right motor. Errors may cause inaccuracies in values printed");
        return hardware_interface::return_type::ERROR;
    }
    this->motorR_->vel = (this->motorR_->pos - prev_pos) / delta_seconds;
    this->motorR_->printValues();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MasonInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (!this->cfg_.port->is_open()) {
        return hardware_interface::return_type::ERROR;
    }

    double clamped_velocity_L = std::clamp(this->motorL_->cmd, -this->cfg_.max_velocity_, this->cfg_.max_velocity_);
    double clamped_velocity_R = std::clamp(this->motorR_->cmd, -this->cfg_.max_velocity_, this->cfg_.max_velocity_);
    int duty_L = static_cast<int>(clamped_velocity_L * this->cfg_.duty_conversion_factor_);
    int duty_R = static_cast<int>(clamped_velocity_R * this->cfg_.duty_conversion_factor_);
    int duty = static_cast<int>(((duty_R + duty_L) / 2));

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Writing commands for left motor . . .");
    this->motorL_->setDuty(this->cfg_.port, duty);

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Writing commands for right motor . . .");
    this->motorR_->setDuty(this->cfg_.port, duty);

    return hardware_interface::return_type::OK;
}
}  // namespace mason_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mason_hardware::MasonInterface, hardware_interface::SystemInterface)