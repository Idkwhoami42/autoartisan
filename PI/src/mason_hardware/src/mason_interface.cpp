#include "../include/mason_hardware/mason_interface.hpp"
// #include "../include/mason_hardware/contact_sensors.hpp"

#include <chrono>
#include "cmath"
#include <limits>
#include <stdint.h>
#include <thread>
#include <mutex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// I have given it the same radius as the motor as a placeholder
#define WHEEL_RADIUS 0.0315
const char *chip = "gpiochip4";
// std::mutex activeSensorMutex;
// std::string activeSensor = "";
// std::thread pollingThread;
// std::chrono::nanoseconds t = std::chrono::nanoseconds{10000};

namespace mason_hardware {

    HomingPublisher::HomingPublisher() : Node("mason_homing_publisher") {
        this->rotation_lim_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("limits", rclcpp::QoS(1).transient_local());
    }

    void HomingPublisher::publishLimits(const double x_limit, const double y_limit, const bool success) {
        auto message = geometry_msgs::msg::Point();
        message.x = static_cast<float>(x_limit);
        message.y = static_cast<float>(y_limit);
        message.z = (success) ? 1.0 : 0.0;
        this->rotation_lim_publisher_->publish(message);
    }

    MasonInterface::MasonInterface() = default;

    MasonInterface::~MasonInterface() {
        // if (this->cfg_.port->is_open()) {
        //     this->cfg_.port->close();
        // }

        if (this->motorL_) {
            delete this->motorL_;
        }
        if (this->motorR_) {
            delete this->motorR_;
        }
        if (this->motorH_) {
            delete this->motorH_;
        }
    }

    // void pollPins(ContactSensors* contactSensors) {
    //     while (contactSensors->keepPolling.load()) {
    //         if (contactSensors) {
    //             contactSensors->event_lines = contactSensors->bulk.event_wait(t);
    //             if (!contactSensors->event_lines.empty()) {
    //                 for (const auto& event : contactSensors->event_lines) {
    //                     std::unique_lock<std::mutex> lock(activeSensorMutex);
    //                     activeSensor = contactSensors->buttonCallback(event.offset(), event.event_read());
    //                 }
    //             }
    //         }
    //     }
    // }

    void MasonInterface::homingCallback() {
        // START_COMMENT
        try {
            auto name_pos_x = std::string("MotorTop/") + hardware_interface::HW_IF_POSITION;
            auto name_pos_y = std::string("MotorJointLeft/") + hardware_interface::HW_IF_POSITION;
            this->motorL_->doHoming(this->cfg_.port);
            this->homing_done_ = this->motorL_->verifyHoming();
            this->motorR_->doHoming(this->cfg_.port);
            this->homing_done_ = this->homing_done_ && this->motorR_->verifyHoming();
            this->motorH_->doHoming(this->cfg_.port);
            this->homing_done_ = this->homing_done_ && this->motorH_->verifyHoming();

            double init_pos = 360;
            // while (true) {
            //     std::string sensor = "";
            //     {
            //         std::unique_lock<std::mutex> lock(activeSensorMutex);
            //         sensor = activeSensor;
            //     }
            //     if (sensor != "right") {
            //         this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorH_});
            //         init_pos = init_pos + 360;
            //     }
            //     if (sensor == "right") break;
            // }

            int i = 0;
            while (i < 8) {
                this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorH_});
                init_pos = init_pos + 360;
                i += 1;
            }
            this->motorH_->setDuty(this->cfg_.port, 0);

            this->max_x_ = this->motorH_->pos;
            // This was not technically a command, but don't want the motor to try to travel to a position it's already at
            this->prev_position_commands_[0] = this->max_x_;

            init_pos = 360.0;
            // while (true) {
            //     std::string sensor = "";
            //     {
            //         std::unique_lock<std::mutex> lock(activeSensorMutex);
            //         sensor = activeSensor;
            //     }
            //     if (sensor != "top-right" || sensor != "top-left") {
            //         this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorL_, this->motorR_});
            //         init_pos = init_pos + 360.0;
            //     }
            //     if (sensor == "top-right" || sensor == "top-left") break;
            // }
            i = 0;
            while (i < 8) {
                this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorL_, this->motorR_});
                init_pos = init_pos + 360;
                i += 1;
            }
            this->motorR_->setDuty(this->cfg_.port, 0);
            this->motorL_->setDuty(this->cfg_.port, 0);

            this->max_y_ = (this->motorR_->pos + this->motorL_->pos) / 2.0;
            set_state(name_pos_y, this->max_y_);
            this->prev_position_commands_[1] = this->max_y_;
        } catch (std::exception &e) {
            std::cout << "Error during homing: " << e.what() << std::endl;
        }
        // END_COMMENT
        this->homing_done_ = true;
        initializePublisher();

        
        this->executor_->spin_node_once(this->homing_pub_);
        this->homing_pub_->publishLimits(this->max_x_, this->max_y_, this->homing_done_);
    }

    void MasonInterface::returnToStart() {
        // START_COMMENT
        try {
            if (!this->homing_done_) {
                double init_pos = -360.0;
                // while (true) {
                //     std::string sensor = "";
                //     {
                //         std::unique_lock<std::mutex> lock(activeSensorMutex);
                //         sensor = activeSensor;
                //     }
                //     if (sensor != "left") {
                //         this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorH_});
                //         init_pos = init_pos - 360.0;
                //     }
                //     if (sensor == "left") break;
                // }
                int i = 6;
                while (i > 0) {
                    this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorH_});
                    init_pos = init_pos - 360;
                    i -= 1;
                }
                
                this->motorH_->setDuty(this->cfg_.port, 0.0);

                init_pos = -360.0;
                // while (true) {
                //     std::string sensor = "";
                //     {
                //         std::unique_lock<std::mutex> lock(activeSensorMutex);
                //         sensor = activeSensor;
                //     }
                //     if (sensor != "bottom-left") {
                //         this->controller_->goToPos(this->cfg_.port, 360, {this->motorL_, this->motorR_});
                //         init_pos = init_pos - 360;
                //     }
                //     if (sensor == "bottom-left") break;
                // }
                i = 6;
                while (i > 0) {
                    this->controller_->goToPos(this->cfg_.port, init_pos, {this->motorL_, this->motorR_});
                    init_pos = init_pos - 360;
                    i -= 1;
                }
                this->motorR_->setDuty(this->cfg_.port, 0);
                this->motorL_->setDuty(this->cfg_.port, 0);
            } else {
                this->controller_->goToPos(this->cfg_.port, 0.0, {this->motorH_, this->motorL_, this->motorR_});
            }
            this->prev_position_commands_[0] = 0.0;
            this->prev_position_commands_[1] = 0.0;
        } catch (std::exception &e) {
            this->at_starting_point_ = false;
            printf("Error whilst returning to start: %s.\n", e.what());
        }
        // END_COMMENT
        this->at_starting_point_ = true;
    }

    void MasonInterface::initializePublisher() {
        if (!this->executor_) {
            this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        }
    }

    hardware_interface::CallbackReturn MasonInterface::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        this->cfg_.device = info_.hardware_parameters["device"].c_str();
        std::string leftMotor = info_.hardware_parameters["left_motor_name"].c_str();
        std::string rightMotor = info_.hardware_parameters["right_motor_name"].c_str();
        std::string horizontalMotor = info_.hardware_parameters["horizontal_motor_name"].c_str();
        int baudRate = std::stoi(info_.hardware_parameters["baud_rate"]);
        int left_motor_id = std::stoi(info_.hardware_parameters["id_left"]);
        int right_motor_id = std::stoi(info_.hardware_parameters["id_right"]);
        int horizontal_motor_id = std::stoi(info_.hardware_parameters["id_horizontal"]);
        // double timeout_ms = std::stod(info_.hardware_parameters["timeout_ms"]);

        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - LEFT_MOTOR_NAME: %s", leftMotor.c_str());
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - RIGHT_MOTOR_NAME: %s", rightMotor.c_str());
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - HORIZONTAL_MOTOR_NAME: %s", horizontalMotor.c_str());
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - LEFT_MOTOR_ID: %d", left_motor_id);
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - RIGHT_MOTOR_ID: %d", right_motor_id);
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - HORIZONTAL_MOTOR_ID: %d", horizontal_motor_id);
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - DEVICE: %s", this->cfg_.device.c_str());
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "PARAM - BAUD_RATE: %d", baudRate);

        std::vector<std::pair<unsigned int, std::string>> contactSensorPins = {
            {1, "bottom_left"},
            {13, "left"},
            {3, "top_left"},
            {4, "top_right"},
            {26, "right"},
            {2, "bottom_right"}
        };

        // ContactSensors comms(chip, contactSensorPins);

        // this->contactSensors = &comms;

        // angular_velocity_max = Power_max / Torque_max
        double angular_velocity = static_cast<double>(2450) / static_cast<double>(7);

        // RPM_max = angular_velocity * 30 / pi
        // v_max = angular_velocity * radius

        // double rpmMax = angular_velocity * (static_cast<double>(30) / static_cast<double>(M_PI));
        this->cfg_.max_velocity_ = angular_velocity * WHEEL_RADIUS;

        // conversion_factor = 1 / v_max
        this->cfg_.duty_conversion_factor_ = static_cast<double>(1) / this->cfg_.max_velocity_;

        try {
            // START_COMMENT
            boost::asio::io_context io;
            boost::asio::serial_port port(io, this->cfg_.device);

            port.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
            port.set_option(boost::asio::serial_port_base::character_size(8));
            port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            this->cfg_.port = &port;

            Motor motor_left(leftMotor, left_motor_id, true);
            Motor motor_right(rightMotor, right_motor_id, true);
            Motor motor_horizontal(horizontalMotor, horizontal_motor_id, false); 
            Controller controller(this->cfg_.port, {left_motor_id, right_motor_id});

            this->motorL_ = &motor_left;
            this->motorR_ = &motor_right;
            this->motorH_ = &motor_horizontal;
            this->controller_ = &controller;
            // END_COMMENT 

            this->prev_position_commands_.resize(2, std::numeric_limits<double>::quiet_NaN());

        } catch (std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Error occurred: %s.", e.what());
        }

        this->homing_pub_ = std::make_shared<HomingPublisher>();

        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Hardware interface activated.");
        // pollingThread = std::thread(pollPins, this->contactSensors);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MasonInterface::on_activate(const rclcpp_lifecycle::State &/*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Activating...please wait...");

        if (!this->cfg_.port->is_open()) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Executing homing sequence...please wait...");
        while (!this->at_starting_point_) {
            this->returnToStart();
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Mason is at start point: %d", this->at_starting_point_);
        }
        while (!this->homing_done_) {
            this->homingCallback();
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Mason is homed: %d", this->homing_done_);
        }

        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MasonInterface::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Deactivating ...please wait...");

        this->returnToStart();
        // Close port if open
        if (this->cfg_.port->is_open()) {
            this->cfg_.port->close();
        }

        // Call destructors
        
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MasonInterface::read(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
        if (!this->cfg_.port->is_open()) {
            return hardware_interface::return_type::ERROR;
        }

        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            if (info_.joints[i].name == this->motorH_->joint_name) {
                set_state(name_pos, this->motorH_->pos);
                set_state(name_pos, 360);
            }

            if (info_.joints[i].name == this->motorL_->joint_name) {
                if (double diff = std::abs((this->motorR_->pos - this->motorL_->pos)); diff > 2) {
                    return hardware_interface::return_type::ERROR;
                }
                set_state(name_pos, this->motorL_->pos);
                set_state(name_pos, 360);
            }
        }

        return hardware_interface::return_type::OK;
    }

    void MasonInterface::updatePosX(const auto name_pos) {
        double desired_pos = std::clamp(0.0, get_command(name_pos), this->max_x_);

        this->controller_->goToPos(this->cfg_.port, desired_pos, {this->motorH_});
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Command written for x position: %f", desired_pos);
        set_state(name_pos, desired_pos);
    }

    void MasonInterface::updatePosY(const auto name_pos) {
        double desired_pos = std::clamp(0.0, get_command(name_pos), this->max_y_);

        this->controller_->goToPos(this->cfg_.port, desired_pos, {this->motorL_, this->motorR_});
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Command written for y position: %f", desired_pos);
        set_state(name_pos, desired_pos);
    }

    hardware_interface::return_type MasonInterface::write(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
        if (!this->cfg_.port->is_open()) {
            return hardware_interface::return_type::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Write function called!");
        for (size_t i = 0; i < info_.joints.size(); i++) {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Joint: %s", name_pos.c_str());
            const double cmd = get_command(name_pos);
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Command: %f", cmd);
            if (double diff = std::abs(this->prev_position_commands_[i] - cmd); diff < 1e-9) {
                RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Diff: %f", diff);
                if (info_.joints[i].name == this->motorH_->joint_name) {
                    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Writing commands for x position . . .");
                    updatePosX(name_pos);
                }

                if (info_.joints[i].name == this->motorL_->joint_name) {
                    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Writing commands for y position . . .");
                    updatePosY(name_pos);
                }
                this->prev_position_commands_[i] = cmd;
                RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Previous command: %f", this->prev_position_commands_[i]);
                RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully navigated to position");
            }
        }

        return hardware_interface::return_type::OK;
    }
} // namespace mason_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mason_hardware::MasonInterface, hardware_interface::SystemInterface)