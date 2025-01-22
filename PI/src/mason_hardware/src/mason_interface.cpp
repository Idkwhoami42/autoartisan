#include "mason_hardware/mason_interface.hpp"

#include <stdint.h>

#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "cmath"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// I have given it the same radius as the motor as a placeholder
#define WHEEL_RADIUS 0.0315
const char *chip = "gpiochip4";
std::mutex activeSensorMutex;
std::string activeSensor = "";
std::chrono::nanoseconds t = std::chrono::nanoseconds{10000};

namespace mason_hardware {

HomingPublisher::HomingPublisher() : Node("mason_homing_publisher") {
    this->rotation_lim_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
        "limits", rclcpp::QoS(1).transient_local());
}

void HomingPublisher::publishLimits(const double x_limit, const double y_limit,
                                    const bool success) {
    auto message = geometry_msgs::msg::Point();
    message.x = static_cast<float>(x_limit);
    message.y = static_cast<float>(y_limit);
    message.z = (success) ? 1.0 : 0.0;
    this->rotation_lim_publisher_->publish(message);
}

MasonInterface::MasonInterface() : serial(nullptr), controller_(nullptr), contactSensors(nullptr){};

MasonInterface::~MasonInterface() {
    if (this->serial->is_open()) {
        this->serial->close();
    }

    if (this->write_thread_.joinable()) this->write_thread_.join();
    if (this->polling_thread_.joinable()) this->polling_thread_.join();
}

void pollPins(ContactSensors *contactSensors) {
    while (contactSensors->keepPolling.load()) {
        if (contactSensors) {
            contactSensors->event_lines = contactSensors->bulk.event_wait(t);
            if (!contactSensors->event_lines.empty()) {
                for (const auto &event : contactSensors->event_lines) {
                    std::unique_lock<std::mutex> lock(activeSensorMutex);
                    activeSensor =
                        contactSensors->buttonCallback(event.offset(), event.event_read());
                }
            }
        }
    }
}

bool MasonInterface::homingCallback() {
    // START_COMMENT
    try {
        std::string name_pos_y =
            std::string("MotorJointLeft/") + hardware_interface::HW_IF_POSITION;
        for (auto &motor : {&this->motorH_, &this->motorL_, &this->motorR_}) {
            motor->doHoming(this->serial.get());
            if (!motor->verifyHoming()) {
                RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"),
                             "Homing went wrong with motor with id %d", motor->id);
                return false;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("MasonInterface"),
                            "Homing went well with motor with id %d", motor->id);
            }
        }

        // // double init_pos = 360;
        // while (true) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor != "RIGHT") {
        //         this->motorH_.setDuty(this->serial.get(), 0.3);
        //     }
        //     if (sensor == "RIGHT") break;
        // }

        // // int i = 0;
        // // while (i < 8) {
        // //     this->controller_->goToPos(this->serial.get(), init_pos, {this->motorH_});
        // //     init_pos = init_pos + 360;
        // //     i += 1;
        // // }
        // this->motorH_.setDuty(this->serial.get(), 0);

        // this->max_x_ = this->motorH_.pos;
        // // This was not technically a command, but don't want the motor to try to travel to a
        // // position it's already at
        // this->prev_position_commands_[0] = this->max_x_;

        // // init_pos = 360.0;
        // while (true) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor != "TOP") {
        //         this->motorL_.setDuty(this->serial.get(), 0.3);
        //         this->motorR_.setDuty(this->serial.get(), 0.3);
        //         // this->controller_->goToPos(this->serial.get(), init_pos, {this->motorL_,
        //         // this->motorR_}); init_pos = init_pos + 360.0;
        //     }
        //     if (sensor == "TOP") break;
        // }
        // // i = 0;
        // // while (i < 8) {
        // //     this->controller_->goToPos(this->serial.get(), init_pos, {this->motorL_,
        // //     this->motorR_}); init_pos = init_pos + 360; i += 1;
        // // }
        // this->motorR_.setDuty(this->serial.get(), 0);
        // this->motorL_.setDuty(this->serial.get(), 0);

        // this->max_y_ = (this->motorR_.pos + this->motorL_.pos) / 2.0;
        // set_state(name_pos_y, this->max_y_);
        // this->prev_position_commands_[1] = this->max_y_;
    } catch (std::exception &e) {
        std::cout << "Error during homing: " << e.what() << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Homing failed");
        return false;
    }
    // END_COMMENT

    initializePublisher();

    this->executor_->spin_node_once(this->homing_pub_);
    this->homing_pub_->publishLimits(this->max_x_, this->max_y_, true);

    return true;
}

bool MasonInterface::returnToStart() {
    // START_COMMENT
    try {
        while (true) {
            std::string sensor = "";
            {
                std::unique_lock<std::mutex> lock(activeSensorMutex);
                sensor = activeSensor;
            }
            if (sensor != "LEFT") {
                this->motorH_.setDuty(this->serial.get(), -0.03);
            }
            if (sensor == "LEFT") break;
        }

        this->motorH_.setDuty(this->serial.get(), 0.0);

        // init_pos = -360.0;
        // while (true) {
        //     std::string sensor = "";
        //     {
        //         std::unique_lock<std::mutex> lock(activeSensorMutex);
        //         sensor = activeSensor;
        //     }
        //     if (sensor != "BOTTOM") {
        //         this->motorL_.setDuty(this->serial.get(), -0.3);
        //         this->motorR_.setDuty(this->serial.get(), -0.3);
        //         // this->controller_->goToPos(this->serial.get(), 360, {this->motorL_,
        //         // this->motorR_}); init_pos = init_pos - 360;
        //     }
        //     if (sensor == "BOTTOM") break;
        // }

        this->motorR_.setDuty(this->serial.get(), 0);
        this->motorL_.setDuty(this->serial.get(), 0);

        this->prev_horizontal_position_ = 0.0;
        this->prev_vertical_position_ = 0.0;
    } catch (std::exception &e) {
        return false;
        printf("Error whilst returning to start: %s.\n", e.what());
    }

    return true;
}

void MasonInterface::initializePublisher() {
    if (!this->executor_) {
        this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
}

hardware_interface::CallbackReturn MasonInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string device = info_.hardware_parameters["device"];
    std::string left_motor_name = info_.hardware_parameters["left_motor_name"];
    std::string right_motor_name = info_.hardware_parameters["right_motor_name"];
    std::string horizontal_motor_name = info_.hardware_parameters["horizontal_motor_name"];
    int left_motor_id = std::stoi(info_.hardware_parameters["id_left"]);
    int right_motor_id = std::stoi(info_.hardware_parameters["id_right"]);
    int horizontal_motor_id = std::stoi(info_.hardware_parameters["id_horizontal"]);

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"),
                "PARAM - LEFT_MOTOR_NAME: %s\n"
                "PARAM - RIGHT_MOTOR_NAME: %s\n"
                "PARAM - HORIZONTAL_MOTOR_NAME: %s\n"
                "PARAM - LEFT_MOTOR_ID: %d\n"
                "PARAM - RIGHT_MOTOR_ID: %d\n"
                "PARAM - HORIZONTAL_MOTOR_ID: %d\n"
                "PARAM - DEVICE: %s\n",
                left_motor_name.c_str(), right_motor_name.c_str(), horizontal_motor_name.c_str(),
                left_motor_id, right_motor_id, horizontal_motor_id, device.c_str());

    // TODO: UPDATE PIN NUMBERS
    std::vector<std::pair<unsigned int, std::string>> contactSensorPins = {
        {27, "RIGHT"}, {22, "TOP"}, {23, "BOTTOM"}, {24, "LEFT"}};

    this->contactSensors.reset(new ContactSensors(chip, contactSensorPins));

    // angular_velocity_max = Power_max / Torque_max
    double angular_velocity = static_cast<double>(2450) / static_cast<double>(7);

    // RPM_max = angular_velocity * 30 / pi
    // v_max = angular_velocity * radius

    // double rpmMax = angular_velocity * (static_cast<double>(30) / static_cast<double>(M_PI));
    this->cfg_.max_velocity_ = angular_velocity * WHEEL_RADIUS;

    // conversion_factor = 1 / v_max
    this->cfg_.duty_conversion_factor_ = static_cast<double>(1) / this->cfg_.max_velocity_;

    try {
        this->serial.reset(new boost::asio::serial_port(this->io, device));
        this->serial->set_option(boost::asio::serial_port_base::baud_rate(115200));
        this->serial->set_option(boost::asio::serial_port_base::character_size(8));
        this->serial->set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        this->serial->set_option(
            boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        this->serial->set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));

        this->motorL_ = Motor(left_motor_name, left_motor_id, serial.get(), true);
        this->motorR_ = Motor(right_motor_name, right_motor_id, serial.get(), true);
        this->motorH_ = Motor(horizontal_motor_name, horizontal_motor_id, serial.get(), true);

        this->controller_.reset(new Controller(
            this->serial.get(), {left_motor_id, right_motor_id, horizontal_motor_id}));

    } catch (std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Error occurred: %s.", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->homing_pub_ = std::make_shared<HomingPublisher>();

    this->polling_thread_ = std::thread(pollPins, this->contactSensors.get());

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Hardware interface activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Activating...please wait...");

    if (!this->serial->is_open()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Returning to start...please wait...");

    // if (!this->returnToStart()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Return to start failed");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Executing homing sequence...please wait...");

    bool homing_done = this->homingCallback();
    if (!homing_done) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Homing failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->write_thread_ = std::thread(&MasonInterface::writeWorker, this);

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Deactivating ...please wait...");

    // this->returnToStart();
    // Close port if open
    if (this->serial->is_open()) {
        this->serial->close();
    }

    // Call destructors

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MasonInterface::read(const rclcpp::Time & /*time*/,
                                                     const rclcpp::Duration & /*period*/) {
    if (!this->serial->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Serial port is not open");
        return hardware_interface::return_type::ERROR;
    }

    if (!check_joints()) {
        return hardware_interface::return_type::ERROR;
    }

    std::string horizontal_name_pos = motorH_.joint_name + "/" + hardware_interface::HW_IF_POSITION;
    std::string left_name_pos = motorL_.joint_name + "/" + hardware_interface::HW_IF_POSITION;

    if (double diff = std::abs((this->motorR_.pos - this->motorL_.pos)); diff >= 2) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"),
                     "Difference between right and left motors is too big; diff = %f", diff);
        return hardware_interface::return_type::ERROR;
    }

    set_state(horizontal_name_pos, this->motorH_.pos);
    set_state(left_name_pos, this->motorL_.pos);

    return hardware_interface::return_type::OK;
}

void MasonInterface::writeWorker() {
    while (true) {
        if (this->write_queue_.empty()) continue;

        auto front = this->write_queue_.front();
        this->write_queue_.pop();

        auto &[horizontal_pos, horizontal_name_pos, skipHorizontal] = front.first;
        auto &[vertical_pos, vertical_name_pos, skipVertical] = front.second;

        // TODO: remove this
        // horizontal_pos = std::clamp(0.0, horizontal_pos, this->max_x_);
        // vertical_pos = std::clamp(0.0, vertical_pos, this->max_y_);

        if (!skipHorizontal) {
            this->controller_->goToPos(this->serial.get(), horizontal_pos, {&this->motorH_});
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Command written for x position: %f",
                        horizontal_pos);
            set_state(horizontal_name_pos, horizontal_pos);
        }

        if (!skipVertical) {
            this->controller_->goToPos(this->serial.get(), vertical_pos,
                                       {&this->motorL_, &this->motorR_});
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Command written for y position: %f",
                        vertical_pos);
            set_state(vertical_name_pos, vertical_pos);
        }
    }
}

hardware_interface::return_type MasonInterface::write(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration & /*period*/) {
    if (!this->serial->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Serial port is not open");
        return hardware_interface::return_type::ERROR;
    }

    if (!check_joints()) {
        return hardware_interface::return_type::ERROR;
    }

    std::string horizontal_name_pos = motorH_.joint_name + "/" + hardware_interface::HW_IF_POSITION;
    std::string left_name_pos = motorL_.joint_name + "/" + hardware_interface::HW_IF_POSITION;

    double horizontal_cmd = get_command(horizontal_name_pos);
    double left_cmd = get_command(left_name_pos);

    bool skipHorizontal = std::isnan(horizontal_cmd) ||
                          std::abs(this->prev_horizontal_position_ - horizontal_cmd) <= 1;
    bool skipVertical =
        std::isnan(left_cmd) || std::abs(this->prev_vertical_position_ - left_cmd) <= 1;

    if (!skipHorizontal) this->prev_horizontal_position_ = horizontal_cmd;
    if (!skipVertical) this->prev_vertical_position_ = left_cmd;

    if (skipHorizontal && skipVertical) {
        return hardware_interface::return_type::OK;
    }

    this->write_queue_.push({Command{horizontal_cmd, horizontal_name_pos, skipHorizontal},
                             Command{left_cmd, left_name_pos, skipVertical}});

    // if (write_thread_busy) {
    //     this->write_thread_.join();
    // }
    // this->write_thread_ = std::thread(&MasonInterface::updatePos, this,
    //                                     Command{horizontal_cmd, horizontal_name_pos,
    //                                     skipHorizontal}, Command{left_cmd, left_name_pos,
    //                                     skipVertical});

    return hardware_interface::return_type::OK;
}

bool MasonInterface::check_joints() {
    if (info_.joints.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Number of joints is not 2");
        return false;
    }

    if (info_.joints[0].name != motorH_.joint_name || info_.joints[1].name != motorL_.joint_name) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Joint names are not correct");
        return false;
    }

    return true;
}

}  // namespace mason_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mason_hardware::MasonInterface, hardware_interface::SystemInterface)