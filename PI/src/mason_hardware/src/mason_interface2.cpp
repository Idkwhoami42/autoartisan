#include "mason_hardware/mason_interface2.hpp"

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

FSMPublisher::FSMPublisher() : Node("mason_fsm_publisher") {
    this->state_publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/mason_fsm_publisher/state", 10);
    this->state_publisher2_ =
        this->create_publisher<std_msgs::msg::Int32>("/mason_fsm_publisher/state2", 10);
}

void FSMPublisher::publishState(const int state, Teensy id) {
    auto message = std_msgs::msg::Int32();
    message.data = state;

    if (id == TEENSY1)
        this->state_publisher_->publish(message);
    else if (id == TEENSY2) {
        this->state_publisher2_->publish(message);
    }
}

MasonInterface::MasonInterface() : /*serial(nullptr), controller_(nullptr),*/ contactSensors(nullptr){};

MasonInterface::~MasonInterface() {
    // if (this->serial->is_open()) {
    //     this->serial->close();
    // }

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

// TODO: Set up homed reference frame: https://docs.odriverobotics.com/v/latest/manual/control.html#homed-reference-frame
bool MasonInterface::homingCallback() {
    // NOT SURE HOW TO IMPLEMENT ACTUAL HOMING WITH ODRIVES
    for (auto& axis : axes_) {
        // <axis>.controller.config.absolute_setpoints = true;
        Set_Axis_State_msg_t homing_msg;
        homing_msg.Axis_Requested_State = AXIS_STATE_HOMING;

        axis.send(homing_msg);
    }

    bool homed_horizontally = false;
    bool homed_vertically = false;
    while (!homed_vertically) {
        std::string sensor = "";
        {
            std::unique_lock<std::mutex> lock(activeSensorMutex);
            sensor = activeSensor;
        }
        if (sensor == "TOP") {
            homed_vertically = true;
            axes_[1].set_velocity(0.0f);
            axes_[2].set_velocity(0.0f);
        } else {
            axes_[1].set_velocity(10.0f);
            axes_[2].set_velocity(10.0f);
        }
    }

    axes_[1].get_position();
    axes_[2].get_position();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (std::abs((axes_[1].pos_estimate_ - axes_[2].pos_estimate_)) > 1.0f) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"),
            "Error occurred during homing: Left and right vertical motors seem misaligned.");
        return false;
    }

    this->max_y_ = std::min(axes_[1].pos_estimate_, axes_[2].pos_estimate_);

    while (!homed_horizontally) {
        std::string sensor = "";
        {
            std::unique_lock<std::mutex> lock(activeSensorMutex);
            sensor = activeSensor;
        }
        if (sensor == "RIGHT") {
            homed_horizontally = true;
            axes_[0].set_velocity(0.0f);
        } else {
            axes_[0].set_velocity(10.0f);
        }
    }

    axes_[0].get_position();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    this->max_x_ = axes_[0].pos_estimate_;

    initializePublisher();

    this->executor_->spin_node_once(this->homing_pub_);
    this->homing_pub_->publishLimits(this->max_x_, this->max_y_, true);

    return true;
}

bool MasonInterface::returnToStart() {
    // NOT SURE HOW TO IMPLEMENT ACTUAL RETURN-TO-START FUNCTION WITH ODRIVES
    while (true) {
        std::string sensor = "";
        {
            std::unique_lock<std::mutex> lock(activeSensorMutex);
            sensor = activeSensor;
        }
        if (sensor == "BOTTOM") {
            axes_[1].set_velocity(0.0f);
            axes_[2].set_velocity(0.0f);
            break;
        } else {
            axes_[1].set_velocity(-10.0f);
            axes_[2].set_velocity(-10.0f);
        }
    }

    axes_[1].get_position();
    axes_[2].get_position();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (std::abs((axes_[1].pos_estimate_ - axes_[2].pos_estimate_)) > 1.0f) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"),
            "Error occurred during homing: Left and right vertical motors seem misaligned.");
        return false;
    }

    this->min_y_ = std::max(axes_[1].pos_estimate_, axes_[2].pos_estimate_);

    while (true) {
        std::string sensor = "";
        {
            std::unique_lock<std::mutex> lock(activeSensorMutex);
            sensor = activeSensor;
        }
        if (sensor == "LEFT") {
            axes_[0].set_velocity(0.0f);
            break;
        } else {
            axes_[0].set_velocity(-10.0f);
        }
    }

    axes_[0].get_position();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    this->min_x_ = axes_[0].pos_estimate_;
    this->at_starting_point_ = true;
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

    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    // std::string device = info_.hardware_parameters["device"];
    // std::string left_motor_name = info_.hardware_parameters["left_motor_name"];
    // std::string right_motor_name = info_.hardware_parameters["right_motor_name"];
    // std::string horizontal_motor_name = info_.hardware_parameters["horizontal_motor_name"];
    // int left_motor_id = std::stoi(info_.hardware_parameters["id_left"]);
    // int right_motor_id = std::stoi(info_.hardware_parameters["id_right"]);
    // int horizontal_motor_id = std::stoi(info_.hardware_parameters["id_horizontal"]);

    // RCLCPP_INFO(rclcpp::get_logger("MasonInterface"),
    //             "PARAM - LEFT_MOTOR_NAME: %s\n"
    //             "PARAM - RIGHT_MOTOR_NAME: %s\n"
    //             "PARAM - HORIZONTAL_MOTOR_NAME: %s\n"
    //             "PARAM - LEFT_MOTOR_ID: %d\n"
    //             "PARAM - RIGHT_MOTOR_ID: %d\n"
    //             "PARAM - HORIZONTAL_MOTOR_ID: %d\n"
    //             "PARAM - DEVICE: %s\n",
    //             left_motor_name.c_str(), right_motor_name.c_str(), horizontal_motor_name.c_str(),
    //             left_motor_id, right_motor_id, horizontal_motor_id, device.c_str());

    std::vector<std::pair<unsigned int, std::string>> contactSensorPins = {
        {27, "RIGHT"}, {22, "TOP"}, {23, "BOTTOM"}, {24, "LEFT"}};

    this->contactSensors.reset(new ContactSensors(chip, contactSensorPins));

    try {
        // this->serial.reset(new boost::asio::serial_port(this->io, device));
        // this->serial->set_option(boost::asio::serial_port_base::baud_rate(115200));
        // this->serial->set_option(boost::asio::serial_port_base::character_size(8));
        // this->serial->set_option(boost::asio::serial_port_base::stop_bits(
        //     boost::asio::serial_port_base::stop_bits::one));
        // this->serial->set_option(
        //     boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        // this->serial->set_option(boost::asio::serial_port_base::flow_control(
        //     boost::asio::serial_port_base::flow_control::none));

        // this->motorL_ = Motor(left_motor_name, left_motor_id, serial.get(), true);
        // this->motorR_ = Motor(right_motor_name, right_motor_id, serial.get(), true);
        // this->motorH_ = Motor(horizontal_motor_name, horizontal_motor_id, serial.get(), true);

        // this->controller_.reset(new Controller(
        //     this->serial.get(), {left_motor_id, right_motor_id, horizontal_motor_id}));


    } catch (std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Error occurred: %s.", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->homing_pub_ = std::make_shared<HomingPublisher>();

    this->fsm_pub_ = std::make_shared<FSMPublisher>();

    this->polling_thread_ = std::thread(pollPins, this->contactSensors.get());

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Hardware interface initialized.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_configure(const rclcpp_lifecycle::State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&MasonInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MasonInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_cleanup(const rclcpp_lifecycle::State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Activating...please wait...");

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Returning to start...please wait...");

    if (!this->returnToStart()) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Return to start failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!this->homingCallback()) {
        RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Homing failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->write_thread_ = std::thread(&MasonInterface::writeWorker, this);

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasonInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Deactivating ...please wait...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MasonInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MasonInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    return command_interfaces;
}

hardware_interface::return_type MasonInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}};

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MasonInterface::read(const rclcpp::Time & time,
                                                     const rclcpp::Duration & /*period*/) {
    timestamp_ = time;

    // if (!check_joints()) {
    //     return hardware_interface::return_type::ERROR;
    // }

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return hardware_interface::return_type::OK;
}

void MasonInterface::writeWorker() {
    while (true) {
        if (this->write_queue_.empty()) continue;

        auto &[msg, pos, name_pos, skip, index] = this->write_queue_.front();
        this->write_queue_.pop();

        if (!skip) {
            axes_[index].send(msg);
            RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Command written for %s: %f",
                        name_pos.c_str(),
                        pos);
        }
    }
}

hardware_interface::return_type MasonInterface::write(const rclcpp::Time & /*time*/,
                                                      const rclcpp::Duration & /*period*/) {

    // if (!check_joints()) {
    //     return hardware_interface::return_type::ERROR;
    // }

    for (size_t i = 0; i < axes_.size(); i++) {
        // Send the CAN message that fits the set of enabled setpoints
        auto& axis = axes_[i];
        if (axis.pos_input_enabled_) {
            std::string name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            // UNCOMMENT FOR SENDING COMMAND BY PROPORTION OF FRAME
            // double pos_cmd = (i == 0) ? (this->min_x_ + axis.pos_setpoint_*(this->max_x_ - this->min_x_)) : (this->min_y_ + axis.pos_setpoint_*(this->max_y_ - this->min_y_));
            Set_Input_Pos_msg_t msg;
            // msg.Input_Pos = pos_cmd / (2 * M_PI);
            msg.Input_Pos = axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_ / (2 * M_PI)) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            bool skip = std::isnan(axis.pos_setpoint_) ||
                          std::abs(axes_[i].pos_estimate_ - axis.pos_setpoint_) <= 1;
            this->write_queue_.push(Command{msg, axis.pos_setpoint_, name_pos, skip, static_cast<int>(i)});
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return hardware_interface::return_type::OK;
}

// bool MasonInterface::check_joints() {
//     if (info_.joints.size() != 2) {
//         RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Number of joints is not 2");
//         return false;
//     }

//     if (info_.joints[0].name != motorH_.joint_name || info_.joints[1].name != motorL_.joint_name) {
//         RCLCPP_ERROR(rclcpp::get_logger("MasonInterface"), "Joint names are not correct");
//         return false;
//     }

//     return true;
// }

void MasonInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void MasonInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t clear_error_msg;
    Set_Axis_State_msg_t state_msg;

    clear_error_msg.Identify = 0;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("MasonInterface"), "No control mode specified. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(state_msg);
        return;
    }

    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("MasonInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

// Added method: sends a CAN message requesting position
// ODrive will respond, triggering on_can_msg, which updates pos_estimate_
void Axis::get_position() {
    Get_Encoder_Estimates_msg_t msg;
    this->send(msg);
}

// Added method: sets velocity until contact sensor is reached
void Axis::set_velocity(double velocity) {
    Set_Input_Vel_msg_t msg;
    msg.Input_Vel = velocity / (2 * M_PI);
    this->send(msg);
}

}  // namespace mason_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mason_hardware::MasonInterface, hardware_interface::SystemInterface)