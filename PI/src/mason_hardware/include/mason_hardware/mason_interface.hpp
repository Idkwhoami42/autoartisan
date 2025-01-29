#ifndef MASON_HARDWARE__MASON_INTERFACE_HPP_
#define MASON_HARDWARE__MASON_INTERFACE_HPP_

#include <boost/asio.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <queue>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>

#include "contact_sensors.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motor.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mason_hardware {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Declared separate class for homing publisher bc it only needs to spin once at the beginning
class HomingPublisher : public rclcpp::Node {
   public:
    HomingPublisher();
    void publishLimits(const double x_limit, const double y_limit, const bool success);

   private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr rotation_lim_publisher_;
};

enum Teensy {
    TEENSY1 = 1,
    TEENSY2 = 2,
};

class FSMPublisher : public rclcpp::Node {
   public:
    FSMPublisher();
    void publishState(const int state, Teensy id);

   private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher2_;
};

struct Command {
    double pos;
    std::string command_name;
    bool skip;
};

class MasonInterface : public hardware_interface::SystemInterface {
    struct Config {
        double duty_conversion_factor_ = 0.0907;
        double max_velocity_ = 11.025;
    };

   public:
    MasonInterface();
    virtual ~MasonInterface();

    virtual hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    virtual hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    virtual hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    virtual hardware_interface::return_type read(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time &time,
                                                  const rclcpp::Duration &period) override;

    bool home();
    bool homingCallback();
    bool returnToStart();
    void initializePublisher();
    bool check_joints();
    void writeWorker();

   private:
    Config cfg_;
    boost::asio::io_context io;
    std::unique_ptr<boost::asio::serial_port> serial;
    Motor motorL_;
    Motor motorR_;
    Motor motorH_;
    std::unique_ptr<Controller> controller_;
    std::unique_ptr<ContactSensors> contactSensors;

    double prev_horizontal_position_ = 0;
    double prev_vertical_position_ = 0;

    double max_x_ = 1000;
    double max_y_ = 1000;
    bool at_starting_point_ = false;

    std::thread write_thread_;
    std::thread polling_thread_;

    std::queue<std::pair<Command, Command>> write_queue_;

    std::shared_ptr<HomingPublisher> homing_pub_;
    std::shared_ptr<FSMPublisher> fsm_pub_;
    rclcpp::Executor::SharedPtr executor_;
};
};  // namespace mason_hardware

#endif  // MASON_HARDWARE__MASON_INTERFACE_HPP_