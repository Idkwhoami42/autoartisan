#ifndef MASON_HARDWARE__MASON_INTERFACE_HPP_
#define MASON_HARDWARE__MASON_INTERFACE_HPP_

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mason_hardware/contact_sensors.hpp"
#include "mason_hardware/motor.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mason_hardware {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MasonInterface : public hardware_interface::SystemInterface {
    struct Config {
        std::string device = "";
        boost::asio::serial_port *port;
        double duty_conversion_factor_ = 0.0907;
        double max_velocity_ = 11.025;
    };

   public:
    MasonInterface();
    virtual ~MasonInterface();

    virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    void homingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void returnToStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void initializeServices(rclcpp::Node::SharedPtr node);

   private:
    Config cfg_;
    Motor *motorL_ = nullptr;
    Motor *motorR_ = nullptr;
    Motor *motorH_ = nullptr;
    ContactSensors *contactSensors = nullptr;
    Controller *controller_ = nullptr;

    std::vector<double> position_commands_;
    std::vector<double> position_states_;

    double max_x_ = -1;
    double max_y_ = -1;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};
};  // namespace mason_hardware

#endif  // MASON_HARDWARE__MASON_INTERFACE_HPP_