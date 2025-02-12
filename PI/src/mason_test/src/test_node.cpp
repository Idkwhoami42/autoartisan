#include <chrono>

#include "odrive_base/include/odrive_base/"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"

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
    }

   private:
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client0;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client1;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_state_client2;

    void set_axis_state(rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client, int axis, ) {
        while (!client->wait_for_service(500ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for the service for axis %d. Exiting.",
                             axis);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service for axis %d not available, waiting again...",
                        axis);
        }

        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state =
            static_cast<int32_t>(odrive_base::enums::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}