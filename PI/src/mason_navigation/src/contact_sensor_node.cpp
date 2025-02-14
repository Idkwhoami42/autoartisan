#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <atomic>
#include <gpiod.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define RIGHT_CONTACT_PIN 27
#define TOP_CONTACT_PIN 22
#define BOTTOM_CONTACT_PIN 23
#define LEFT_CONTACT_PIN 24

class ContactSensorNode : public rclcpp::Node {
   public:
    ContactSensorNode() : Node("contact_sensor_node") {
        const char *chip_name = "gpiochip4";
        std::vector<std::pair<unsigned int, std::string>> pinNumbers = {
            {RIGHT_CONTACT_PIN, "right"},
            {TOP_CONTACT_PIN, "top"},
            {BOTTOM_CONTACT_PIN, "bottom"},
            {LEFT_CONTACT_PIN, "left"}};

        this->chip.open(std::string(chip_name), 3);
        if (!chip) {
            RCLCPP_ERROR(this->get_logger(), "Error opening chip");
            return;
        }

        this->getPins(pinNumbers);
        RCLCPP_INFO(this->get_logger(), "Got GPIO offsets from pair vector");
        this->bulk = this->chip.get_lines(pins);

        if (!bulk) {
            this->chip.reset();
            RCLCPP_ERROR(this->get_logger(), "Error getting lines from bulk");
            return;
        }

        // ret = gpiod_line_request_bulk_both_edges_events(&bulk, "gpio-monitor");
        this->bulk.request({"", gpiod::line_request::EVENT_BOTH_EDGES,
                            /*gpiod::line_request::FLAG_BIAS_PULL_UP*/});

        for (const auto &pinNumber : pinNumbers) {
            this->pinMap[static_cast<int>(pinNumber.first)] = pinNumber.second;
        }

        this->sensor_publisher_ =
            this->create_publisher<std_msgs::msg::String>("/contact_sensor_triggered", 10);
    }

    void buttonCallback(unsigned int pin, gpiod::line_event event) {
        auto message = std_msgs::msg::String();
        int index = static_cast<int>(pin);
        if (event.event_type == gpiod::line_event::FALLING_EDGE) {
            message.data = this->pinMap[index];
            this->sensor_publisher_->publish(message);
        }
    }

    ~ContactSensorNode() {
        if (this->keepPolling.load()) {
            RCLCPP_INFO(this->get_logger(), "Entered destructor");
            this->keepPolling.store(false);
            // bulk.release();
            this->bulk.~line_bulk();
            this->chip.~chip();
        }
    }

    gpiod::line_bulk bulk;
    gpiod::line_bulk event_lines;
    std::atomic<bool> keepPolling{true};

   private:
    void getPins(std::vector<std::pair<unsigned int, std::string>> pinNumbers) {
        for (auto &pin : pinNumbers) {
            pins.push_back(pin.first);
        }
    }

    std::unordered_map<int, std::string> pinMap;
    gpiod::chip chip;
    std::vector<unsigned int> pins;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_publisher_;
};

void pollPins(ContactSensorNode *contactSensors) {
    while (contactSensors->keepPolling.load()) {
        if (contactSensors) {
            contactSensors->event_lines = contactSensors->bulk.event_wait(10us);
            if (!contactSensors->event_lines.empty()) {
                for (const auto &event : contactSensors->event_lines) {
                    // std::unique_lock<std::mutex> lock(activeSensorMutex);
                    // activeSensor =
                    contactSensors->buttonCallback(event.offset(), event.event_read());
                }
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<ContactSensorNode> contact_sensor_node = std::make_shared<ContactSensorNode>();
    std::thread polling_thread = std::thread(pollPins, contact_sensor_node.get());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Contact sensor node initialized");

    rclcpp::spin(contact_sensor_node);
    rclcpp::on_shutdown([contact_sensor_node]() { contact_sensor_node->~ContactSensorNode(); });

    if (polling_thread.joinable()) polling_thread.join();

    return 0;
}