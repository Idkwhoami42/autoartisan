#ifndef CONTACT_SENSOR_HPP
#define CONTACT_SENSOR_HPP

#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <gpiod.hpp>
#include <atomic>

class ContactSensors {
public:

    std::string buttonCallback(unsigned int pin, gpiod::line_event event);

    explicit ContactSensors(const char* chip_name, const std::vector<std::pair<unsigned int, std::string>> &pinNumbers);

    ~ContactSensors();

    gpiod::line_bulk bulk;
    gpiod::line_bulk event_lines;
    std::atomic<bool> keepPolling{true};
    
private:
    std::unordered_map<int, std::string> pinMap;
    std::vector<std::string> activeSensors = {};
    gpiod::chip chip;
};

#endif // CONTACT_SENSOR_HPP