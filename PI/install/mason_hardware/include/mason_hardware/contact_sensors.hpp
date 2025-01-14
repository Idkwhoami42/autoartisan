#ifndef CONTACT_SENSOR_HPP
#define CONTACT_SENSOR_HPP

#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>

class ContactSensors {
public:

    void buttonCallback(int pin);

    // void bottomLeftCallback();

    // void leftCallback();

    // void topLeftCallback();

    // void bottomRightCallback();

    // void rightCallback();

    // void topRightCallback();

    explicit ContactSensors(std::vector<std::pair<int, std::string>> pinNumbers);

    ~ContactSensors();

    std::string mostRecentlyActivated();

private:
    std::unordered_map<int, std::string> pinMap;
    std::vector<std::string> activeSensors = {};
};

#endif // CONTACT_SENSOR_HPP