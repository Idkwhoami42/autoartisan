#include <algorithm>
#include <atomic>
#include <iostream>
#include <gpiod.hpp>
#include <mutex>
#include <stdio.h>
#include <string.h>
#include "../include/mason_hardware/contact_sensors.hpp"
#include <thread>

#define TIMEOUT_NS 1000000000

std::vector<unsigned int> pins;

std::string ContactSensors::buttonCallback(unsigned int pin, gpiod::line_event event) {
    int index = static_cast<int>(pin);
    if (event.event_type == gpiod::line_event::FALLING_EDGE) {
        return this->pinMap[index];
    } else {
        return "";
    }
}

void getPins(std::vector<std::pair<unsigned int, std::string>> pinNumbers) {
    for (auto &pin : pinNumbers) {
        pins.push_back(pin.first);
    }
}

ContactSensors::ContactSensors(const char* chip_name, const std::vector<std::pair<unsigned int, std::string>> &pinNumbers) {
    this->chip.open(std::string(chip_name), 3);
    if (!chip) {
        perror("Error initializing chip");
    }

    getPins(pinNumbers);
    std::cout << "Got GPIO offsets from pair vector" << std::endl;
    this->bulk = this->chip.get_lines(pins);

    if (!bulk) {
        this->chip.reset();
        std::cerr << "Error storing lines in bulk" << std::endl;
    }

    // ret = gpiod_line_request_bulk_both_edges_events(&bulk, "gpio-monitor");
    this->bulk.request({"", gpiod::line_request::EVENT_BOTH_EDGES, /*gpiod::line_request::FLAG_BIAS_PULL_UP*/});

    for (const auto & pinNumber : pinNumbers) {
        this->pinMap[static_cast<int>(pinNumber.first)] = pinNumber.second;
    }
}

ContactSensors::~ContactSensors() {
    if (this->keepPolling.load()) {
        std::cout << "Entered destructor" << std::endl;
        this->keepPolling.store(false);
        // bulk.release();
        this->bulk.~line_bulk();
        this->chip.~chip();
    }
}