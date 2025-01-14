#include "mason_hardware/contact_sensors.hpp"

#include <algorithm>
#include <iostream>

ContactSensors* globalContactSensors = nullptr;

void ContactSensors::buttonCallback(int pin) {
    // bool state = digitalRead(pin);
    // auto it = activeSensors.begin();
    // state ? activeSensors.insert(it, pinMap[pin])
    //       : activeSensors.erase(std::remove(it, activeSensors.end(), pinMap[pin]));
}

// void ContactSensors::bottomLeftCallback() {
//     // TODO: FIND ACTUAL PIN NUMBER
//     buttonCallback(0);
// }

void globalBottomLeftInterruptHandler() {
    if (globalContactSensors) {
        globalContactSensors->buttonCallback(0);
    } else {
        std::cerr << "Interrupt triggered, but no instance is set!" << std::endl;
    }
}

void globalLeftInterruptHandler() {
    if (globalContactSensors) {
        globalContactSensors->buttonCallback(0);
    } else {
        std::cerr << "Interrupt triggered, but no instance is set!" << std::endl;
    }
}

void globalTopLeftInterruptHandler() {
    if (globalContactSensors) {
        globalContactSensors->buttonCallback(0);
    } else {
        std::cerr << "Interrupt triggered, but no instance is set!" << std::endl;
    }
}

void globalTopRightInterruptHandler() {
    if (globalContactSensors) {
        globalContactSensors->buttonCallback(0);
    } else {
        std::cerr << "Interrupt triggered, but no instance is set!" << std::endl;
    }
}

void globalRightInterruptHandler() {
    if (globalContactSensors) {
        globalContactSensors->buttonCallback(0);
    } else {
        std::cerr << "Interrupt triggered, but no instance is set!" << std::endl;
    }
}

void globalBottomRightInterruptHandler() {
    if (globalContactSensors) {
        globalContactSensors->buttonCallback(0);
    } else {
        std::cerr << "Interrupt triggered, but no instance is set!" << std::endl;
    }
}

// void ContactSensors::leftCallback() {
//     // TODO: FIND ACTUAL PIN NUMBER
//     buttonCallback(0);
// }

// void ContactSensors::topLeftCallback() {
//     // TODO: FIND ACTUAL PIN NUMBER
//     buttonCallback(0);
// }

// void ContactSensors::bottomRightCallback() {
//     // TODO: FIND ACTUAL PIN NUMBER
//     buttonCallback(0);
// }

// void ContactSensors::rightCallback() {
//     // TODO: FIND ACTUAL PIN NUMBER
//     buttonCallback(0);
// }

// void ContactSensors::topRightCallback() {
//     // TODO: FIND ACTUAL PIN NUMBER
//     buttonCallback(0);
// }

ContactSensors::ContactSensors(std::vector<std::pair<int, std::string>> pinNumbers) {
    // wiringPiSetup();
    // std::vector<void (*)(void)> functions = {&globalBottomLeftInterruptHandler, &globalLeftInterruptHandler,
    //                                          &globalTopLeftInterruptHandler,    &globalTopRightInterruptHandler,
    //                                          &globalRightInterruptHandler,      &globalBottomRightInterruptHandler};
    // for (size_t i = 0; i < pinNumbers.size(); i++) {
    //     this->pinMap[pinNumbers[i].first] = pinNumbers[i].second;
    //     pinMode(pinNumbers[i].first, INPUT);
    //     wiringPiISR(pinNumbers[i].first, INT_EDGE_BOTH, functions[i]);
    // }

    // ContactSensors* globalContactSensors = this;
    // // for (size_t i = 0; i < pinNumbers.size(); i++) {
    //     wiringPiISR(pinNumbers[i].first, INT_EDGE_BOTH, functions[i]);
    // }
}

ContactSensors::~ContactSensors() = default;

std::string ContactSensors::mostRecentlyActivated() {
    // noInterrupts();
    return (activeSensors.size() > 0) ? activeSensors[0] : "";
    // interrupts();
}