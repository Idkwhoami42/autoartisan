#include <iostream>

#include "motor.h"

std::thread pos_thread;

int main() {
    std::cout << "Starting program...\n";

    boost::asio::io_context io;
    boost::asio::serial_port serial(io, "/dev/ttyACM0");  // serialport is a string like "COM3" or "/dev/ttyUSB0"

    // Configure port settings
    serial.set_option(boost::asio::serial_port_base::baud_rate(921600));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    Motor right_motor("right_motor", 72, &serial, true);
    Motor left_motor("left_motor", 125, &serial, true);
    Motor horizontal_motor("horizontal_motor", 1, &serial, true);
    Controller controller(&serial, {1, 72, 125});

    std::vector vertical_motors = { &right_motor, &left_motor };
    std::vector horizontal_motors = { &horizontal_motor };

    std::cout << "Going to position." << std::endl;

    for (Motor* motor : { &right_motor, &left_motor, &horizontal_motor }) {
     motor->doHoming(&serial);
     if (!motor->verifyHoming()) {
         std::cout << "Homing went wrong for motor " << motor->id << std::endl;
         return 0;
     }
     std::cout << "Homing successful for motor " << motor->id << std::endl;
    }

    controller.goToPos(&serial, 195, vertical_motors, 0.1 ,10, 60);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    controller.goToPos(&serial, 200, vertical_motors, 0.1 ,10, 60);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    controller.goToPos(&serial, 30, vertical_motors, 0.1 ,10, 30);

    //
    // left_motor.setDuty(&serial, 0.03);
    // right_motor.setDuty(&serial, 0.03);
    //
    // int k = 80;
    // while (k--)
    // {
    //     // left_motor.getValues(&serial);
    //     // right_motor.getValues(&serial);
    //     std::cout << left_motor.pos << " " << right_motor.pos << std::endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //
    // left_motor.setDuty(&serial, 0);
    // right_motor.setDuty(&serial, 0);
    //
    // // left_motor.getValues(&serial);
    // // right_motor.getValues(&serial);
    // std::cout << left_motor.pos << " " << right_motor.pos << std::endl;
    //
    // controller.goToPos(&serial, 20, vertical_motors, 0.1 ,10, 60);


    // controller.goToPos(&serial, 180, vertical_motors, 0.1 ,10, 60);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // controller.goToPos(&serial, 20, vertical_motors, 0.025 ,10, 20);

    // controller.goToPos(&serial, 90, vertical_motors, 0.01, 1, 60);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // controller.goToPos(&serial, 30, vertical_motors, 0.001, 1, 20);

    // std::vector<uint8_t> b(6);
    // right_motor.setDerivativeGain(serial, 0.001);
    // boost::asio::read(serial, boost::asio::buffer(b, 6));
    // left_motor.setDerivativeGain(serial, 0.001);
    // boost::asio::read(serial, boost::asio::buffer(b, 6));
    //
    // controller.goToPos(&serial, 180, vertical_motors, 0.1 ,10, 60);
    //
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    // right_motor.getValues(&serial);
    // std::cout << right_motor.pos << std::endl;
    // right_motor.setPos(&serial, 180);
    // left_motor.setPos(&serial, 180);
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    // controller.goToPos(&serial, 20, vertical_motors, 0.025 ,10, 20);
    // right_motor.setDerivativeGain(serial, 0.000);
    // left_motor.setDerivativeGain(serial, 0.000);

    // controller.goToPos(&serial, 360, vertical_motors, 0.1, 10, 60);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // controller.goToPos(&serial, 180, vertical_motors, 0.025, 10, 20);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // controller.goToPos(&serial, 50, vertical_motors, 0.025, 10, 20);
    // std::cout << "Going to new position" << std::endl;
    // controller.goToPos(&serial, 0, horizontal_motors, 0.1, 10, 60);

    std::cout << "Stopping...\n";

    std::this_thread::sleep_for(std::chrono::seconds(10000));

    return 0;
}

