#ifndef MOTOR_H
#define MOTOR_H

#include <boost/asio.hpp>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "commands.h"

class Motor;

class Package {
   public:
    int startByte;
    int payloadLength;
    std::vector<uint8_t> payload;
    std::vector<uint8_t> crc;

    Package();

    void encodeCommand(Commands command, std::vector<int> fields);

    static std::vector<uint8_t> packEndian(int field);

    void send(boost::asio::serial_port* serial);
};

class Motor {
   public:
    std::string joint_name;
    int id;
    bool is_can;
    double offset;
    bool homed;
    int home_cooldown;
    double rotations;
    double pid_pos_now;
    double actual_pos;
    double last_clamped_pos;
    double clamped_pos;
    double pos;
    double vel;
    double cmd;

    Package pos_package;

    // Constructor
    Motor(std::string joint_name, int id, boost::asio::serial_port* serial, bool is_can = false);
    Motor() = default;
    // Destructor
    // ~Motor();

    static int32_t unpackI(const std::vector<uint8_t>& buffer, int startIndex, int endIndex);

    bool getValues(boost::asio::serial_port* serial);

    void setDuty(boost::asio::serial_port* serial, double duty) const;

    void setPos(boost::asio::serial_port* serial, double pos);

    void setDerivativeGain(boost::asio::serial_port& serial, float gain);

    uint32_t buffer_append_float32_auto(float number);

    void doHoming(boost::asio::serial_port* serial);

    bool verifyHoming() const;
};

class Controller {
   public:
    std::vector<int> can_ids;
    std::atomic<bool> shutdown;
    std::thread heart_beat_thread;

    Controller(boost::asio::serial_port* serialport, std::vector<int> can_ids);
    ~Controller();

    void heartbeatFunc(boost::asio::serial_port* serialport);

    void goToPos(boost::asio::serial_port* serial, double pos, std::vector<Motor*> motors,
                 double degrees_per_step = 0.1, int acc_substeps = 10, int velocity = 40,
                 int velocity_rampsteps = -1);

    void goToPosXY(boost::asio::serial_port* serial, double pos_x, double pos_y,
                   Motor* horizontal_motor, std::vector<Motor*> vertical_motors,
                   double degrees_per_step = 0.1, int acc_substeps = 10, int velocity = 40,
                   int velocity_rampsteps = -1);
};

#endif  // MOTOR_HPP