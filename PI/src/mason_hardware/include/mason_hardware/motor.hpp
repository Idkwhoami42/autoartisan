#ifndef MOTOR_H
#define MOTOR_H

#endif //MOTOR_H

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "commands.hpp"
#include <utility>
#include <boost/asio.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <thread>

class Motor;

class Package {
    public:
        int startByte;
        int payloadLength;
        std::vector<uint8_t> payload;
        std::vector<uint8_t> crc;

        Package();


        void getFromSerial(boost::asio::serial_port* serial);

        std::pair<bool, std::vector<uint8_t>> getFromBuffer(std::vector<uint8_t> buffer);

        void printContents();

        void encodeCommand(Commands command, std::vector<int> fields);

        static std::vector<uint8_t> packEndian(int field);

        void send(boost::asio::serial_port* serial);

        int16_t unpackH(int startIndex, int endIndex) const;

        int32_t unpackI(int startIndex, int endIndex) const;

        int unpackB(int index) const;

        void decode(std::vector<Motor*> motors) const;
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
        double temp_fet;
        double temp_motor;
        double avg_motor_current;
        double avg_in_current;
        double avg_id;
        double avg_iq;
        double duty_now;
        double rpm;
        double v_in;
        double amp_hours;
        double amp_hours_charged;
        double watt_hours;
        double watt_hours_charged;
        int tachometer;
        int tachometer_abs;
        int mc_fault_code;
        double pid_pos_now;
        double actual_pos;
        double last_clamped_pos;
        double clamped_pos;
        double pos;
        double vel;
        double cmd;
        double temp_mos1;
        double temp_mos2;
        double temp_mos3;
        double avg_vd;
        double avg_vq;
        int status;
        double test_pos;

        Package pos_package;

        // Constructor
        Motor(std::string joint_name, int id, bool is_can = false);

        // Destructor
        // ~Motor();

        void printValues() const;

        bool getValues(boost::asio::serial_port* serial);

        void setDuty(boost::asio::serial_port* serial, double duty) const;

        void setPos(boost::asio::serial_port* serial, double pos);

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

        std::vector<std::pair<double, double>> goToPos(boost::asio::serial_port* serial, double pos, std::vector<Motor*> motors, double degrees_per_step=0.1, int acc_substeps=10, int velocity=40, int velocity_rampsteps=-1);

        void goToPosXY(boost::asio::serial_port *serial, std::pair<double, double> pos, Motor *motorHorizontal,
                       std::vector<Motor *> motorsVertical, double degrees_per_step = 0.1,
                       int acc_substeps=10, int velocity = 40, int velocity_rampsteps = -1);
};



#endif // MOTOR_HPP
