#include "../include/mason_hardware/motor.hpp"
#include <boost/endian/conversion.hpp>
#include <boost/endian/arithmetic.hpp>
#include <boost/crc.hpp>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>
#include <thread>
#include <random>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include "../include/mason_hardware/crc.h"

std::mutex mtx;

// Constructor
Motor::Motor(std::string joint_name, const int id, const bool is_can) :
    joint_name(std::move(joint_name)),
    id(id),
    is_can(is_can),
    offset(0),
    homed(false),
    home_cooldown(0),
    rotations(0),
    temp_fet(-1),
    temp_motor(-1),
    avg_motor_current(-1),
    avg_in_current(-1),
    avg_id(-1),
    avg_iq(-1),
    duty_now(-1),
    rpm(-1),
    v_in(-1),
    amp_hours(-1),
    amp_hours_charged(-1),
    watt_hours(-1),
    watt_hours_charged(-1),
    tachometer(-1),
    tachometer_abs(-1),
    mc_fault_code(-1),
    pid_pos_now(-1),
    actual_pos(-1),
    last_clamped_pos(-1),
    clamped_pos(-1),
    pos(-1),
    vel(0),
    cmd(0),
    temp_mos1(-1),
    temp_mos2(-1),
    temp_mos3(-1),
    avg_vd(-1),
    avg_vq(-1),
    status(-1),
    test_pos(-1)
{
    pos_package = Package();
    if (this->is_can) {
        const std::vector data = {this->id, static_cast<int>(COMM_SET_POS), 0};
        pos_package.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector data = {0};
        pos_package.encodeCommand(COMM_SET_POS, data);
    }
}

void Motor::printValues() const {
    printf("temp_fet: %f \n", this->temp_fet);
    printf("temp_motor: %f \n", this->temp_motor);
    printf("avg_motor_current: %f \n", this->avg_motor_current);
    printf("avg_current_in: %f \n", this->avg_in_current);
    printf("avg_id: %f \n", this->avg_id);
    printf("avg_iq: %f \n", this->avg_iq);
    printf("duty_now: %f \n", this->duty_now);
    printf("rpm: %f \n", this->rpm);
    printf("v_in: %f \n", this->v_in);
    printf("amp_hours: %f \n", this->amp_hours);
    printf("amp_hours_charged: %f \n", this->amp_hours_charged);
    printf("watt_hours: %f \n", this->watt_hours);
    printf("watt_hours_charged: %f \n", this->watt_hours_charged);
    printf("tachometer: %d \n", this->tachometer);
    printf("tachometer_abs: %d \n", this->tachometer_abs);
    printf("mc_fault_code: %d \n", this->mc_fault_code);
    printf("pid_pos_now: %f \n", this->pid_pos_now);
    printf("temp_mos1: %f \n", this->temp_mos1);
    printf("temp_mos2: %f \n", this->temp_mos2);
    printf("temp_mos3: %f \n", this->temp_mos3);
    printf("avg_vd: %f \n", this->avg_vd);
    printf("avg_vq: %f \n", this->avg_vq);
    printf("status: %d \n", this->status);
}

bool Motor::getValues(boost::asio::serial_port* serial) {
    Package p;
    if (this->is_can) {
        const std::vector<int> data = {this->id, COMM_SET_DETECT, 0};
        p.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector<int> data = {0};
        p.encodeCommand(COMM_SET_DETECT, data);
    }
    p.send(serial);

    Package p2;
    if (this->is_can) {
        const std::vector<int> data = {this->id, COMM_GET_VALUES};
        p2.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector<int> data = {};
        p2.encodeCommand(COMM_GET_VALUES, data);
    }
    p2.send(serial);

    std::vector<uint8_t> buffer = {};
    bool terminate = false;

    Package package;

    while (!terminate) {
        try {
            uint8_t byte;
            boost::asio::read(*serial, boost::asio::buffer(&byte, 1));
            buffer.push_back(byte);

            Package packageTemp;
            std::pair<bool, std::vector<uint8_t>> result = packageTemp.getFromBuffer(buffer);

            if (result.first && packageTemp.payload[0] == 4) {
                package = packageTemp;
                terminate = true;
            }

        } catch (std::exception &e) {
            printf("Error occurred in getFromSerial: %s \n", e.what());
            return false;
        }
    }

    std::vector input = {this};
    package.decode(input);

    if (this->clamped_pos != -1 && this->last_clamped_pos != -1 && this->homed) {
        if (double delta = this->clamped_pos - this->last_clamped_pos; delta > 180) {
            this->rotations -= 1;
        } else if (delta < -180) {
            this->rotations += 1;
        }
    }

    if (this->homed) {
        this->pos = this->clamped_pos + (this->rotations * 360);
    }

    return true;
}

void Motor::setDuty(boost::asio::serial_port* serial, const double duty) const {
    Package package;
    if (this->is_can) {
        const std::vector<int> data = {this->id, COMM_SET_DUTY, static_cast<int>((duty*1e5))};
        package.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector data = {static_cast<int>((duty*1e5))};
        package.encodeCommand(COMM_SET_DUTY, data);
    }
    package.send(serial);
}

void Motor::setPos(boost::asio::serial_port* serial, const double pos) {
    this->getValues(serial);
    const std::vector<uint8_t> new_bytes = Package::packEndian(static_cast<int>((static_cast<double>(static_cast<int>(pos+this->offset) % 360) * 1e6)));
    int startIndex = 1;
    if (this->is_can) {
        startIndex = 3;
    }
    pos_package.payload.erase(pos_package.payload.begin() + startIndex, pos_package.payload.begin() + startIndex + 4);
    pos_package.payload.insert(pos_package.payload.begin() + startIndex, new_bytes.begin(), new_bytes.end());
    const uint16_t new_crc = crc16(pos_package.payload.data(), pos_package.payload.size());
    pos_package.crc[0] = new_crc >> 8 & 0xFF;
    pos_package.crc[1] = new_crc & 0xFF;
    pos_package.send(serial);
}

void Motor::doHoming(boost::asio::serial_port* serial) {
    this->clamped_pos = -1;
    this->last_clamped_pos = -1;
    this->rotations = 0;
    std::vector<double> positions;
    for (int i = 0; i < 20; i++) {
        this->getValues(serial);
        positions.push_back(this->actual_pos);
    }
    this->offset = std::accumulate(positions.begin(), positions.end(), 0.0) / static_cast<double>(positions.size());
    this->homed = true;

    std::cout << "domyhinh\n";

    // IDK if this actually does anything but this is the only trick I could think of to prevent rotations shooting up
    // or down right after homing due to encoder noise
    this->home_cooldown = 5;
    // It's not an endless loop, home_cooldown is reduced to 0 in getValues()
    // ReSharper disable once CppDFAEndlessLoop
    // ReSharper disable once CppDFAConstantConditions
    while (this->home_cooldown > 0) {
        this->getValues(serial);
    }
}

bool Motor::verifyHoming() const {
    if (this->homed) {
        if (std::abs(this->pos) > 1) {
            std::cout << "Position is not (close to) 0, something probably went wrong. Try homing again. " << this->pos << " " << this->rotations << std::endl;
            return false;
        }
        if (this->pos < 0 && this->rotations != -1) {
            std::cout << "Something went wrong with the rotations, try homing again. " << this->pos << " " << this->rotations  << std::endl;
            return false;
        }
        return true;
    }
    return false;
}

// PACKAGE METHODS

Package::Package() :
    startByte(-1),
    payloadLength(-1),
    payload(std::vector<uint8_t>()),
    crc(std::vector<uint8_t>())
{ }

void Package::getFromSerial(boost::asio::serial_port* serial) {
    uint8_t byte;
    while (true) {
        try {
            while (boost::asio::read(*serial, boost::asio::buffer(&byte, 1)) > 0) {
                if (byte == 0x02 || byte == 0x03) {
                    if (this->startByte == -1) {
                        this->startByte = byte;
                        continue;
                    }
                }

                if (byte == 0x03 && this->startByte != -1) {
                    this->payloadLength = this->payload[0];
                    this->crc.assign(this->payload.end() - 2, this->payload.end());
                    this->payload = std::vector(this->payload.begin()+1, this->payload.end()-2);
                    return;
                }

                if (byte != 0x02 && byte != 0x03 && this->startByte != -1) {
                    this->payload.push_back(byte);
                }
            }
        }
        catch (std::exception &e) {
            printf("Error occurred in getFromSerial: %s \n", e.what());
        }
    }
}

std::pair<bool, std::vector<uint8_t>> Package::getFromBuffer(std::vector<uint8_t> buffer) {
    for (size_t index = 0; index < buffer.size(); ++index) {
        auto byte = buffer[index];
        if ((byte == 0x02 || byte == 0x03) && this->startByte == -1) {
            this->startByte = byte;
            continue;
        }

        if (this->startByte != -1 && this->payloadLength == -1) {
            this->payloadLength = byte;
            continue;
        }

        if (byte == 0x03 && static_cast<int>(this->payload.size()) == (this->payloadLength + 2)) {
            this->crc.assign(this->payload.end() - 2, this->payload.end());
            this->payload = std::vector((this->payload.begin()), (this->payload.end()-2));

            const uint16_t crc = crc16(this->payload.data(), this->payload.size());
            const uint8_t crc_test_0 = (crc >> 8) & 0xFF;
            const uint8_t crc_test_1 = crc & 0xFF;
            if (crc_test_0 == this->crc[0] && crc_test_1 == this->crc[1]) {
                return std::make_pair(true, std::vector((buffer.begin()+index), buffer.end()));
            }
            return std::make_pair(false, std::vector((buffer.begin()+1), buffer.end()));
        }

        if (static_cast<int>(this->payload.size()) < (this->payloadLength + 2)) {
            this->payload.push_back(byte);
        } else if (static_cast<int>(this->payload.size()) == (this->payloadLength + 2) && byte != 0x03 && this->payloadLength != -1) {
            return std::make_pair(false, std::vector(buffer.begin()+1, buffer.end()));
        }
    }

    return std::make_pair(false, buffer);
}

void Package::printContents() {
    std::vector<uint8_t> package;
    package.push_back(this->startByte);
    package.push_back(this->payloadLength);
    package.insert(package.end(), this->payload.begin(), this->payload.end());
    package.insert(package.end(), this->crc.begin(), this->crc.end());
    package.push_back(0x03);

    // Print
    std::cout << "[" << static_cast<int>(package[0]);
    for (size_t i = 1; i < package.size(); ++i) {
        std::cout << ", " << static_cast<int>(package[i]);
    }
    std::cout << "]" << std::endl;
}

std::vector<uint8_t> Package::packEndian(int field) {
    // Convert to big-endian format
    const auto big_endian_value = static_cast<boost::endian::big_int32_t>(field);

    // Serialize to bytes
    std::vector<uint8_t> bytes(sizeof(big_endian_value));
    std::memcpy(bytes.data(), &big_endian_value, sizeof(big_endian_value));

    // Uncomment for debugging
    // for (uint8_t byte : bytes) {
    //     //std::cout << std::hex << static_cast<int>(byte) << " ";
    // }
    return bytes;
}

void Package::encodeCommand(const Commands command, std::vector<int> fields) {
    this->startByte = 0x02;
    if (command != COMM_FORWARD_CAN) {
        this->payloadLength = 1 + 4*static_cast<int>(fields.size());
        this->payload.push_back(command);

        for (const int field : fields) {
            std::vector<uint8_t> bytes = packEndian(field);
            for (uint8_t byte : bytes) {
                this->payload.push_back(byte);
            }
        }
    } else {
        this->payloadLength = 3 + 4*(static_cast<int>(fields.size()) - 2);
        this->payload.push_back(command);
        this->payload.push_back(fields[0]);
        this->payload.push_back(fields[1]);
        for (const int field : std::vector(fields.begin()+2, fields.end())) {
            std::vector<uint8_t> bytes = packEndian(field);
            for (uint8_t byte : bytes) {
                this->payload.push_back(byte);
            }
        }
    }

    // PYTHON: crc = crc_checker.calc(bytearray(self.payload))

    // C++:
    // crc_checker.process_bytes(this.payload.data(), this.payload.size());
    // uint8_t crc_test = crc_checker.checksum();

    const uint16_t crc = crc16(this->payload.data(), this->payload.size());
    this->crc.push_back(((crc >> 8) & 0xFF));
    this->crc.push_back((crc & 0xFF));
}

void Package::send(boost::asio::serial_port* serial) {
    std::vector<uint8_t> package;
    package.push_back(this->startByte);
    package.push_back(this->payloadLength);
    package.insert(package.end(), this->payload.begin(), this->payload.end());
    package.insert(package.end(), this->crc.begin(), this->crc.end());
    package.push_back(0x03);

    boost::asio::write(*serial, boost::asio::buffer(package));
}

int16_t Package::unpackH(const int startIndex, const int endIndex) const {
    if (endIndex - startIndex != 2 || startIndex < 0 || endIndex > static_cast<int>(this->payload.size())) {
        throw std::invalid_argument("Invalid range for unpacking 16-bit value.");
    }

    // Combine the two bytes into a 16-bit integer
    const int16_t value = static_cast<int16_t>(this->payload[startIndex]) << 8 |
                    static_cast<int16_t>(this->payload[startIndex + 1]);

    return value;
}

int32_t Package::unpackI(const int startIndex, const int endIndex) const {
    if (endIndex - startIndex != 4 || startIndex < 0 || endIndex > static_cast<int>(this->payload.size())) {
        throw std::invalid_argument("Invalid range for unpacking 32-bit value.");
    }

    // Combine the four bytes into a 32-bit integer
    const int32_t value = static_cast<int32_t>(this->payload[startIndex]) << 24 |
                          static_cast<int32_t>(this->payload[startIndex + 1]) << 16 |
                          static_cast<int32_t>(this->payload[startIndex + 2]) << 8 |
                          static_cast<int32_t>(this->payload[startIndex + 3]);

    return value;
}

int Package::unpackB(const int index) const {
    return static_cast<int8_t>(this->payload[index]);
}

double round_to_decimals(const double value, const int decimals) {
    const double scale = std::pow(10.0, decimals);
    return std::round(value * scale) / scale;
}

void Package::decode(std::vector<Motor*> motors) const {
    const auto command = static_cast<Commands>(static_cast<int>(this->payload[0]));
    bool motor_found = false;

    switch (command) {
        case COMM_GET_VALUES:
            for (const auto motor : motors) {
                if (unpackB(58) == motor->id) {
                    motor_found = true;
                    motor->temp_fet = round_to_decimals((unpackH(1, 3) / 1e1), 1);
                    motor->temp_motor = round_to_decimals((unpackH(3, 5) / 1e1), 1);
                    motor->avg_motor_current = round_to_decimals((unpackI(5, 9) / 1e2), 2);
                    motor->avg_in_current = round_to_decimals((unpackI(9, 13) / 1e2), 2);
                    motor->avg_id = round_to_decimals((unpackI(13, 17) / 1e2), 2);
                    motor->avg_iq = round_to_decimals((unpackI(17, 21) / 1e2), 2);
                    motor->duty_now = round_to_decimals((unpackH(21, 23) / 1e3), 3);
                    motor->rpm = round_to_decimals((unpackI(23, 27) / 1e0), 0);
                    motor->v_in = round_to_decimals((unpackH(27, 29) / 1e1), 1);
                    motor->amp_hours = round_to_decimals((unpackI(29, 33) / 1e4), 4);
                    motor->amp_hours_charged = round_to_decimals((unpackI(33, 37) / 1e4), 4);
                    motor->watt_hours = round_to_decimals((unpackI(37, 41) / 1e4), 4);
                    motor->watt_hours_charged = round_to_decimals((unpackI(41, 45) / 1e4), 4);
                    motor->tachometer = static_cast<int>(round_to_decimals(unpackI(45, 49), 0));
                    motor->tachometer_abs = static_cast<int>(round_to_decimals(unpackI(49, 53), 0));
                    motor->mc_fault_code = unpackB(53);
                    motor->home_cooldown = static_cast<int>(std::fmax((motor->home_cooldown - 1), 0));
                    motor->actual_pos = round_to_decimals((unpackI(54, 58) / 1e6), 6);
                    if (motor->homed) {
                        motor->pid_pos_now = round_to_decimals(((unpackI(54, 58) - 1e6*motor->offset + 1e6*motor->home_cooldown) / 1e6), 6);
                        motor->last_clamped_pos = motor->clamped_pos;
                        motor->clamped_pos = std::fmod(motor->pid_pos_now, 360.0);
                        if (motor->clamped_pos < 0) {
                            motor->clamped_pos += 360.0;
                        }
                    }
                    motor->temp_mos1 =  round_to_decimals((unpackH(59, 61) / 1e1), 1);
                    motor->temp_mos2 =  round_to_decimals((unpackH(61, 63) / 1e1), 1);
                    motor->temp_mos3 =  round_to_decimals((unpackH(63, 65) / 1e1), 1);
                    motor->avg_vd =  static_cast<double>(unpackI(65, 69));
                    motor->avg_vq =  static_cast<double>(unpackI(69, 73));
                    motor->status = unpackB(73);
                }
            }
            if (!motor_found) {
                printf("Package not meant for these motors, but for motor %d\n", unpackB(58));
            }
            break;
        case COMM_ROTOR_POSITION:
            motors[0]->pid_pos_now = unpackI(1, static_cast<int>(this->payload.size())) / 1e5;
            break;
        default:
            std::cout << "Unhandled command: " << command << " (" << static_cast<int>(command) << ")" << std::endl;
    };
}

// CONTROLLER METHODS

Controller::Controller(boost::asio::serial_port* serialport, std::vector<int> can_ids) :
can_ids(std::move(can_ids)),
shutdown(false)
{
    heart_beat_thread = std::thread(&Controller::heartbeatFunc, this, serialport);
}

Controller::~Controller() {
    this->shutdown = true;
    if (heart_beat_thread.joinable()) {
        heart_beat_thread.join();
    }
}

void Controller::heartbeatFunc(boost::asio::serial_port* serialport) {
    try {
        while (!this->shutdown) {
            Package package;
            package.encodeCommand(COMM_ALIVE, std::vector<int>());
            package.send(serialport);

            for (const int can_id : this->can_ids) {
                Package p;
                const std::vector data = {can_id, 30};
                p.encodeCommand(COMM_FORWARD_CAN, data);
                p.send(serialport);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // sleep .1 second
        }
    } catch (std::exception &e) {
        printf("Error occurred in getFromSerial: %s \n", e.what());
    }
}

std::vector<std::pair<double, double>> Controller::goToPos(boost::asio::serial_port* serial, const double pos, std::vector<Motor*> motors, const double degrees_per_step, int acc_substeps, const int velocity, int velocity_rampsteps) {
    std::cout << "starting goToPos" << std::endl;

    mtx.lock();
    for (Motor* motor : motors) {
        motor->getValues(serial);
    }
    mtx.unlock();

    std::cout << "Got values \n";


    if (velocity_rampsteps == -1) {
        velocity_rampsteps = static_cast<int>(velocity / (degrees_per_step * 50));
    }

    std::cout << velocity_rampsteps << std::endl;

    double mid_pos;
    if (motors.size() > 1) {
        double all_positions = 0;
        for (const Motor* motor : motors) {
            all_positions += motor->pos;
        }
        const double average_position = all_positions / static_cast<double>(motors.size());

        for (const Motor* motor : motors) {
            if (std::abs(motor->pos - average_position) > 2) {
                std::cout << "Motors are not aligned!" << std::endl;
                return {};
            }
        }

        mid_pos = average_position;
    } else {
        mid_pos = motors[0]->pos;
    }

    std::vector<std::pair<double, double>> mid_positions = {std::make_pair(mid_pos, 0)};
    double total_time = 0;
    if (mid_pos < pos) {
        while (mid_pos < pos) {
            const int current_step = static_cast<int>(mid_positions.size());

            const double current_velocity = (current_step <= velocity_rampsteps) ? ((velocity / velocity_rampsteps) * current_step) : velocity;

            const double time_increment = degrees_per_step / current_velocity;

            for (int i = 0; i < acc_substeps; i++) {
                total_time += time_increment / acc_substeps;
                mid_pos += degrees_per_step / acc_substeps;
                mid_positions.emplace_back(mid_pos, total_time);
            }

            if (current_step == velocity_rampsteps) {
                acc_substeps = 0;
            }
        }
    } else {
        while (mid_pos > pos) {
            const int current_step = static_cast<int>(mid_positions.size());

            const double current_velocity = (current_step <= velocity_rampsteps) ? ((velocity / velocity_rampsteps) * current_step) : velocity;

            const double time_increment = degrees_per_step / current_velocity;

            for (int i = 0; i < acc_substeps; i++) {
                total_time += time_increment / acc_substeps;
                mid_pos -= degrees_per_step / acc_substeps;
                mid_positions.emplace_back(mid_pos, total_time);
            }

            if (current_step == velocity_rampsteps) {
                acc_substeps = 0;
            }
        }
    }

    // Change the ending steps to slowly decelerate, 3 times as slow as the initial acceleration
    // velocity_rampsteps /= 2;
    if (velocity_rampsteps > static_cast<int>(mid_positions.size())) {
        velocity_rampsteps = static_cast<int>(mid_positions.size());
    }
    total_time = mid_positions[mid_positions.size() - velocity_rampsteps - 1].second;

    int i = velocity_rampsteps;
    while (i > 0) {
        const double current_velocity = (static_cast<double>(velocity) / velocity_rampsteps) * (i);
        const double time_increment = degrees_per_step / current_velocity;
        total_time += time_increment;
        mid_positions[(mid_positions.size() - i)].second = total_time;
        i -= 1;
    }

    // Make sure the last position is the desired position
    if (mid_positions[(mid_positions.size()-1)].first != pos) {
        mid_positions[(mid_positions.size()-1)].first = pos;
    }

    // for (auto &[pos, time] : mid_positions) {
    //     std::cout << pos << " " << time << std::endl;
    // }

    std::cout << "done calc\n";

    return mid_positions;

    // const auto start_time = std::chrono::high_resolution_clock::now();
    // for (auto [pos, timestamp] : mid_positions) {
    //     auto current_time = std::chrono::high_resolution_clock::now();
    //     if (const double elapsed = std::chrono::duration<double>(current_time - start_time).count(); timestamp > elapsed) {
    //         std::this_thread::sleep_for(std::chrono::duration<double>(timestamp - std::chrono::duration<double>((std::chrono::high_resolution_clock::now() - start_time)).count()));
    //     } else {
    //         std::cout << "Missed a step" << std::endl;
    //         continue;
    //     }

    //     for (Motor* motor : motors) {
    //         motor->setPos(serial, pos);
    //     }
    // }

    // mtx4.lock();

    // for (Motor* motor : motors) {
    //     motor->getValues(serial);
    // }

    // mtx4.unlock();
}

struct Path {
    std::pair<double, double> val;
    int dir; // 0 = horizontal, 1 = vertical
};

void Controller::goToPosXY(boost::asio::serial_port *serial, std::pair<double, double> pos, Motor *motorHorizontal,
                           std::vector<Motor *> motorsVertical, double degrees_per_step, int acc_substeps, int velocity,
                           int velocity_rampsteps) {
    double x = pos.first;   // horizontal
    double y = pos.second;  // vertical
 
 
    auto horizontalPath = goToPos(serial, x,  std::vector<Motor *>{motorHorizontal}, degrees_per_step, acc_substeps, velocity, velocity_rampsteps);
    auto verticalPath = goToPos(serial, y, motorsVertical, degrees_per_step, acc_substeps, velocity, velocity_rampsteps);

    if (horizontalPath.size() == 0 || verticalPath.size() == 0) return;

    auto finalPath = std::vector<Path>();
    for (auto [x, y] : horizontalPath) {
        finalPath.push_back({{x, 10 * y}, 0});
    }
    // for (auto [x, y] : verticalPath) {
    //     finalPath.push_back({{x, y + 0.001}, 1});
    // }

    sort(finalPath.begin(), finalPath.end(), [&](auto &p1, auto &p2) {
        if (p1.val.second < p2.val.second) return true;
        else if (p1.val.second > p2.val.second) return false;
        
        return p1.val.first < p2.val.first;
    });

    int similar = 0;

    for (size_t i = 1; i < finalPath.size(); i++) {
        if (finalPath[i].val.second == finalPath[i-1].val.second) {
            similar++;
        }
    }



    // return;
    int missed = 0;
    const auto start_time = std::chrono::high_resolution_clock::now();
    for (auto pos : finalPath) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto timestamp = pos.val.second;
        if (const double elapsed = std::chrono::duration<double>(current_time - start_time).count(); timestamp > elapsed) {
            std::this_thread::sleep_for(std::chrono::duration<double>(timestamp - std::chrono::duration<double>((std::chrono::high_resolution_clock::now() - start_time)).count()));
        } else {
            missed++;
            // std::cout << "Missed a step" << std::endl;
            continue;
        }

        // for (Motor* motor : motors) {
        //     motor->setPos(serial, pos);
        // }
        if (pos.dir == 0) {
            motorHorizontal->setPos(serial, pos.val.first);
        } else {
            for (Motor* motor : motorsVertical) {
                motor->setPos(serial, pos.val.first);
            }
        }
    }

    std::cout << "Missed: " << missed << std::endl;
    std::cout << "Similar: " << similar << std::endl;
    std::cout << "total: " << finalPath.size() << std::endl;
    std::cout << "horizontal: " << horizontalPath.size() << std::endl;
    std::cout << "vertical: " << verticalPath.size() << std::endl;

    
}
