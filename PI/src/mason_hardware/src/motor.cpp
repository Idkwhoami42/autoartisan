#include "mason_hardware/motor.h"

#include <unistd.h>

#include <boost/crc.hpp>
#include <boost/endian/arithmetic.hpp>
#include <boost/endian/conversion.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>
#include <utility>
#include <vector>

#include "crc.h"

std::mutex mtx;

// Constructor
Motor::Motor(std::string joint_name, const int id, boost::asio::serial_port* serial,
             const bool is_can)
    : joint_name(std::move(joint_name)),
      id(id),
      is_can(is_can),
      offset(0),
      homed(false),
      home_cooldown(0),
      rotations(0),
      pid_pos_now(-1),
      actual_pos(-1),
      last_clamped_pos(-1),
      clamped_pos(-1),
      pos(-1),
      vel(0),
      cmd(0) {
    pos_package = Package();
    if (this->is_can) {
        const std::vector data = {this->id, static_cast<int>(COMM_SET_POS), 0};
        pos_package.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector data = {0};
        pos_package.encodeCommand(COMM_SET_POS, data);
    }

    Package p;
    if (this->is_can) {
        const std::vector<int> data = {this->id, COMM_SET_DETECT, 0};
        p.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector<int> data = {0};
        p.encodeCommand(COMM_SET_DETECT, data);
    }
    p.send(serial);
}

int32_t Motor::unpackI(const std::vector<uint8_t>& buffer, const int startIndex,
                       const int endIndex) {
    if (endIndex - startIndex != 4 || startIndex < 0 ||
        endIndex > static_cast<int>(buffer.size())) {
        throw std::invalid_argument("Invalid range for unpacking 32-bit value.");
    }

    // Combine the four bytes into a 32-bit integer
    const int32_t value = static_cast<int32_t>(buffer[startIndex]) << 24 |
                          static_cast<int32_t>(buffer[startIndex + 1]) << 16 |
                          static_cast<int32_t>(buffer[startIndex + 2]) << 8 |
                          static_cast<int32_t>(buffer[startIndex + 3]);

    return value;
}

bool Motor::getValues(boost::asio::serial_port* serial) {
    Package p2;
    if (this->is_can) {
        const std::vector<int> data = {this->id, COMM_GET_VALUES_SELECTIVE, 1 << 16};
        p2.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector<int> data = {1 << 16};
        p2.encodeCommand(COMM_GET_VALUES_SELECTIVE, data);
    }
    p2.send(serial);

    std::vector<uint8_t> buffer(14);
    boost::asio::read(*serial, boost::asio::buffer(buffer, 14));

    this->home_cooldown = static_cast<int>(std::fmax((this->home_cooldown - 1), 0));
    this->actual_pos = unpackI(buffer, 7, 11) / 1e6;
    if (this->homed) {
        this->pid_pos_now =
            ((unpackI(buffer, 7, 11) - 1e6 * this->offset + 1e6 * this->home_cooldown) / 1e6);
        this->last_clamped_pos = this->clamped_pos;
        this->clamped_pos = std::fmod(this->pid_pos_now, 360.0);
        if (this->clamped_pos < 0) {
            this->clamped_pos += 360.0;
        }
    }

    if (this->clamped_pos != -1 && this->last_clamped_pos != -1 && this->homed) {
        if (const double delta = this->clamped_pos - this->last_clamped_pos; delta > 180) {
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
        const std::vector<int> data = {this->id, COMM_SET_DUTY, static_cast<int>((duty * 1e5))};
        package.encodeCommand(COMM_FORWARD_CAN, data);
    } else {
        const std::vector data = {static_cast<int>((duty * 1e5))};
        package.encodeCommand(COMM_SET_DUTY, data);
    }
    package.send(serial);
}

void Motor::setPos(boost::asio::serial_port* serial, const double pos) {
    if (!this->homed) {
        std::cout << "This motor is not homed, aborting" << std::endl;
        return;
    }
    // this->getValues(serial);
    const std::vector<uint8_t> new_bytes = Package::packEndian(
        static_cast<int>((static_cast<double>(static_cast<int>(pos + this->offset) % 360) * 1e6)));
    int startIndex = 1;
    if (this->is_can) {
        startIndex = 3;
    }
    pos_package.payload.erase(pos_package.payload.begin() + startIndex,
                              pos_package.payload.begin() + startIndex + 4);
    pos_package.payload.insert(pos_package.payload.begin() + startIndex, new_bytes.begin(),
                               new_bytes.end());
    const uint16_t new_crc = crc16(pos_package.payload.data(), pos_package.payload.size());
    pos_package.crc[0] = new_crc >> 8 & 0xFF;
    pos_package.crc[1] = new_crc & 0xFF;
    pos_package.send(serial);
}

uint32_t Motor::buffer_append_float32_auto(float number) {
    // Set subnormal numbers to 0 as they are not handled properly
    // using this method.
    if (fabsf(number) < 1.5e-38) {
        number = 0.0;
    }

    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;

    if (sig_abs >= 0.5) {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0) {
        res |= 1U << 31;
    }

    return res;
}

void Motor::setDerivativeGain(boost::asio::serial_port& serial, float gain) {
    Package p;
    p.encodeCommand(COMM_FORWARD_CAN, {this->id, COMM_GET_MCCONF});
    p.send(&serial);

    std::vector<uint8_t> buffer(484);
    boost::asio::read(serial, boost::asio::buffer(buffer, 484));

    Package p2;

    auto new_buffer = std::vector(buffer.begin(), buffer.end());

    auto newpid = buffer_append_float32_auto(gain);
    new_buffer[355] = newpid >> 24;
    new_buffer[356] = newpid >> 16;
    new_buffer[357] = newpid >> 8;
    new_buffer[358] = newpid;
    new_buffer[3] = COMM_SET_MCCONF;

    uint16_t new_buffer2_length = (buffer[1] << 8 | buffer[2]) + 2;
    std::vector<uint8_t> new_buffer2 = {3, static_cast<uint8_t>(new_buffer2_length >> 8),
                                        static_cast<uint8_t>(new_buffer2_length & 0xFF),
                                        COMM_FORWARD_CAN, static_cast<uint8_t>(this->id)};

    new_buffer2.insert(new_buffer2.end(), new_buffer.begin() + 3, new_buffer.end() - 3);

    unsigned char crcBuffer[new_buffer2_length];
    for (int i = 0; i < new_buffer2_length; i++) {
        crcBuffer[i] = new_buffer2[i + 3];
    }

    auto crc = crc16(crcBuffer, new_buffer2_length);
    new_buffer2.push_back((crc >> 8) & 0xFF);
    new_buffer2.push_back(crc & 0xFF);
    new_buffer2.push_back(3);

    if (new_buffer2.size() != new_buffer2_length + 6) {
        std::cout << "huh";
    }

    boost::asio::write(serial, boost::asio::buffer(new_buffer2, new_buffer2.size()));
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
    this->offset = std::accumulate(positions.begin(), positions.end(), 0.0) /
                   static_cast<double>(positions.size());
    this->homed = true;

    // IDK if this actually does anything but this is the only trick I could think of to prevent
    // rotations shooting up or down right after homing due to encoder noise
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
            std::cout
                << "Position is not (close to) 0, something probably went wrong. Try homing again. "
                << this->pos << " " << this->rotations << std::endl;
            return false;
        }
        if (this->pos < 0 && this->rotations != -1) {
            std::cout << "Something went wrong with the rotations, try homing again. " << this->pos
                      << " " << this->rotations << std::endl;
            return false;
        }
        return true;
    }
    return false;
}

// PACKAGE METHODS

Package::Package()
    : startByte(-1),
      payloadLength(-1),
      payload(std::vector<uint8_t>()),
      crc(std::vector<uint8_t>()) {}

std::vector<uint8_t> Package::packEndian(const int field) {
    // Convert to big-endian format
    const auto big_endian_value = static_cast<boost::endian::big_int32_t>(field);

    // Serialize to bytes
    std::vector<uint8_t> bytes(sizeof(big_endian_value));
    std::memcpy(bytes.data(), &big_endian_value, sizeof(big_endian_value));

    return bytes;
}

void Package::encodeCommand(const Commands command, std::vector<int> fields) {
    this->startByte = 0x02;
    if (command != COMM_FORWARD_CAN) {
        this->payloadLength = 1 + 4 * static_cast<int>(fields.size());
        this->payload.push_back(command);

        for (const int field : fields) {
            std::vector<uint8_t> bytes = packEndian(field);
            for (uint8_t byte : bytes) {
                this->payload.push_back(byte);
            }
        }
    } else {
        this->payloadLength = 3 + 4 * (static_cast<int>(fields.size()) - 2);
        this->payload.push_back(command);
        this->payload.push_back(fields[0]);
        this->payload.push_back(fields[1]);
        for (const int field : std::vector(fields.begin() + 2, fields.end())) {
            std::vector<uint8_t> bytes = packEndian(field);
            for (uint8_t byte : bytes) {
                this->payload.push_back(byte);
            }
        }
    }

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

// CONTROLLER METHODS

Controller::Controller(boost::asio::serial_port* serialport, std::vector<int> can_ids)
    : can_ids(std::move(can_ids)), shutdown(false) {
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
            // Package package;
            // package.encodeCommand(COMM_ALIVE, std::vector<int>());
            // package.send(serialport);

            for (const int can_id : this->can_ids) {
                Package p;
                const std::vector data = {can_id, 30};
                p.encodeCommand(COMM_FORWARD_CAN, data);
                p.send(serialport);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // sleep .1 second
        }
    } catch (std::exception& e) {
        printf("Error occurred in heartbeatFunc: %s \n", e.what());
    }
}

void Controller::goToPos(boost::asio::serial_port* serial, const double pos,
                         std::vector<Motor*> motors, const double degrees_per_step,
                         int acc_substeps, const int velocity, int velocity_rampsteps) {
    std::cout << "starting goToPos" << std::endl;

    for (Motor* motor : motors) {
        if (!motor->homed) {
            std::cout << "The motor is not homed!" << std::endl;
            return;
        }
        motor->getValues(serial);
    }

    if (velocity_rampsteps == -1) {
        velocity_rampsteps = static_cast<int>(velocity / (degrees_per_step * 25));
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
            }
        }

        mid_pos = average_position;
    } else {
        mid_pos = motors[0]->pos;
    }

    std::cout << "Starting movement from " << mid_pos << std::endl;

    std::vector<std::pair<double, double>> mid_positions = {std::make_pair(mid_pos, 0)};
    double total_time = 0;
    if (mid_pos < pos) {
        while (mid_pos < pos) {
            const int current_step = static_cast<int>(mid_positions.size());

            const double current_velocity =
                (current_step <= velocity_rampsteps)
                    ? ((static_cast<double>(velocity) / velocity_rampsteps) * current_step)
                    : velocity;

            const double time_increment = degrees_per_step / current_velocity;

            for (int i = 0; i < acc_substeps; i++) {
                total_time += time_increment / acc_substeps;
                mid_pos += degrees_per_step / acc_substeps;
                mid_positions.emplace_back(mid_pos, total_time);
            }

            if ((current_step - 1) / acc_substeps == velocity_rampsteps) {
                acc_substeps = 1;
            }
        }
    } else {
        while (mid_pos > pos) {
            const int current_step = static_cast<int>(mid_positions.size());

            const double current_velocity =
                (current_step <= velocity_rampsteps)
                    ? ((static_cast<double>(velocity) / velocity_rampsteps) * current_step)
                    : velocity;

            const double time_increment = degrees_per_step / current_velocity;

            for (int i = 0; i < acc_substeps; i++) {
                total_time += time_increment / acc_substeps;
                mid_pos -= degrees_per_step / acc_substeps;
                mid_positions.emplace_back(mid_pos, total_time);
            }

            if ((current_step - 1) / acc_substeps == velocity_rampsteps) {
                acc_substeps = 1;
            }
        }
    }

    // Change the ending steps to slowly decelerate, 2 times as slow as the initial acceleration
    velocity_rampsteps *= 2;
    if (velocity_rampsteps > mid_positions.size()) {
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
    if (mid_positions[(mid_positions.size() - 1)].first != pos) {
        mid_positions[(mid_positions.size() - 1)].first = pos;
    }

    // for (auto &[pos, time] : mid_positions) {
    //     std::cout << pos << " " << time << std::endl;
    // }
    //
    // return;

    std::cout << "done calc\n";

    int missed = 0;
    int it = 0;

    const auto start_time = std::chrono::high_resolution_clock::now();
    for (auto [pos, timestamp] : mid_positions) {
        double new_timestamp = 2 * timestamp;
        auto current_time = std::chrono::high_resolution_clock::now();
        if (const double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            new_timestamp > elapsed) {
            std::this_thread::sleep_for(std::chrono::duration<double>(
                new_timestamp - std::chrono::duration<double>(
                                    (std::chrono::high_resolution_clock::now() - start_time))
                                    .count()));
        } else {
            missed++;
            continue;
        }

        // std::cout << pos << ", " << timestamp << std::endl;

        for (Motor* motor : motors) {
            motor->setPos(serial, pos);
            if (it % static_cast<int>(30 / degrees_per_step) == 0) {
                motor->getValues(serial);
            }
        }
        it++;
    }

    std::cout << "Finished the loop with " << missed << " misses, with a total of "
              << mid_positions.size() << " steps" << std::endl;

    for (Motor* motor : motors) {
        motor->getValues(serial);
    }

    std::cout << "Got the final values" << std::endl;
}

void Controller::goToPosXY(boost::asio::serial_port* serial, const double pos_x, const double pos_y,
                           Motor* horizontal_motor, std::vector<Motor*> vertical_motors,
                           const double degrees_per_step, const int acc_substeps,
                           const int velocity, int velocity_rampsteps) {
    std::cout << "Going to go to pos " << pos_x << ", " << pos_y << std::endl;

    this->goToPos(serial, pos_x, {horizontal_motor}, degrees_per_step, acc_substeps, velocity,
                  velocity_rampsteps);
    this->goToPos(serial, pos_y, std::move(vertical_motors), degrees_per_step, acc_substeps,
                  velocity, velocity_rampsteps);

    std::cout << "Finished" << std::endl;
}