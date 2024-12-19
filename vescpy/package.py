from crccheck.crc import CrcXmodem
import struct
from commands import Commands

crc_checker = CrcXmodem()

class Package:
    def __init__(self):
        self.startByte = -1
        self.payloadLength = -1
        self.payload = []
        self.crc = []

    def getFromSerial(self, serial):
        while True:
            for byte in serial.read():
                if byte == 0x02 or byte == 0x03:
                    if self.startByte == -1:
                        self.startByte = byte
                        continue

                if byte == 0x03 and self.startByte != -1:
                    self.payloadLength = self.payload[0]
                    self.crc = self.payload[-2:]
                    self.payload = self.payload[1:-2]
                    return
                    
                if byte != 0x02 and byte != 0x03 and self.startByte != -1:
                    self.payload.append(byte)

    def getFromBuffer(self, buffer):
        for i, byte in enumerate(buffer):
            if (byte == 0x02 or byte == 0x03) and self.startByte == -1:
                self.startByte = byte
                continue

            if self.startByte != -1 and self.payloadLength == -1:
                self.payloadLength = byte
                continue

            if byte == 0x03 and len(self.payload) == (self.payloadLength + 2):
                self.crc = self.payload[-2:]
                self.payload = self.payload[:-2]
                crc_test = crc_checker.calc(bytearray(self.payload))
                crc_test_0 = (crc_test >> 8) & 0xFF
                crc_test_1 = crc_test & 0xFF
                if crc_test_0 == self.crc[0] and crc_test_1 == self.crc[1]:
                    return True, buffer[i:]
                else:
                    return False, buffer[1:]
                
            if len(self.payload) < (self.payloadLength + 2): ## +2 for crc
                self.payload.append(byte)
            elif len(self.payload) == (self.payloadLength + 2) and byte != 0x03 and self.payloadLength != -1:
                return False, buffer[1:]
            
        return False, buffer

    def print_contents(self):
        print([self.startByte, self.payloadLength] + self.payload + self.crc + [0x03])

    def encode_command(self, command, fields):
        self.startByte = 0x02
        if command != Commands.COMM_FORWARD_CAN:
            self.payloadLength = 1 + len(fields) * 4
            self.payload.append(command.value)
            for field in fields:
                for byte in struct.pack('>i', field):
                    self.payload.append(byte)
        else:
            self.payloadLength = 3 + (len(fields) - 2) * 4
            self.payload.append(command.value)
            self.payload.append(fields[0])
            self.payload.append(fields[1])
            for field in fields[2:]:
                for byte in struct.pack('>i', field):
                    self.payload.append(byte)
        crc = crc_checker.calc(bytearray(self.payload))
        self.crc.append((crc >> 8) & 0xFF)
        self.crc.append(crc & 0xFF)

    def send(self, serial):
        command = Commands(int(self.payload[0]))
        package = bytearray([self.startByte, self.payloadLength] + self.payload + self.crc + [0x03])
        serial.write(package)

    def decode(self, motors):
        command = Commands(int(self.payload[0]))
        
        match command:
            case Commands.COMM_GET_VALUES:
                motor_found = False
                for motor in motors:
                    if struct.unpack('>b', bytearray([self.payload[58]]))[0] == motor.id:
                        motor_found = True
                        motor.temp_fet = struct.unpack('>h', bytearray(self.payload[1:3]))[0] / 1e1
                        motor.temp_motor = struct.unpack('>h', bytearray(self.payload[3:5]))[0] / 1e1
                        motor.avg_motor_current = struct.unpack('>i', bytearray(self.payload[5:9]))[0] / 1e2
                        motor.avg_in_current = struct.unpack('>i', bytearray(self.payload[9:13]))[0] / 1e2
                        motor.avg_id = struct.unpack('>i', bytearray(self.payload[13:17]))[0] / 1e2
                        motor.avg_iq = struct.unpack('>i', bytearray(self.payload[17:21]))[0] / 1e2
                        motor.duty_now = struct.unpack('>h', bytearray(self.payload[21:23]))[0] / 1e3
                        motor.rpm = struct.unpack('>i', bytearray(self.payload[23:27]))[0] / 1e0
                        motor.v_in = struct.unpack('>h', bytearray(self.payload[27:29]))[0] / 1e1
                        motor.amp_hours = struct.unpack('>i', bytearray(self.payload[29:33]))[0] / 1e4
                        motor.amp_hours_charged = struct.unpack('>i', bytearray(self.payload[33:37]))[0] / 1e4
                        motor.watt_hours = struct.unpack('>i', bytearray(self.payload[37:41]))[0] / 1e4
                        motor.watt_hours_charged = struct.unpack('>i', bytearray(self.payload[41:45]))[0] / 1e4
                        motor.tachometer = struct.unpack('>i', bytearray(self.payload[45:49]))[0]
                        motor.tachometer_abs = struct.unpack('>i', bytearray(self.payload[49:53]))[0]
                        motor.mc_fault_code = struct.unpack('>b', bytearray([self.payload[53]]))[0]
                        motor.pid_pos_now = (struct.unpack('>i', bytearray(self.payload[54:58]))[0] / 1e6) - motor.offset + motor.home_cooldown
                        motor.home_cooldown = max(motor.home_cooldown - 1, 0)
                        motor.actual_pos = (struct.unpack('>i', bytearray(self.payload[54:58]))[0] / 1e6)
                        if motor.homed:
                            motor.last_clamped_pos = motor.clamped_pos if motor.clamped_pos != motor.offset else 0
                        motor.clamped_pos = (motor.pid_pos_now % 360)
                        if motor.direction == 1:
                            motor.pos = motor.clamped_pos + motor.rotations * 360
                        else:
                            motor.pos = -(motor.clamped_pos + motor.rotations * 360)
                        motor.temp_mos1 = struct.unpack('>h', bytearray(self.payload[59:61]))[0] / 1e1
                        motor.temp_mos2 = struct.unpack('>h', bytearray(self.payload[61:63]))[0] / 1e1
                        motor.temp_mos3 = struct.unpack('>h', bytearray(self.payload[63:65]))[0] / 1e1
                        motor.avg_vd = struct.unpack('>i', bytearray(self.payload[65:69]))[0]
                        motor.avg_vq = struct.unpack('>i', bytearray(self.payload[69:73]))[0]
                        motor.status = struct.unpack('>b', bytearray([self.payload[73]]))[0]
                if not motor_found:
                    print("Package not meant for these motors, but for motor {}!".format(struct.unpack('>b', bytearray([self.payload[58]]))[0]))
            case Commands.COMM_ROTOR_POSITION:
                motors[0].pid_pos_now = struct.unpack('>i', bytearray(self.payload[1:]))[0] / 1e5
