

from pyrplidar import PyRPlidar
import time
import serial
from math import *
import stop
import numpy as np
from multiprocessing import Process, Array
import ctypes as c
import pylas


def plot_scatter(points, sc, fig):
    sc.set_offsets(points)
    fig.canvas.draw()


class RPLidar:
    def __init__(self):
        self.lidar = PyRPlidar()
        self.lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)

        self.lidar.set_motor_pwm(500)
        time.sleep(2)

        self.scan_generator = self.lidar.force_scan()
        self.last_angle = 350
        self.points = np.zeros((360, 2))

        self.rev_count = 0
        self.time_at_last_scan = 0

    def take_measurement(self, sp):
        for count, scan in enumerate(self.scan_generator()):
            if scan.distance > 0:
                if scan.angle < self.last_angle:
                    sp[1440] += 1

                current_angle_index = floor(scan.angle * 2)

                if sp[min(719, current_angle_index)] == 0 and sp[min(719, current_angle_index) + 720] == 0:
                    sp[min(719, current_angle_index)] = float(scan.distance) * cos(radians(scan.angle))
                    sp[min(719, current_angle_index) + 720] = float(scan.distance) * sin(radians(scan.angle))
                else:
                    sp[min(719, current_angle_index)] = (sp[min(719, current_angle_index)] + float(scan.distance) * cos(
                        radians(scan.angle))) / 2
                    sp[min(719, current_angle_index) + 720] = (sp[min(719, current_angle_index) + 720] + float(
                        scan.distance) * sin(radians(scan.angle))) / 2
                self.last_angle = scan.angle


class TicSerial(object):
    def __init__(self, port, device_number=None):
        self.port = port
        self.device_number = device_number

    def send_command(self, cmd, *data_bytes):
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(bytes(header + list(data_bytes)))

    def exit_safe_start(self):
        self.send_command(0x83)

    def set_target_position(self, target):
        self.send_command(0xE0,
                          ((target >> 7) & 1) | ((target >> 14) & 2) |
                          ((target >> 21) & 4) | ((target >> 28) & 8),
                          target >> 0 & 0x7F,
                          target >> 8 & 0x7F,
                          target >> 16 & 0x7F,
                          target >> 24 & 0x7F)

    def set_target_velocity(self, target):
        self.send_command(0xE3,
                          ((target >> 7) & 1) | ((target >> 14) & 2) |
                          ((target >> 21) & 4) | ((target >> 28) & 8),
                          target >> 0 & 0x7F,
                          target >> 8 & 0x7F,
                          target >> 16 & 0x7F,
                          target >> 24 & 0x7F)

    def reset_position(self, target=0):
        self.send_command(0xEC,
                          ((target >> 7) & 1) | ((target >> 14) & 2) |
                          ((target >> 21) & 4) | ((target >> 28) & 8),
                          target >> 0 & 0x7F,
                          target >> 8 & 0x7F,
                          target >> 16 & 0x7F,
                          target >> 24 & 0x7F)

    # Gets one or more variables from the Tic.
    def get_variables(self, offset, length):
        self.send_command(0xA1, offset, length)
        result = self.port.read(length)
        if len(result) != length:
            raise RuntimeError("Expected to read {} bytes, got {}."
                               .format(length, len(result)))
        return bytearray(result)

    # Gets the "Current position" variable from the Tic.
    def get_current_position(self):
        b = self.get_variables(0x22, 4)
        position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
        if position >= (1 << 31):
            position -= (1 << 32)
        return position

    # Gets the "Current position" variable from the Tic.
    def get_current_potentio_position(self):
        b = self.get_variables(0x41, 2)
        position = b[0] + (b[1] << 8)
        return position


def map_values(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


rotational_values = [0] * 20
current_count = 0


def current_pos():
    global current_count, rotational_values
    in_minimum, in_maximum, out_minimum, out_maximum = 6912, 50000, 180, 0

    value = map_values(tic.get_current_potentio_position(), in_minimum, in_maximum, out_minimum, out_maximum)
    rotational_values[current_count] = value
    current_count += 1
    if current_count > 19: current_count = 0
    return sum(rotational_values) / 20


def current_pos_accurate():
    for i in range(80):
        current_pos()
    return current_pos()


def go_to_pos(target_position, tic):
    correct_setting_count = 0
    while True:

        offset = current_pos() - target_position
        if offset > 2:
            tic.set_target_velocity(-20000000)
        elif offset < -2:
            tic.set_target_velocity(20000000)
        elif offset > 1:
            tic.set_target_velocity(-5000000)
        elif offset < -1:
            tic.set_target_velocity(5000000)
        elif offset > 0.5:
            tic.set_target_velocity(-1000000)
        elif offset < -0.5:
            tic.set_target_velocity(1000000)
        else:
            tic.set_target_velocity(0)
            correct_setting_count += 1
            if correct_setting_count > 50:
                return True


if __name__ == "__main__":
    try:
        # Choose the serial port name.
        port_name = "/dev/ttyUSB1"
        baud_rate = 9600
        device_number = None
        port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)
        tic = TicSerial(port, device_number)
        print(current_pos_accurate())
        current_rotation = 80
        go_to_pos(current_rotation, tic)
        tic.reset_position()
        tic.exit_safe_start()
        print("TIC started!")
        rplidar = RPLidar()

        shared_points = Array(c.c_float, 360 * 4 + 1)
        t = Process(target=rplidar.take_measurement, args=(shared_points,))
        t.start()

        while t.is_alive():
            if shared_points[1440] > 30:
                las = pylas.create()
                angle_multiplier = cos(radians(current_rotation))
                shared_points[0:720] = np.flip(shared_points[0:720])
                las.X = [element * angle_multiplier for element in shared_points[0:720]]
                las.Z = shared_points[720:1440]
                las.Y = shared_points[0:720]
                las.write("Output{}.las".format(current_rotation))
                print("wrote las")
                current_rotation += 0.5
                tic.set_target_position(int(444 * (current_rotation - 80)))
                time.sleep(1)
                shared_points[0:1441] = [0] * 1441
                if current_rotation > 170:
                    t.kill()


    except KeyboardInterrupt:
        stop.disconnect()
    except Exception as e:
        print(e)
        stop.disconnect()
