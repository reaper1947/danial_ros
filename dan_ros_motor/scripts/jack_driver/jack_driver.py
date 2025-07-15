#!/usr/bin/env python3

import can
import struct
import time
import math
import sys
import threading


class JackDriver:
    def __init__(self, motor_id=3, channel='can0'):
        # Bus
        self.motor_id = motor_id
        self.bus = can.Bus(channel=channel, bustype='socketcan',bitrate=500000)
        self.bus_filter_id = [0x180 + self.motor_id, 0x280 + self.motor_id] 
        self.bus_filter = [{'can_id': fid, 'can_mask': 0x7FF} for fid in self.bus_filter_id]
        self.bus.set_filters(self.bus_filter)

        self.gear_ratio = 20.0

        self.upper_limit = False
        self.lower_limit = False

        self.fault_detected = False
        self.last_status_word = 0
        self._stop_flag = False

        # Start background thread
        self.fault_monitor_thread = threading.Thread(target=self._monitor_faults, daemon=True)
        self.fault_monitor_thread.start()

    def _monitor_faults(self):
        while not self._stop_flag:
            status_word = self.read_id([0x183]) 
            self.last_status_word = status_word

            if status_word is not None:
                fault_bit = (status_word.data[-2] >> 3) & 1
                if fault_bit:
                    print("FAULT DETECT!!!")
                    self.set_motor('Fault Reset')
                    time.sleep(0.1)
                    self.set_motor('Shutdown')
                    time.sleep(0.1)
                    self.set_motor('Switchon')
                    time.sleep(0.1)
                    self.set_motor('Enable Operation')
                    time.sleep(0.1)
                    self.stop_motor()
                    time.sleep(0.1)
                    print("CLEAR FAULT")

            time.sleep(0.1)  # Small delay to reduce CPU load

    def read_id(self, id):
        start = time.monotonic()
        while time.monotonic() - start < 1:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            if msg.arbitration_id in id:
                return msg
        return None  # timeout

    def set_motor(self, command):
        # CIA 402
        if command == 'Shutdown':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False))
        elif command == 'Switchon':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00], is_extended_id=False))
        elif command == 'Enable Operation':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00], is_extended_id=False))
        elif command == 'Fault Reset':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False))
        # DIN1 Disable
        elif command == 'DIN1_disable':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2f, 0x10, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00], is_extended_id=False))
        # DIN1 Enable
        elif command == 'DIN1_enable':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2f, 0x10, 0x20, 0x03, 0x01, 0x00, 0x00, 0x00], is_extended_id=False))
        # Enable Heartbeat
        elif command == 'Enable Heartbeat':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2b, 0x17, 0x10, 0x00, 0x0a, 0x00, 0x00, 0x00], is_extended_id=False))
        # Enable CAN
        elif command == 'Enable CAN':
            self.bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00], is_extended_id=False))
        # Set Position Mode
        elif command == 'Set Postion Mode':
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00], is_extended_id=False))
        # Set default opeation mode to profile speed
        elif command == 'Set Profile Speed Mode':
            # self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2f, 0x20, 0x20, 0x0e, 0x03, 0x00, 0x00, 0x00], is_extended_id=False))
            self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=[0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00], is_extended_id=False))
        # Set default opeation mode to raw speed
        else:
            sys.exit("UNKNOWN TRANSITION")

    def init_motor(self):
        self.set_motor('Enable CAN')        
        time.sleep(0.1)
        self.set_motor('DIN1_disable')
        time.sleep(0.1)
        self.set_motor('Enable Heartbeat')
        time.sleep(0.1)
        self.set_motor('Shutdown')
        time.sleep(0.1)
        self.set_motor('Switchon')
        time.sleep(0.1)
        self.set_motor('Enable Operation')
        time.sleep(0.1)
        # self.set_motor('Set Raw Speed Mode')
        self.set_motor('Set Profile Speed Mode')
        time.sleep(0.1)
        self.set_accel_decel(26214,26214)
        time.sleep(0.1)

    def rpm_to_data_frame(self, rpm):
        mystery_coeff = math.pi
        objects_DEC = (int(rpm*mystery_coeff) * 512 * 2500 * 4) / 1875
        vel_hex = struct.pack('<i', int(objects_DEC))
        data_frame = [
            0x23,
            0xFF,
            0x60,
            0x00,
            vel_hex[0],
            vel_hex[1],
            vel_hex[2],
            vel_hex[3]]
        return data_frame

    def emergency_stop(self):
        self.set_motor('Shutdown')
        self.stop_motor()

    def restart(self):
        self.set_motor('Switchon')
        self.set_motor('Enable Operation')

    def set_speed(self, speed_rpm):
        self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=self.rpm_to_data_frame(speed_rpm), is_extended_id=False))

    def get_speed(self):
        SCALING_FACTOR = 1875 / (512 * 10000 * math.pi)

        speed = self.read_id([0x183])
        if speed:
            if len(speed.data) >= 6:
                raw_bytes = speed.data[0:4]
                raw_int = int.from_bytes(raw_bytes, byteorder='little', signed=True)
                rpm = raw_int * SCALING_FACTOR 
                return rpm

    def stop_motor(self):
        """
        Stop motor by sending zero speed
        """
        self.set_speed(0)

    def set_accel_decel(self,acc, dec):
        """
        Set acceleration and deceleration (example via SDO)
        This is just a placeholder. You need to implement SDO write
        """
        value_hex = struct.pack('<i', int(acc))
        data_frame = [0x23, 0x83, 0x60, 0x00, value_hex[0], value_hex[1], value_hex[2], value_hex[3]]
        self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=data_frame, is_extended_id=False))
        value_hex = struct.pack('<i', int(dec))
        data_frame = [0x23, 0x84, 0x60, 0x00, value_hex[0], value_hex[1], value_hex[2], value_hex[3]]
        self.bus.send(can.Message(arbitration_id=0x600 | self.motor_id, data=data_frame, is_extended_id=False))

    def read_din_status(self):
        """
        Read digital input status.
        This requires a CAN frame or SDO read.
        Placeholder for your specific device.
        """
        # Send SDO read request for 0x2010:0A10 (DIN real input)

        din_state = self.read_id([0x283])
        # print(din_state)
        if din_state:
            if len(din_state.data) >= 4:
                raw_bytes = din_state.data[0:2]
                din3 = (raw_bytes[0] >> 1) & 1
                din2 = (raw_bytes[0] >> 2) & 1
                return (din2, din3, raw_bytes)
        return (-1,-1)
        # read 