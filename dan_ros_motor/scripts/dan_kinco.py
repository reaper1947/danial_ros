#!/usr/bin/env python3

import can
# import time
import struct
import rclpy
from geometry_msgs.msg import Twist

def init_motor():
    global bus
    bus = can.Bus(interface='socketcan', channel='can0')
    init_sequence = [
    [0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00], # 0x2B 0x200F 0x00 0x00000000 (set async control)
    [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00], # 0x2F 0x6060 0x00 0x00000003 (6060:0-Operation Mode / 3-Speed Mode)
    # [0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00],
    # [0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00],
    # [0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00],
    # [0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00],
    [0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], # Shutdown # 0x2B 0x6040 0x00 0x00000006 (6040:0-Controlword / 6(0000000000000110)-Shutdown Transition)
    [0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00], # Switch ON # 0x2B 0x6040 0x00 0x00000007 (6040:0-Controlword / 6(0000000000000111)-Switch On)
    [0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00], # Switch ON + Enable Operation # 0x2B 0x6040 0x00 0x0000000F (6040:0-Controlword / 6(0000000000001111)-Switch ON+Enable oper)
    ]
    try:
        for command in init_sequence:
            l_msg = can.Message(arbitration_id=0x601, data=command, is_extended_id=False)
            r_msg = can.Message(arbitration_id=0x602, data=command, is_extended_id=False)
            bus.send(l_msg)
            bus.send(r_msg)
            
        print("init successfull")
    except can.CanError as e:
        print("ERROR : {e}")

def cal_data_frame(rpm):
    mystery_coeff = 3.15
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

def run_motor():
    c = input("RUN")
    target = 500
    # data_frame = cal_data_frame(target)
    # print(data_frame)
    # print(data_frame)
    
    # motor_msg = can.Message(arbitration_id=0x601, data=data_frame, is_extended_id=False)

    # bus.send(motor_msg)

    left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(0), is_extended_id=False)
    right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(0), is_extended_id=False)
    while 1:
        if c == 'q':
            left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(0), is_extended_id=False)
            right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(0), is_extended_id=False)
            bus.send(left_motor_msg)
            bus.send(right_motor_msg)
            break
        elif c == 'u':
            left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(target), is_extended_id=False)
            bus.send(left_motor_msg)
        elif c == 'j':
            left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(0), is_extended_id=False)
            bus.send(left_motor_msg)
        elif c == 'm':
            left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(-target), is_extended_id=False)
            bus.send(left_motor_msg)
        elif c == 'o':
            right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(-target), is_extended_id=False)
            bus.send(right_motor_msg)
        elif c == 'l':
            right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(0), is_extended_id=False)
            bus.send(right_motor_msg)
        elif c == '.':
            right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(target), is_extended_id=False)
            bus.send(right_motor_msg)
        c = input("RUN")
        
def stop_motor():
    left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(0), is_extended_id=False)
    right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(0), is_extended_id=False)
    bus.send(left_motor_msg)
    bus.send(right_motor_msg)

init_motor()
try:
    run_motor()
except :
    stop_motor()