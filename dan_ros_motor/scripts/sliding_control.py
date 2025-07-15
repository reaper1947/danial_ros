#!/usr/bin/env python3

import os
os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

import can
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler # Additional
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from next2_msgs.msg import RobotMode
from std_msgs.msg import Float64


import json
import sys
import csv
from datetime import datetime
import time

import struct
import math
from math import sin, cos, pi
import threading

class Ros2MotorNode(Node):

    def __init__(self):
        super().__init__('daniel_motor_node_ros2')

        os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

        # Lock
        self.lock = threading.Lock()

        # Logging
        self.logger = self.get_logger()

        # Time
        self.last_time = self.get_clock().now()

        # Timer
        self.rate = self.create_rate(1)
        self.create_timer(0.01, self.flow_state_machine)
        self.create_timer(0.01, self.pub_odom)
        self.create_timer(1, self.check_heartbeat)

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_twist_x = 0.0
        self.odom_twist_y = 0.0
        self.odom_twist_z = 0.0

        # Initial Speed
        self.v_left = 0.0
        self.v_right = 0.0

        # 601 right motor
        # 602 left motor

        # Parameter Declaration

        package_share_directory = get_package_share_directory('dan_ros_motor')
        json_file_path = os.path.join(package_share_directory, 'config/motor_param.json')

        self.json_file = json_file_path
        self.param_data = self.load_from_json(self.json_file)
        for key, value in self.param_data.items():
          self.declare_parameter(key, value)
          setattr(self, key, self.get_parameter(key).value)

        # Subscription
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.mode_sub = self.create_subscription(RobotMode, '/robot_mode', self.mode_callback, 10)
        self.masteron_sub = self.create_subscription(Bool, '/allowed_master_on', self.masteron_callback, 10) # allowed_master_on
        self.control_input_sub = self.create_subscription(Float64, '/control_input', self.control_input_callback, 10) # Add control input subscriber

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10)
        self.current_vel_pub = self.create_publisher(Float64, '/current_velocity', 10) # Add current velocity publisher
        #self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) #####

        # Parameter callback when update
        self.add_on_set_parameters_callback(self.param_reconfig)

        # Bus
        self.bus = can.ThreadSafeBus(interface='socketcan', channel=self.can_dev, bitrate=self.baudrate)
        self.bus_id = {
            "MOTOR_R":0x181,
            "MOTOR_L":0x182,
            "HEARTBEAT_R":0x701,
            "HEARTBEAT_L":0x702
        }

        # Master On
        self.last_btn_state = True # At starting assume master on is press (after starting phase)

        # Starting phase
        self.starting = True

        # Robot Mode
        self.robot_mode = 2

        # Mode
        self.all_mode = [
            'Auto',
            'Manual',
            'SemiAuto',
            'Stop'
        ]
        self.current_mode = 'Auto'

        # Motor message
        self.control_msg_r = None
        self.control_msg_l = None
        self.status_msg_r = None
        self.status_msg_l = None

        # Flow state
        self.flow_state = [
            'init',
            'alarm_check',
            'auto_mode_check',
            'robot_mode_check',
            'enable_check',
            'spinning',
            'cal_odom'
        ]

        self.current_state = 'init'

        # # Set ASYNC

        # l_msg = can.Message(arbitration_id=0x601, data=[0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        # self.bus.send(l_msg)
        # r_msg = can.Message(arbitration_id=0x602, data=[0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        # self.bus.send(r_msg)

        # Heartbeat Pulse
        self.is_heartbeat = False
        self.last_pulse = False

        # CSV Log
        self.filepath = '/home/next2/next_log/device/base_control/log.csv'

        # Flow
        self.send_start_msg()
        self.flow_state_machine()

    # ----- pre start ------------

    def load_from_json(self, file):
        if not os.path.exists(file):
            self.logger.error(f'No parameter file found')
            return 0
        with open(self.json_file, 'r') as f:
            data = json.load(f)
        return data

    def param_reconfig(self, params):
        with open(self.json_file, 'r') as f:
            exist_param = json.load(f)
        for param in params:
            data = {param.name: param.value}
            setattr(self, param.name, param.value)
            self.param_reconfig_callback(param.name)
            exist_param.update(data)
        with open(self.json_file, 'w') as f:
            json.dump(exist_param, f,indent=2, separators=(',', ': '))
        return SetParametersResult(successful=True)

    def param_reconfig_callback(self, name):
        param_name = name
        if param_name == "odom_topic_name":
            self.destroy_publisher(self.odom_pub)
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10)
        elif param_name == "tf_topic_name":
            pass # for now
        elif param_name == "broadcast_enable":
            pass # ?
        elif param_name == "can_dev" or param_name == "baudrate":
            self.bus.shutdown()
            self.bus = can.Bus(interface='socketcan', channel=self.can_dev, bitrate=self.baudrate)
        elif param_name == "motor_start":
            pass # ?
        elif param_name == "motor_disable":
            pass # ?
        elif param_name == "manual_mode":
            pass # ?
        elif param_name == "auto_mode":
            pass # ?
        elif param_name == "motor_max_spd":
            max_speed_data = self.motor_max_spd.to_bytes(4, byteorder='little', signed=False)
            max_spd_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=[0x23, 0x80, 0x60, 0x00] + list(max_speed_data), is_extended_id=False)
            self.bus.send(max_spd_msg)
            self.control_msg_r = self.bus.recv(timeout=0.1)
            # self.rate.sleep()
            max_spd_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=[0x23, 0x80, 0x60, 0x00] + list(max_speed_data), is_extended_id=False)
            self.bus.send(max_spd_msg)
            self.control_msg_l = self.bus.recv(timeout=0.1)
        elif param_name == "robot_max_spd":
            pass # ?
        elif param_name == "pulse_per_rev":
            pass # ?

    def stop_motor(self, motor_id):
        stop_msg = can.Message(arbitration_id=0x600 | motor_id, data=[0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.bus.send(stop_msg)

    def control_input_callback(self, msg):
        """Callback for receiving control input from sliding controller"""
        # Store the control input
        self.control_input = msg.data
        self.get_logger().debug(f'Received control input: {self.control_input}')

    def cmd_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate base velocities
        self.v_left  = (linear_x - (self.wheel_seperate_distance / 2.0) * angular_z)
        self.v_right = (linear_x + (self.wheel_seperate_distance / 2.0) * angular_z)

        # Apply control input from sliding controller if available
        if hasattr(self, 'control_input'):
            # Scale the control input to motor velocities
            control_scale = 0.5  # Adjust this value based on your motor characteristics
            self.v_left *= (1.0 + self.control_input * control_scale)
            self.v_right *= (1.0 + self.control_input * control_scale)

    def mode_callback(self, msg):
        self.robot_mode = msg.robot_mode
        # pass

    def masteron_callback(self, msg): ##############
        # print(msg.data)

        if self.starting == True:
            if msg.data == True: # master on is press
                self.starting = False
        else:
            if msg.data != self.last_btn_state: # There is a change in E-stop
                if msg.data == False: # E-stop press
                    self.current_state = 'emergency_state'

                elif msg.data == True: # E-stop release

                    self.current_state = 'recovery_state'

            self.last_btn_state = msg.data

    def motor_transition(self, motor_id, transition):
        # CIA 402
        if transition == 'Shutdown':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False))
        elif transition == 'Switchon':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00], is_extended_id=False))
        elif transition == 'Enable Operation':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00], is_extended_id=False))
        # DIN1 Unmap
        elif transition == 'DIN1_unmap':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2f, 0x10, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00], is_extended_id=False))
        # Enable Heartbeat
        elif transition == 'Enable Heartbeat':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2b, 0x17, 0x10, 0x00, 0x0a, 0x00, 0x00, 0x00], is_extended_id=False))
        # Enable CAN
        elif transition == 'Enable CAN':
            self.bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00], is_extended_id=False))
        # Set default opeation mode to profile speed
        elif transition == 'Set Profile Speed Mode':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2f, 0x20, 0x20, 0x0e, 0x03, 0x00, 0x00, 0x00], is_extended_id=False))
        # Set default opeation mode to raw speed
        elif transition == 'Set Raw Speed Mode':
            self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=[0x2f, 0x20, 0x20, 0x0e, 0xfd, 0xff, 0xff, 0xff], is_extended_id=False))
        else:
            self.logger.error("UNKNOWN TRANSITION")
            sys.exit("UNKNOWN TRANSITION")

    def read_id(self, id):
        start = time.monotonic()
        while time.monotonic() - start < 1:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            if msg.arbitration_id in id:
                return msg
        return None  # timeout

    def check_heartbeat(self):

        if self.is_heartbeat != self.last_pulse:
              # Ensure directory exists
            os.makedirs(os.path.dirname(self.filepath), exist_ok=True)

            # Check if file exists to know if header is needed
            file_exists = os.path.isfile(self.filepath)
            now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            if self.is_heartbeat == False:
                self.logger.error("HEARTBEAT NOT OPERATING")
                with open(self.filepath, mode='a', newline='\n') as file:
                    writer = csv.writer(file)
                    if not file_exists:
                        # Write header only once
                        writer.writerow(['Timestamp', 'Device Status'])
                    writer.writerow([now, "disconnect"])
                self.bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00], is_extended_id=False))
            else:

                with open(self.filepath, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    if not file_exists:
                        # Write header only once
                        writer.writerow(['Timestamp', 'Device Status'])
                    writer.writerow([now, "reconnect"])
        self.last_pulse = self.is_heartbeat


    # ----------- Emergency State ----------------------

    def emergency_press(self):
        self.motor_transition(self.motor_id_right, 'Shutdown')
        self.motor_transition(self.motor_id_left, 'Shutdown')
        self.stop_motor(self.motor_id_right)
        self.stop_motor(self.motor_id_left)
        self.v_left = 0.0
        self.v_right = 0.0
        # while self.current_state == 'emergency_state':
        #     pass

    def emergency_release(self):
        self.rate.sleep()
        self.motor_transition(self.motor_id_right, 'Switchon')
        self.motor_transition(self.motor_id_right, 'Enable Operation')
        self.motor_transition(self.motor_id_left, 'Switchon')
        self.motor_transition(self.motor_id_left, 'Enable Operation')
        self.current_state = 'alarm_check'

    # ------------------- Init State -----------------------

    def motor_init(self, motor_id):
        self.motor_transition(motor_id, 'DIN1_unmap')
        time.sleep(0.1)
        self.motor_transition(motor_id, 'Enable Heartbeat')
        time.sleep(0.1)
        self.motor_transition(motor_id, 'Shutdown')
        time.sleep(0.1)
        self.motor_transition(motor_id, 'Switchon')
        time.sleep(0.1)
        self.motor_transition(motor_id, 'Enable Operation')
        time.sleep(0.1)
        # self.motor_transition(motor_id, 'Set Raw Speed Mode')
        self.motor_transition(motor_id, 'Set Profile Speed Mode')
        time.sleep(0.1)
        self.set_accel_decel(26214,26214,motor_id)
        time.sleep(0.1)

    def send_start_msg(self):

        # Enable CAN
        self.motor_transition("0", 'Enable CAN')

        self.motor_init(self.motor_id_right)
        self.motor_init(self.motor_id_left)

        # # Set DIN1 to none
        # self.motor_transition(self.motor_id_right, 'DIN1_unmap')
        # self.motor_transition(self.motor_id_left, 'DIN1_unmap')

        # # Enable Heartbeat
        # self.motor_transition(self.motor_id_right, 'Enable Heartbeat')
        # self.motor_transition(self.motor_id_left, 'Enable Heartbeat')

        # # CIA 402
        # self.motor_transition(self.motor_id_right, 'Shutdown')
        # self.motor_transition(self.motor_id_right, 'Switchon')
        # self.motor_transition(self.motor_id_right, 'Enable Operation')
        # self.motor_transition(self.motor_id_left, 'Shutdown')
        # self.motor_transition(self.motor_id_left, 'Switchon')
        # self.motor_transition(self.motor_id_left, 'Enable Operation')

        # # Set Profile Speed Mode
        # self.motor_transition(self.motor_id_right, 'Set Profile Speed Mode')
        # self.motor_transition(self.motor_id_left, 'Set Profile Speed Mode')
        # # self.motor_transition(self.motor_id_right, 'Set Raw Speed Mode')
        # # self.motor_transition(self.motor_id_left, 'Set Raw Speed Mode')

        # Set Motor Accel Decel
        # self.set_accel_decel(4000,4000,self.motor_id_right)
        # self.set_accel_decel(4000,4000,self.motor_id_left)

        self.current_state = 'alarm_check'

    # --------- Check Motor Alarm -----------

    def motor_alarm_check(self): ############

        self.control_msg_r = self.read_id([0x181])
        self.control_msg_l = self.read_id([0x182])

        try:
            if self.control_msg_r.arbitration_id == 0x181 and self.control_msg_l.arbitration_id == 0x182:
                self.current_state = 'auto_mode_check'
            else :
                self.logger.error("something wrong")
                self.disable_motor()
        except:
            self.logger.error("CAN down?")


    def disable_motor(self):
        self.motor_transition(self.motor_id_right, 'Shutdown')
        self.motor_transition(self.motor_id_left, 'Shutdown')
        self.current_state = 'cal_odom'
        # self.current_state = 'init'

    # ------------ Check Auto Mode -------

    def auto_mode_check(self):
        if self.current_mode != 'Auto':
            self.logger.error(f"Current mode is {self.current_mode}, not Auto Mode")
            self.disable_motor()
        else:
            # self.logger.info("Auto mode on")
            self.current_state = 'robot_mode_check'

    # ----------- Check Nav ------------

    def check_robot_mode(self):
        if self.robot_mode != 2 :
            self.logger.error(f"Current robot mode is {self.robot_mode}, not 2")
            self.disable_motor()
        else:
            # self.logger.info("Robot mode 2")
            self.current_state = 'enable_check'

    # ------------ Check Motor Enable ----

    def check_motor_enable(self):

        self.status_msg_r = self.read_id([0x181])
        self.status_msg_l = self.read_id([0x182])

        try:
            if self.status_msg_r.arbitration_id == 0x181 and self.status_msg_l.arbitration_id == 0x182:
                status_byte_1 = self.status_msg_r.data[4]
                status_byte_2 = self.status_msg_l.data[4]

                if status_byte_1 & 2 == False or status_byte_2 & 2 == False:
                    self.logger.error(f"Motor 1 is {status_byte_1 & 2} , Motor 2 is {status_byte_2 & 2}")
                    self.enable_motor()
                else:
                    self.current_state = 'spinning'
        except:
            # self.logger.error("CAN down?")
            return 0

    def enable_motor(self):
        self.motor_transition(self.motor_id_right, 'Switchon')
        self.motor_transition(self.motor_id_right, 'Enable Operation')
        self.motor_transition(self.motor_id_left, 'Switchon')
        self.motor_transition(self.motor_id_left, 'Enable Operation')

    # ------------ Spin Motor ------------

    def set_accel_decel(self, acc, dec, motor_id):
        value_hex = struct.pack('<i', int(acc))
        data_frame = [0x23, 0x83, 0x60, 0x00, value_hex[0], value_hex[1], value_hex[2], value_hex[3]]
        self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=data_frame, is_extended_id=False))
        value_hex = struct.pack('<i', int(dec))
        data_frame = [0x23, 0x84, 0x60, 0x00, value_hex[0], value_hex[1], value_hex[2], value_hex[3]]
        self.bus.send(can.Message(arbitration_id=0x600 | motor_id, data=data_frame, is_extended_id=False))

    def cmd_to_rpm(self, v):
        rpm = (v / (math.pi * self.wheel_diameter)) * 60
        rpm = rpm * self.gear_ratio
        rpm = min(self.motor_max_spd, rpm)
        # self.logger.info(f"RPM_set = {rpm}")
        return rpm

    def rpm_to_data_frame(self, rpm):
        mystery_coeff = math.pi
        objects_DEC = (int(rpm) * 512 * 2500 * 4) / 1875
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

    def spin_motor(self):
        rpm_l_set = self.cmd_to_rpm(self.v_left)
        rpm_r_set = self.cmd_to_rpm(self.v_right)
        # self.logger.info(f"RPM_set r = {self.v_right}")
        # self.logger.info(f"RPM_set l = {self.v_left}")
        data_frame_l = self.rpm_to_data_frame(-1.0*rpm_l_set)
        data_frame_r = self.rpm_to_data_frame(rpm_r_set)
        left_motor_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=data_frame_l, is_extended_id=False)
        right_motor_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=data_frame_r, is_extended_id=False)

        self.bus.send(right_motor_msg)
        self.control_msg_r = self.read_id([0x181])

        self.bus.send(left_motor_msg)
        self.control_msg_l = self.read_id([0x182])
        self.current_state = 'cal_odom'
        # self.current_state = 'alarm_check'

    # ---------- Cal Odom ---------------

    def get_current_rpm(self):
        SCALING_FACTOR = 1875 / (512 * 10000 )

        motor_r = self.read_id([0x181])
        if motor_r:
            if len(motor_r.data) >= 6:
                raw_bytes = motor_r.data[0:4]
                raw_int = int.from_bytes(raw_bytes, byteorder='little', signed=True)
                self.rpm_r = raw_int * SCALING_FACTOR / self.gear_ratio
                self.logger.info(f"RPM_get r = {self.rpm_r}")
            else:
                self.logger.error("Error data frame")

        motor_l = self.read_id([0x182])
        if motor_l:
            if len(motor_l.data) >= 6:
                raw_bytes = motor_l.data[0:4]
                raw_int = int.from_bytes(raw_bytes, byteorder='little', signed=True)
                self.rpm_l = raw_int * SCALING_FACTOR * -1.0 / self.gear_ratio
                self.logger.info(f"RPM_get l = {self.rpm_l}")
            else:
                self.logger.error("Error data frame")

        # Calculate and publish current velocity
        current_vel = (self.rpm_r - self.rpm_l) * (math.pi * self.wheel_diameter / 60.0) / 2.0
        vel_msg = Float64()
        vel_msg.data = current_vel
        self.current_vel_pub.publish(vel_msg)

    def update_odometry(self):

        self.get_current_rpm()

        vel_l = self.rpm_l * (math.pi*self.wheel_diameter/60.00)
        vel_r = self.rpm_r * (math.pi*self.wheel_diameter/60.00)

        vel_x = (vel_l + vel_r) / 2.0
        ang_z = (vel_r - vel_l) / self.wheel_seperate_distance

        vel_y = 0.0

        vel_x = 0.0 if abs(vel_x) <= 0.06 else vel_x
        ang_z = 0.0 if abs(ang_z) <= 0.1 else ang_z

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        d_x = (vel_x * cos(self.theta) - vel_y * sin(self.theta)) * dt
        d_y = (vel_x * sin(self.theta) + vel_y * cos(self.theta)) * dt

        self.x += d_x
        self.y += d_y
        self.theta += ang_z * dt

        self.odom_twist_x = vel_x
        self.odom_twist_y = vel_y
        self.odom_twist_z = ang_z

    def cal_odom(self):
        self.update_odometry()
        self.current_state = 'alarm_check'

    def pub_odom(self):

        current_time = self.get_clock().now()

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat_msg = Quaternion()
        q = quaternion_from_euler(0, 0, self.theta)
        quat_msg.x = q[0]
        quat_msg.y = q[1]
        quat_msg.z = q[2]
        quat_msg.w = q[3]

        odom_msg.pose.pose.orientation = quat_msg

        # Simple velocity estimation
        odom_msg.twist.twist.linear.x = self.odom_twist_x
        odom_msg.twist.twist.linear.y = self.odom_twist_y
        odom_msg.twist.twist.angular.z = self.odom_twist_z
        self.odom_pub.publish(odom_msg)

        # Publish TF
        if self.pub_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    # ---------------------------------

    def flow_state_machine(self):

        # # Flow state
        # self.flow_state = [
        #     'init',
        #     'alarm_check',
        #     'auto_mode_check',
        #     'robot_mode_check',
        #     'enable_check',
        #     'spinning',
        #     'cal_odom'
        # ]

        try:
            pulse_get = self.read_id([0x701,0x702])
            self.is_heartbeat = pulse_get.data[0] == 0x05
        except Exception as e:
            self.logger.error(f"except {e}")

        # self.logger.info(str(self.is_heartbeat))
        if self.starting == False and self.is_heartbeat == True:
            self.logger.info(self.current_state)
            if self.current_state == 'init':
                self.send_start_msg()
            elif self.current_state == 'alarm_check':
                self.motor_alarm_check()
            elif self.current_state == 'auto_mode_check':
                self.auto_mode_check()
            elif self.current_state == 'robot_mode_check':
                self.check_robot_mode()
            elif self.current_state == 'enable_check':
                self.check_motor_enable()
            elif self.current_state == 'spinning':
                self.spin_motor()
            elif self.current_state == 'cal_odom':
                self.cal_odom()
            elif self.current_state == 'emergency_state':
                self.emergency_press()
            elif self.current_state == 'recovery_state':
                self.emergency_release()

def main(args=None):
    rclpy.init(args=args)

    motor_node = Ros2MotorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(motor_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        motor_node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        motor_node.stop_motor(motor_node.motor_id_right)
        motor_node.stop_motor(motor_node.motor_id_left)
        motor_node.bus.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


    # rclpy.spin(motor_node)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # motor_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()