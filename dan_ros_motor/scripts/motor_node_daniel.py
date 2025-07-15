#!/usr/bin/env python3

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


import json
import os

import struct
import math
import threading

class Ros2MotorNode(Node):

    def __init__(self):
        super().__init__('daniel_motor_node_ros2')

        # Lock
        self.lock = threading.Lock()
        
        # Logging
        self.logger = self.get_logger()

        # Time
        self.last_time = self.get_clock().now()
        self.rate = self.create_rate(1)

        # 50 Hz Timer
        self.create_timer(0.02, self.flow_state_machine)

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Robot last state 
        self.last_motor_R = 0
        self.last_motor_L = 0

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
        self.mode_sub = self.create_subscription(Int32, '/robot_mode', self.mode_callback, 10)
        self.masteron_sub = self.create_subscription(Bool, '/emergency_io', self.masteron_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10)
        #self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) #####

        self.left_ticks = 0
        self.right_ticks = 0

        # Parameter callback when update
        self.add_on_set_parameters_callback(self.param_reconfig)

        # Bus
        self.bus = can.Bus(interface='socketcan', channel=self.can_dev, bitrate=self.baudrate)

        self.bus.set_filters([
            {"can_id": 0x581, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x582, "can_mask": 0x7FF, "extended": False}
        ])

        # Master On 
        self.last_btn_state = True # At start E-stop must not presss
        self.freeze_system = False

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

        # Flow
        self.flow_state_machine()

    # ----- INIT STATE ------------

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

    def cmd_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.v_left  = (linear_x - (self.wheel_seperate_distance / 2.0) * angular_z) / (self.wheel_diameter / 2.0)
        self.v_right = (linear_x + (self.wheel_seperate_distance / 2.0) * angular_z) / (self.wheel_diameter / 2.0)

    def mode_callback(self, msg):
        self.robot_mode = msg.data

    def masteron_callback(self, msg):
        # print(msg.data)
        if msg.data != self.last_btn_state: # There is a change in E-stop
            if msg.data == False: # E-stop press
                self.current_state = 'emergency_state'
                self.stop_motor(self.motor_id_right)
                self.control_msg_r = self.bus.recv(timeout=0.1)
                self.stop_motor(self.motor_id_left)
                self.control_msg_l = self.bus.recv(timeout=0.1)

            elif msg.data == True: # E-stop release
                
                self.current_state = 'recovery_state'

        self.last_btn_state = msg.data

    # ----------- Emergency State ----------------------

    def emergency_press(self):
        self.v_left = 0.0
        self.v_right = 0.0

    def emergency_release(self):
        stop_msg = can.Message(arbitration_id=0x00 , data=[0x81, 0x00], is_extended_id=False)
        self.bus.send(stop_msg)
        resume_msg = can.Message(arbitration_id=0x00, data=[0x01, 0x00], is_extended_id=False)
        self.bus.send(resume_msg) 
        self.rate.sleep()
        self.current_state = 'init'

    # ------------------- Init State -----------------------

    def send_start_msg(self):
        
        # Send check alarm message for motor 1 (right)
        check_alarm_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=[0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        # if not hasattr(self, 'alarm_1'):
        self.bus.send(check_alarm_msg)
        self.control_msg_r = self.bus.recv(timeout=0.1)

        # Send check alarm message for motor 2 (left)
        check_alarm_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=[0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        # if not hasattr(self, 'alarm_2'):
        self.bus.send(check_alarm_msg)
        self.control_msg_l = self.bus.recv(timeout=0.1)

        self.current_state = 'alarm_check'
        
    # --------- Check Motor Alarm -----------

    def motor_alarm_check(self):

        if self.control_msg_r == None or self.control_msg_l == None:
            self.logger.error("Frame not found")
            self.disable_motor()

        if self.control_msg_r.arbitration_id == (0x580 | self.motor_id_right) and self.control_msg_l.arbitration_id == (0x580 | self.motor_id_left):
            command_byte_1 = self.control_msg_r.data[0]
            command_byte_2 = self.control_msg_l.data[0]

            if command_byte_1 == 0x60 and command_byte_2 == 0x60:
                # self.logger.info("No problem")
                self.current_state = 'auto_mode_check'
            else:
                self.disable_motor()
        elif self.control_msg_l.arbitration_id == (0x580 | self.motor_id_right) and self.control_msg_r.arbitration_id == (0x580 | self.motor_id_left):
            self.logger.error("swap id")
        else :
            self.logger.error("something wrong")
                
    def disable_motor(self):
        self.stop_motor(self.motor_id_right)
        self.control_msg_r = self.bus.recv(timeout=0.1)
        self.stop_motor(self.motor_id_left)
        self.control_msg_l = self.bus.recv(timeout=0.1)
        reset_msg = can.Message(arbitration_id=0x00, data=[0x81, 0x00], is_extended_id=False)
        self.bus.send(reset_msg)
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

        # Send check status message for motor 1
        check_status_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=[0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.bus.send(check_status_msg)
        self.status_msg_r = self.bus.recv(timeout=0.1)
        
        # self.rate.sleep()
        
        # Send check status message for motor 2
        check_status_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=[0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.bus.send(check_status_msg)
        self.status_msg_l = self.bus.recv(timeout=0.1)

        if self.status_msg_r.arbitration_id == (0x580 | self.motor_id_right) and self.status_msg_l.arbitration_id == (0x580 | self.motor_id_left):
            status_byte_1 = self.status_msg_r.data[4]
            status_byte_2 = self.status_msg_l.data[4]
            if status_byte_1 & 2 == False or status_byte_2 & 2 == False:
                self.logger.error(f"Motor 1 is {status_byte_1 & 2} , Motor 2 is {status_byte_2 & 2}")
                self.enable_motor()
            else:
                self.current_state = 'spinning'
                     
    def enable_motor(self):
        start_msg = can.Message(arbitration_id=0x00, data=[0x01, 0x00], is_extended_id=False)
        self.bus.send(start_msg)

    # ------------ Spin Motor ------------

    def cmd_to_rpm(self, v):
        rpm = (v / (math.pi * self.wheel_diameter)) * 60
        rpm = rpm / self.gear_ratio
        rpm = min(self.motor_max_spd, rpm)
        self.logger.info(f"RPM = {rpm}")
        return rpm

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

    def spin_motor(self):
        self.rpm_l = self.cmd_to_rpm(self.v_left)
        self.rpm_r = self.cmd_to_rpm(self.v_right)
        data_frame_l = self.rpm_to_data_frame(-self.rpm_l)
        data_frame_r = self.rpm_to_data_frame(self.rpm_r)
        left_motor_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=data_frame_l, is_extended_id=False)
        right_motor_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=data_frame_r, is_extended_id=False)
        self.bus.send(right_motor_msg)
        self.control_msg_r = self.bus.recv(timeout=0.1)
        # self.rate.sleep()
        self.bus.send(left_motor_msg)
        self.control_msg_l = self.bus.recv(timeout=0.1)
        self.current_state = 'cal_odom'
        # self.current_state = 'alarm_check'

    # ---------- Cal Odom ---------------

    def get_encoder_ticks(self, rpm, ppr, t):
        tick = (rpm*ppr*t)/60.0
        return tick # rpm * ppr = ppm , ppm / 60 = pps , pps * t = p . p = t * 1/60 *rpm * ppr

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        left_ticks = self.get_encoder_ticks(self.rpm_l, self.pulse_per_rev, dt) 
        right_ticks = self.get_encoder_ticks(self.rpm_r, self.pulse_per_rev, dt) 

        d_left = math.pi * self.wheel_diameter * (left_ticks / self.pulse_per_rev)
        d_right = math.pi * self.wheel_diameter * (right_ticks / self.pulse_per_rev)

        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_seperate_distance

        self.x += d_center * math.cos(self.theta + delta_theta / 2.0) * dt
        self.y += d_center * math.sin(self.theta + delta_theta / 2.0) * dt
        self.theta += delta_theta * dt

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

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
        odom_msg.twist.twist.linear.x = d_center / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom_msg)

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def cal_odom(self):
        self.update_odometry()
        self.current_state = 'alarm_check'

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

        # self.logger.info(self.current_state)
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
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()    
    

    # rclpy.spin(motor_node)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # motor_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()