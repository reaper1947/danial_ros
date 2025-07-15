import can
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler # Additional 
import tf2_ros

import json
import os

import struct
import math

class Ros2MotorNode(Node):

    def __init__(self):
        super().__init__('daniel_motor_node_ros2')
        
        # Logging
        self.logger = self.get_logger()

        # Time
        self.last_time = self.get_clock().now()

        # Robot pose
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0

        # Initial Speed
        self.v_left = 0.0
        self.v_right = 0.0

        # Parameter Declaration
        self.json_file = "motor_param.json"
        self.param_data = self.load_from_json(self.json_file)
        for key, value in self.param_data.items():
          self.declare_parameter(key, value)  
          setattr(self, key, self.get_parameter(key).value)

        # Subscription
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.mode_sub = self.create_subscription(Int32, '/robot_mode', self.mode_callback, 10)

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
        self.merge_flow()

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
            self.odom_pub.destroy()
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
            max_spd_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=[0x23, 0x80, 0x60, 0x00] + list(max_speed_data), is_extended_id=False)
            self.bus.send(max_spd_msg)
            max_spd_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=[0x23, 0x80, 0x60, 0x00] + list(max_speed_data), is_extended_id=False)
            self.bus.send(max_spd_msg)
        elif param_name == "robot_max_spd":
            pass # ?
        elif param_name == "pulse_per_rev":
            pass # ?

    def cmd_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.v_left  = (linear_x - (self.wheel_seperate_distance / 2.0) * angular_z) / (self.wheel_diameter / 2.0)
        self.v_right = (linear_x + (self.wheel_seperate_distance / 2.0) * angular_z) / (self.wheel_diameter / 2.0)

    def mode_callback(self, msg):
        self.robot_mode = msg.data

    def send_start_msg(self):
        
        # Send check alarm message for motor 1
        check_alarm_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=[0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        if not hasattr(self, 'alarm_1'):
            self.bus.send(check_alarm_msg)
            self.alarm_1 = self.bus.recv(timeout=0.1)

        # Send check alarm message for motor 2
        check_alarm_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=[0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        if not hasattr(self, 'alarm_2'):
            self.bus.send(check_alarm_msg)
            self.alarm_2 = self.bus.recv(timeout=0.1)

        self.current_state = 'alarm_check'
        
    # --------- Check Motor Alarm -----------

    def motor_alarm_check(self):

        if self.alarm_1 == None or self.alarm_2 == None:
            self.logger.error("Frame not found")
            self.disable_motor()

        if self.alarm_1.arbitration_id == 0x581 and self.alarm_2.arbitration_id == 0x582:
            command_byte_1 = self.alarm_1.data[0]
            command_byte_2 = self.alarm_2.data[0]
            if command_byte_1 == 0x60 and command_byte_2 == 0x60:
                self.logger.info("No problem")
                self.current_state = 'auto_mode_check'
            else:
                if command_byte_1 == 0x80:
                    self.logger.error(f"Motor 1 : Abort")
                elif command_byte_1 == 0x60:
                    self.logger.info(f"Motor 1 : Clear")
                else:
                    self.logger.error(f"Motor 1 : Unknown command")
                if command_byte_2 == 0x80:
                    self.logger.error(f"Motor 2 : Abort")
                elif command_byte_2 == 0x60:
                    self.logger.info(f"Motor 2 : Clear")
                else:
                    self.logger.error(f"Motor 2 : Unknown command")
                self.disable_motor()
                
    def disable_motor(self):
        reset_msg = can.Message(arbitration_id=0x00, data=[0x81, 0x00], is_extended_id=False)
        self.bus.send(reset_msg)
        # self.current_state = 'cal_odom'
        self.current_state = 'init'

    # ------------ Check Auto Mode -------

    def auto_mode_check(self):
        if self.current_mode != 'Auto':
            self.logger.error(f"Current mode is {self.current_mode}, not Auto Mode")
            self.disable_motor()
        else:
            self.logger.info("Auto mode on")
            self.current_state = 'robot_mode_check'

    # ----------- Check Nav ------------

    def check_robot_mode(self):
        if self.robot_mode != 2 :
            self.logger.error(f"Current robot mode is {self.robot_mode}, not 2")
            self.disable_motor()
        else:
            self.logger.info("Robot mode 2")
            self.current_state = 'enable_check'

    # ------------ Check Motor Enable ----

    def check_motor_enable(self):

        # Send check status message for motor 1
        check_status_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=[0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.bus.send(check_status_msg)
        self.status_1 = self.bus.recv(timeout=0.1)
        
        # Send check status message for motor 2
        check_status_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=[0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.bus.send(check_status_msg)
        self.status_2 = self.bus.recv(timeout=0.1)

        if self.status_1.arbitration_id == 0x581 and self.status_2.arbitration_id == 0x582:
            status_byte_1 = self.status_1.data[4]
            status_byte_2 = self.status_2.data[4]
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
        rpm = (v / (3.1416 * self.wheel_diameter)) * 60
        rpm = rpm / self.gear_ratio
        rpm = min(self.motor_max_spd, rpm)
        self.logger.info(f"RPM = {rpm}")
        return rpm

    def rpm_to_data_frame(self, rpm):
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

    def spin_motor(self):
        rpm_l = self.cmd_to_rpm(self.v_left)
        rpm_r = self.cmd_to_rpm(self.v_right)
        data_frame_l = self.rpm_to_data_frame(rpm_l)
        data_frame_r = self.rpm_to_data_frame(rpm_r)
        left_motor_msg = can.Message(arbitration_id=0x600 | self.motor_id_left, data=data_frame_l, is_extended_id=False)
        right_motor_msg = can.Message(arbitration_id=0x600 | self.motor_id_right, data=data_frame_r, is_extended_id=False)
        self.bus.send(left_motor_msg)
        self.alarm_1 = self.bus.recv(timeout=0.1)
        self.bus.send(right_motor_msg)
        self.alarm_2 = self.bus.recv(timeout=0.1)
        # self.current_state = 'cal_odom'
        self.current_state = 'alarm_check'

    # ---------- Cal Odom ---------------

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        left_ticks, right_ticks = self.get_encoder_ticks() ####################

        d_left = math.pi * self.wheel_diameter * ((left_ticks - self.last_left_ticks) / self.pulse_per_rev)
        d_right = math.pi * self.wheel_diameter * ((right_ticks - self.last_right_ticks) / self.pulse_per_rev)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_seperate_distance

        self.x += d_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += d_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

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
        odom_msg.pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, self.theta))

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

    def merge_flow(self):

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

        while True:
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


def main(args=None):
    rclpy.init(args=args)

    motor_node = Ros2MotorNode()

    rclpy.spin(motor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()