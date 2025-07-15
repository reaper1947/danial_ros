#!/usr/bin/env python3

import os
import json
import math
from math import floor

from indicator_driver import DMXDriver

import rclpy
from rclpy.node import Node
from next2_msgs.msg import RobotMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from ament_index_python.packages import get_package_share_directory

RUNNING = 2
EMERGENCY = 5 
IDLE = 1
NO_COLOR = 0 
BATTERY_COLORS = 22
BOOTING = 11
CHARGING = 15
MANUAL_CHARGE = 19
ERROR = 14
PAUSE = 23

class DMX512Node(Node):
    def __init__(self):
        super().__init__('dmx_512')

        # Color
        self.RUNNING = (0,255,0) # green
        self.EMERGENCY = (255,0,0) # red
        self.IDLE = (0,0,255) # blue
        self.NO_COLOR = (0,0,0) # none
        self.BATTERY_COLORS = [(229,31,31), (242,161,52), 	(247,227,121), (187,219,68)] #green shade from 0% to full
        self.BOOTING = (128,0,128) # yellow
        self.CHARGING = (255,255,0) # pink
        self.PAUSE = (255,0,255)


        package_share_directory = get_package_share_directory('dan_ros_motor')
        json_file_path = os.path.join(package_share_directory, 'config/motor_param.json')
        if not os.path.exists(json_file_path):
            return 0
        with open(json_file_path, 'r') as f:
            data = json.load(f)
        
        odom_topic_name = data['odom_topic_name']

        self.dmx = DMXDriver()

        # Subscribe to an integer topic to receive commands
        self.subscription = self.create_subscription(
            RobotMode,
            'robot_mode',
            self.mode_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic_name,
            self.odom_callback,
            10
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )

        self.battery_percent = 100
        self.mode = 11
        self.last_cmd = None

        self.color = self.NO_COLOR
        self.blink = False
        self.direction = None

        self.get_logger().info("DMX Controller Node started, listening for commands on 'dmx_color_cmd'.")
    
    def mode_callback(self, msg):
        # print(msg.robot_mode)
        self.mode = msg.robot_mode
     
    def odom_callback(self, msg):
        cmd_l_x = msg.twist.twist.linear.x
        cmd_a_z = msg.twist.twist.angular.z
        
        if cmd_a_z == 0 and cmd_l_x == 0:
            self.direction = 'stay'
        elif cmd_a_z != 0:
            self.direction = 'right' if cmd_a_z < 0 else 'left'
        elif cmd_l_x != 0:
            self.direction = 'straight'

    def battery_callback(self, msg):
        self.battery_percent = msg.percentage

    def dmx_compute(self):
        
        if self.mode == RUNNING:
            # If mode is RUNNING but robot is static, show battery percentage
            if self.direction == 'stay':
                self.dmx.set_blink(False)
                self.color = self.BATTERY_COLORS
                if self.battery_percent >= 1:  # Full 
                    self.dmx.set_led_colour(self.color[3])
                else:
                    self.dmx.set_led_colour(self.color[floor(self.battery_percent*4)])
            else:
                self.dmx.set_blink(True)
                self.color = self.RUNNING
                if self.direction == 'right':
                    start = 3
                    end = 4
                elif self.direction == 'left':
                    start = 0
                    end = 1
                else:
                    start = 0
                    end = 4
                self.dmx.set_led_colour(self.color,start=start, end=end)

        elif self.mode == EMERGENCY:
            self.color = self.EMERGENCY
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

        elif self.mode == IDLE:
            self.color = self.IDLE
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

        elif self.mode == BOOTING:
            self.color = self.BOOTING
            self.dmx.set_blink(True)
            self.dmx.set_led_colour(self.color)

        elif self.mode == CHARGING or self.mode == MANUAL_CHARGE:
            self.color = self.CHARGING
            self.dmx.set_blink(True)
            self.dmx.set_led_colour(self.color)
        
        elif self.mode == ERROR:
            self.color = self.EMERGENCY
            self.dmx.set_blink(True)
            self.dmx.set_led_colour(self.color)
        
        elif self.mode == PAUSE:
            self.color = self.PAUSE
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

        else:
            self.color = self.NO_COLOR
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

    def run(self):
        try:
            while rclpy.ok():
                self.dmx_compute()
                self.dmx.update_dmx()
                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.dmx.destroy_dmx()


def main(args=None):
    rclpy.init(args=args)

    dmx_node = DMX512Node()
    dmx_node.run()
    dmx_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()