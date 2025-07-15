#!/usr/bin/env python3

import time
from typing import List

from dmx import Colour, DMXInterface, DMXUniverse, DMXLight

import rclpy
from rclpy.node import Node
from next2_msgs.msg import RobotMode

class DMXLight4Slot(DMXLight):
    """DMX light with RGBW channels."""

    def __init__(self, address: int = 1):
        super().__init__(address=address)
        self._colour = Colour(0, 0, 0)
        self._white = 0

    @property
    def slot_count(self) -> int:
        return 4

    def set_colour(self, colour: Colour):
        self._colour = colour

    def set_white(self, white: int):
        self._white = max(0, min(int(white), 255))

    def serialise(self) -> List[int]:
        return self._colour.serialise() + [self._white]

class DMX512Node(Node):
    def __init__(self):
        super().__init__('dmx_512')
        # self.mode_sub = self.create_subscription(RobotMode, '/robot_mode_dan', self.mode_callback, 10)
        self.interface = DMXInterface("FT232R")
        self.universe = DMXUniverse()

        self.lights = []
        for i in range(60):
            addr = 1 + i * 4
            light = DMXLight4Slot(address=addr)
            self.lights.append(light)
            self.universe.add_light(light)

        self.interface.__enter__()

        # Subscribe to an integer topic to receive commands
        self.subscription = self.create_subscription(
            RobotMode,
            'robot_mode_dan',
            self.mode_callback,
            10
        )

        self.get_logger().info("DMX Controller Node started, listening for commands on 'dmx_color_cmd'.")

        # Default to all lights off
        self._current_colour = Colour(0, 0, 0)
        self._current_white = 0

        self.blink_enabled = False
        self.blink_state = False
        self._last_blink_time = self.get_clock().now()
        self._blink_interval = 2  # seconds

        self.last_mode = None

    
    def mode_callback(self, msg):
        # print(msg.robot_mode)
        cmd = msg.robot_mode
        
        if cmd == self.last_mode:
            pass

        # RUNNING
        elif cmd == 2: 
            self.blink_enabled = False
            self.current_color = Colour(255, 0, 0)  # green
            # self.update_light(self.current_color)

        # EMERGENCY
        elif cmd == 5:
            self.blink_enabled = True
            self.blink_color = Colour(0, 255, 0)  # red blinking
            # self.current_color = Colour(0, 255, 0)  # red
            # self.update_light(self.current_color)
            

        # OTHER
        else:
            self.blink_enabled = False
            self.current_color = Colour(0, 0, 255)  # blue
            # self.update_light(self.current_color)

        self.last_mode = cmd

    def update_light(self, colour):
        for light in self.lights:
            light.set_colour(colour)

    def run_blinking(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_blink_time).nanoseconds / 1e9
        if self.blink_enabled and elapsed >= self._blink_interval:
            self._last_blink_time = now
            self.blink_state = not self.blink_state
            if self.blink_state:
                self.update_light(self.blink_color)
            else:
                self.update_light(Colour(0, 0, 0))  # off
        elif not self.blink_enabled:
            # Keep showing solid color if not blinking
            self.update_light(self.current_color)

    def run(self):
        try:
            while rclpy.ok():
                self.run_blinking()
                self.interface.set_frame(self.universe.serialise())
                self.interface.send_update()

                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.interface.__exit__(None, None, None)


def main(args=None):
    rclpy.init(args=args)

    dmx_node = DMX512Node()
    dmx_node.run()
    dmx_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    

