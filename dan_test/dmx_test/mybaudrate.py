#!/usr/bin/env python3
"""DMX with ROS 2 integer command to switch preset colors."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Integer message

from time import sleep
from typing import List

from dmx import Colour, DMXInterface, DMXUniverse, DMXLight


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


class DMXControllerNode(Node):
    def __init__(self):
        super().__init__('dmx_controller_node')

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
            Int32,
            'dmx_dan',
            self.color_cmd_callback,
            10
        )

        self.get_logger().info("DMX Controller Node started, listening for commands on 'dmx_color_cmd'.")

        # Default to all lights off
        self._current_colour = Colour(0, 0, 0)
        self._current_white = 0

    def color_cmd_callback(self, msg: Int32):
        cmd = msg.data
        self.get_logger().info(f"Received color command: {cmd}")

        if cmd == 1:
            # Green
            self._current_colour = Colour(0, 255, 0)
            self._current_white = 0
        elif cmd == 2:
            # Red
            self._current_colour = Colour(255, 0, 0)
            self._current_white = 0
        elif cmd == 3:
            # Blue
            self._current_colour = Colour(0, 0, 255)
            self._current_white = 0
        else:
            # Unknown command — turn off lights
            self._current_colour = Colour(0, 0, 0)
            self._current_white = 0

        for light in self.lights:
            light.set_colour(self._current_colour)
            light.set_white(self._current_white)

    def run(self):
        try:
            while rclpy.ok():
                self.interface.set_frame(self.universe.serialise())
                self.interface.send_update()

                rclpy.spin_once(self, timeout_sec=0.05)

        except KeyboardInterrupt:
            pass
        finally:
            self.interface.__exit__(None, None, None)


def main(args=None):
    rclpy.init(args=args)
    node = DMXControllerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
