#!/usr/bin/env python3

import time
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

class DMXDriver():
    """DMX Driver"""

    def __init__(self):

        # Device Setup
        self.led_num = 60

        self._colour = Colour(0, 0, 0)
        self._white = 0

        self.interface = DMXInterface("FT232R")
        self.universe = DMXUniverse()

        self.lights = []
        for i in range(self.led_num):
            addr = 1 + i * 4
            light = DMXLight4Slot(address=addr)
            self.lights.append(light)
            self.universe.add_light(light)

        self.interface.__enter__()

        # LED Status
        self.blink_enabled = False
        self.blink_state = False
        self._last_blink_time = time.time()
        self._blink_interval = 0.1  # seconds
        self.last_mode = None

        self.led_start = 0
        self.led_end = self.led_num
        # self.mode = 'static'

        # Default to all lights off
        self.current_colour = Colour(0, 0, 0)

    def set_blink(self, blink):
        self.blink_enabled = blink
    
    def set_led_colour(self, colour_tuple, start=None, end=None):

        # User Error
        start = 0 if start == None else start
        end = self.led_num if end == None else end

        if end >= start:
            self.led_start = max(int(start), 0)
            self.led_end = min(int(end), self.led_num)
        else:
            self.led_start = max(int(end), 0)
            self.led_end = min(int(start), self.led_num)

        self.current_colour = Colour(colour_tuple[1], colour_tuple[0], colour_tuple[2])
        # self.mode = mode

    def set_freq(self, freq):
        self._blink_interval = freq

    def update_light(self, colour):
        for i in range(0, len(self.lights)):
            if i >= self.led_start and i < self.led_end:
                self.lights[i].set_colour(colour)
            else:
                self.lights[i].set_colour(Colour(0, 0, 0))

    def run_led(self):
        now = time.time()
        elapsed = now - self._last_blink_time
        if self.blink_enabled and elapsed >= self._blink_interval:
            self._last_blink_time = now
            self.blink_state = not self.blink_state
            if self.blink_state:
                self.update_light(self.current_colour)
            else:
                self.update_light(Colour(0, 0, 0))  # off
        elif not self.blink_enabled:
            # Keep showing solid color if not blinking
            self.update_light(self.current_colour)

    def update_dmx(self):
        self.run_led()
        self.interface.set_frame(self.universe.serialise())
        self.interface.send_update()

    def destroy_dmx(self):
        self.interface.__exit__(None, None, None)
