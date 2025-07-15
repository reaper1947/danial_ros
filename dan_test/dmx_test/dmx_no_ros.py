from indicator_driver import DMXDriver
import time

def main(args=None):
    dmx = DMXDriver() # ttyUSB3
    dmx.set_led_colour((255,0,0), mode="Dim")
    while True:
        dmx.update_dmx()
        # time.sleep(0.01)

if __name__ == '__main__':
    main()
    

