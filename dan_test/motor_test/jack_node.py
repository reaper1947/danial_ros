from jack_driver import JackDriver
import time

def main(args=None):
    jack = JackDriver()
    jack.init_motor()
    
    status = 'tofloor'
    last_status = None

    while True:
        din2, din3, raw_bytes = jack.read_din_status()

        if din2 == 1 and din3 == 1:
            print('Hit floor')
            jack.stop_motor()
            time.sleep(10)  # wait
            jack.set_speed(-100)
            while True:
                din2, din3, _ = jack.read_din_status()
                if not (din2 == 1 and din3 == 1):
                    print('Left floor zone')
                    break
                time.sleep(0.1)
            status = 'toceiling'

        elif din2 == 0 and din3 == 0:
            print('Hit ceiling')
            jack.stop_motor()
            time.sleep(10)
            jack.set_speed(100)
            while True:
                din2, din3, _ = jack.read_din_status()
                if not (din2 == 0 and din3 == 0):
                    print('Left ceiling zone')
                    break
                time.sleep(0.1)
            status = 'tofloor'

        if status != last_status:
            if status == 'tofloor':
                print('Going to floor')
                jack.set_speed(100)
            elif status == 'toceiling':
                print('Going to ceiling')
                jack.set_speed(-100)
            last_status = status

        time.sleep(0.1)

if __name__ == '__main__':
    main()
