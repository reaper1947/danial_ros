import can

bus = can.Bus(interface='socketcan', channel='can0')

motor_l_get = False
motor_r_get = False

while not (motor_l_get and motor_r_get):
    frame = bus.recv(timeout=0.1)
    if frame == None:
        print("Frame not found")
        break

    if frame.arbitration_id == 0x581 or frame.arbitration_id == 0x582:
        command_byte = frame.data[0]
        print(command_byte)
        if command_byte == 0x60:
            pass
        elif command_byte == 0x80:
            print(f"Motor {hex(frame.arbitration_id)} : Abort")
            break
        else:
            print(f"Motor {hex(frame.arbitration_id)} : Unknown command")
            break
            