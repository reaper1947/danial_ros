import can
import time

# Factor for scaling
SCALING_FACTOR = 1875 / (512 * 10000 * 3.14)

# CAN interface configuration
can_interface = 'can0'  # Change this to your CAN interface
bus = can.Bus(interface='socketcan', channel=can_interface)

bus.set_filters([
            {"can_id": 0x181, "can_mask": 0x7FF, "extended": False},
        ])

# Message to send (ID: 0x601, Data: 406C600000000000)
msg = can.Message(arbitration_id=0x601,
                  data=[0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
                  is_extended_id=False)

try:
    while True:
        # Send message
        # bus.send(msg)
        print("Message sent")

        # Wait for response
        response = bus.recv(timeout=0.1)
        if response:
            # Extract bytes 4-7 (assuming little-endian 32-bit int)
            # if len(response.data) >= 8:
            #     raw_bytes = response.data[4:8]
            #     raw_int = int.from_bytes(raw_bytes, byteorder='little', signed=True)
            #     scaled_value = raw_int * SCALING_FACTOR
            #     print(f"Raw_Hex {raw_bytes}, Raw_Int: {raw_int}, Scaled: {scaled_value:.6f}")
            if len(response.data) >= 6:
                raw_bytes = response.data[0:4]
                raw_int = int.from_bytes(raw_bytes, byteorder='little', signed=True)
                scaled_value = raw_int * SCALING_FACTOR
                print(f"Raw_Hex {raw_bytes}, Raw_Int: {raw_int}, Scaled: {scaled_value:.6f}")
            else:
                print("Invalid response length")
        else:
            print("No response received")

except KeyboardInterrupt:
    print("Terminated by user")

finally:
    bus.shutdown()
