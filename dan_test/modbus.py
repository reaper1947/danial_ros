import can

# Define filters: only accept messages with CAN ID 0x701 or 0x702
can_filters = [
    {"can_id": 0x701, "can_mask": 0x7FF, "extended": False},
    {"can_id": 0x702, "can_mask": 0x7FF, "extended": False}
]
all_filters = [
        {"can_id": 0x181, "can_mask": 0x7FF, "extended": False},
        {"can_id": 0x182, "can_mask": 0x7FF, "extended": False}
]
heartbeat_filters = [
    {"can_id": 0x701, "can_mask": 0x7FF, "extended": False},
    {"can_id": 0x702, "can_mask": 0x7FF, "extended": False}
]
# Create the CAN bus with socketcan interface and apply filters
bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000, can_filters=can_filters)

print("Listening for CAN messages with ID 0x701 or 0x702...")

try:
    while True:
        bus.set_filters(heartbeat_filters)
        while bus.recv(timeout=0) is not None:
            pass
        pulse_get = bus.recv(timeout=0.1)
        print(pulse_get)
        bus.set_filters(all_filters)
        while bus.recv(timeout=0) is not None:
            pass
except KeyboardInterrupt:
    print("Stopped by user.")
