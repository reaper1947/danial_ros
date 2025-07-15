import can
import rclpy
from geometry_msgs.msg import Twist
import struct

bus = can.Bus(interface='socketcan', channel='can0')

def cal_data_frame(rpm):
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

def motor_run(m_l, m_r):   
    global bus     
    left_motor_msg = can.Message(arbitration_id=0x601, data=cal_data_frame(m_l), is_extended_id=False)
    right_motor_msg = can.Message(arbitration_id=0x602, data=cal_data_frame(m_r), is_extended_id=False)
    bus.send(left_motor_msg)
    bus.send(right_motor_msg)   

def callback(msg):
    l_x = msg.linear.x
    a_z = msg.angular.z
    left = (l_x - a_z)*100
    right = (l_x + a_z)*100
    # target_l = 100 if (l_x > 0 and a_z < 0) else 0 if (l_x == 0 and a_z == 0) else -100 
    # target_r = 100 if (l_x < 0 and a_z > 0) else 0 if (l_x == 0 and a_z == 0) else -100   
    print(left, -right)
    motor_run(left, -right)

rclpy.init()
cmd_node = rclpy.create_node("cmd_vel_node")
cmd_callback = cmd_node.create_subscription(Twist, '/cmd_vel', callback, 10)
rclpy.spin(cmd_node)
cmd_node.destroy_node()
rclpy.shutdown()