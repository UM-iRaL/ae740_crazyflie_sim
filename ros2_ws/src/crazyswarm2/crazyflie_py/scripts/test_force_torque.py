#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import ForceTorqueCmd

class ForceTorquePublisher(Node):
    def __init__(self):
        super().__init__('force_torque_publisher')
        self.publisher = self.create_publisher(ForceTorqueCmd, '/cf_1/force_torque_cmd', 10)
        
    def send_command(self):
        msg = ForceTorqueCmd()
        msg.thrust_si = 1.0
        msg.torque_x = 0.0
        msg.torque_y = 0.0
        msg.torque_z = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: thrust={msg.thrust_si}, torques=({msg.torque_x}, {msg.torque_y}, {msg.torque_z})')

def main():
    rclpy.init()
    publisher = ForceTorquePublisher()
    
    # Send one message
    publisher.send_command()
    
    # Or send continuously
    timer = publisher.create_timer(0.1, publisher.send_command)  # 10 Hz
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()