#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import ForceTorqueCmd
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

class ForceTorqueServer(Node):
    def __init__(self):
        super().__init__('force_torque_server')
        
        # Initialize cflib
        cflib.crtp.init_drivers()
        
        # Parameters
        self.declare_parameter('uri', 'radio://0/80/2M/E7E7E7E7E7')
        self.uri = self.get_parameter('uri').get_parameter_value().string_value
        
        # Create subscription for force/torque commands
        self.subscription = self.create_subscription(
            ForceTorqueCmd,
            'force_torque_cmd',
            self.force_torque_callback,
            10
        )
        
        # Connect to Crazyflie
        self.get_logger().info(f'Connecting to Crazyflie at {self.uri}')
        self.cf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
        
    def force_torque_callback(self, msg):
        """Handle incoming force/torque commands"""
        try:
            with self.cf as scf:
                scf.cf.commander.send_force_torque_setpoint(
                    msg.thrust_si,
                    msg.torque_x, 
                    msg.torque_y,
                    msg.torque_z
                )
        except Exception as e:
            self.get_logger().error(f'Failed to send force/torque command: {e}')

def main(args=None):
    rclpy.init(args=args)
    server = ForceTorqueServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
