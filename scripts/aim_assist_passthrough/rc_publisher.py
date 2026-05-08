# feeds the rc topics from the flight controller to the flight controller through mavlink
# this allows us to modify the rc inputs yaw channel in flight while still having full position mode without emulation

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from px4_msgs.msg import RcChannels
from rclpy.qos import QoSPresetProfiles

class RcPassthroughNode(Node):
    def __init__(self):
        super().__init__('rc_passthrough_node')
        
        # Connect to MAVLink (Use 14540 for offboard API config in PX4 SITL)
        connection_string = 'udp:127.0.0.1:14540'
        self.get_logger().info(f"Connecting to MAVLink on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        
        self.get_logger().info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.get_logger().info(f"Heartbeat from system {self.master.target_system} component {self.master.target_component}")

        # Subscribe to the live RC input topic from PX4 via XRCE-DDS
        self.subscription = self.create_subscription(
            RcChannels,
            '/fmu/out/rc_channels',
            self.rc_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
            
        self.yaw_channel_idx = 3 # Channel 4 (0-indexed in Python array) is usually Yaw
        self.override_pwm = 30 # The value you want to inject

    def rc_callback(self, msg):
        rc_values = [65535] * 18
        
        # msg.channels contains the live RC inputs (scaled -1.0 to 1.0)
        # We need to map them back to PWM equivalents (1000 to 2000 us) to forward correctly
        for i, val in enumerate(msg.channels):
            if i < 18:
                rc_values[i] = int((val + 1.0) * 500 + 1000)
                
        # Override just the specific channel we want to control
        rc_values[self.yaw_channel_idx] = rc_values[self.yaw_channel_idx] + self.override_pwm
        
        # Send the modified package of RC channels back to PX4 as an override
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_values
        )
        

def main(args=None):
    rclpy.init(args=args)
    node = RcPassthroughNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Releasing channels...")
        # Release the override so the physical transmitter takes over completely again
        node.master.mav.rc_channels_override_send(
            node.master.target_system, node.master.target_component, *[65535]*18)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()