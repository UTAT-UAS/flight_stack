# feeds the rc topics from the flight controller to the flight controller through mavlink
# this allows us to modify the rc inputs yaw channel in flight while still having full position mode without emulation

import math

from std_msgs.msg import Float32

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

        self.xerror_subscriber = self.create_subscription(
            Float32,
            "/uas/cv/x_error",
            self.x_error_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
            

        # controller
        self.dx = 0.0
        self.k = 0.0005
        self.max_rate = 8 * math.pi/180
        self.max_px4_rate = 45 * math.pi/180
        self.max_pwm = int((self.max_rate / self.max_px4_rate) * 500) + 55

        # override params
        self.yaw_channel_idx = 3 # yaw channel (3)
        self.yaw_rate_target = 0.0
        self.override_pwm = 0

    def rc_callback(self, msg):
        # update yaw controller
        self.yaw_controller()

        # map yaw rate target to pwm channels (full scale is 45 dps for 1500-2000 us)
        # -ve yaw left, +ve yaw right

        # scale to max_px4_rate and then to PWM range

        if msg.channels[4] > 0:
            self.override_pwm = int(((self.yaw_rate_target / self.max_px4_rate) * 500) + 55 * math.copysign(1, self.yaw_rate_target)) # add a small offset to overcome deadzone
            self.override_pwm = max(min(self.override_pwm, self.max_pwm), -self.max_pwm)
        else: 
            self.override_pwm = 0
        
        rc_values = [65535] * 18
        
        # msg.channels contains the live RC inputs (scaled -1.0 to 1.0)
        # map to PWM equivalents (1000 to 2000 us) to forward correctly
        for i, val in enumerate(msg.channels):
            if i < 18:
                rc_values[i] = int((val + 1.0) * 500 + 1000)
                
        # override yaw
        rc_values[self.yaw_channel_idx] = rc_values[self.yaw_channel_idx] + self.override_pwm
        print(f"yaw_rate_input: {rc_values[self.yaw_channel_idx]} | override: {self.override_pwm}) | target: {self.yaw_rate_target * 180/math.pi:.2f} dps")
        
        # send RC override mavlink message
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_values
        )

    def x_error_cb(self, msg):
        self.dx = msg.data

    def yaw_controller(self):
        if self.dx is None:
            print("warning: target_dx not found in blackboard")
            self.yaw_rate_target = 0.0
        else:
            self.yaw_rate_target = min(max(self.k * self.dx, -self.max_rate), self.max_rate)
        

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