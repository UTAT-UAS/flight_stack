from rclpy.node import Node

from px4_msgs.msg import GotoSetpoint


class FlightStack(Node):
    def __init__(self):
        super().__init__("flight_stack")
        self._publisher = self.create_publisher(
            GotoSetpoint, "/uas/core/goto_setpoint", 10
        )

        timer_period = 0.5  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = GotoSetpoint()
        msg.position = [0.0, 0.0, -5.0]
        self._publisher.publish(msg)
        print("Published setpoint")
