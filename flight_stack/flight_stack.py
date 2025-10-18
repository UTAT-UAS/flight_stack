from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from px4_msgs.msg import (
    GotoSetpoint,
    VehicleAttitude,
    VehicleStatus,
    VehicleLocalPosition,
)
from flight_stack_msgs.srv import CoreCommand


class FlightPlanner(Node):
    def __init__(self):
        super().__init__("flight_planner")
        self._attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._attitude = VehicleAttitude()
        self._status_subscriber = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._status_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._status = VehicleStatus()
        self._position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._position_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._position = VehicleLocalPosition()

        self._goto_publisher = self.create_publisher(
            GotoSetpoint, "/uas/core/goto_setpoint", 10
        )

        self._core_command_client = self.create_client(CoreCommand, "/uas/core/command")

        timer_period = 0.1  # seconds
        self._timer = self.create_timer(timer_period, self.main_loop)

    def main_loop(self) -> None:
        """abstract"""
        pass

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        self._attitude = msg

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._status = msg

    def _position_cb(self, msg: VehicleLocalPosition) -> None:
        self._position = msg
