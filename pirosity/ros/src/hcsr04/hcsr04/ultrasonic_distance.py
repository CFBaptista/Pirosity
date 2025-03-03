from dataclasses import dataclass

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float64

from pirosity.core.sensors import UltrasonicSensorHCSR04, UltrasonicSensorHCSR04Data


@dataclass(frozen=True)
class UltrasonicDistancePublisherData:
    """
    Data class to hold constants for the ultrasonic distance publisher.

    Attributes
    ----------
    `queue_size` : int
        The size of the message queue.
    `timer` : float
        The period in seconds to publish the measured distance.
    """

    queue_size: int = 10
    timer: float = 0.5


class UltrasonicDistancePublisher(Node):
    """
    ROS2 node to publish the distance measured with the HC-SR04 ultrasonic sensor.
    """

    def __init__(self) -> None:
        self._declare_parameters()
        parameters = self._get_parameters()

        self.sensor = UltrasonicSensorHCSR04(
            parameters["trigger_pin"],
            parameters["echo_pin"],
            speed_of_sound=parameters["speed_of_sound"],
        )

        super().__init__("ultrasonic_distance_publisher")
        self.publisher_ = self.create_publisher(
            Float64, "ultrasonic_distance", parameters["queue_size"]
        )
        self.timer = self.create_timer(parameters["timer"], self.timer_callback)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "trigger_pin",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin used to trigger the sensor.",
            ),
        )
        self.declare_parameter(
            "echo_pin",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin used to receive the echo signal.",
            ),
        )
        self.declare_parameter("speed_of_sound", UltrasonicSensorHCSR04Data.speed_of_sound)
        self.declare_parameter("queue_size", UltrasonicDistancePublisherData.queue_size)
        self.declare_parameter("timer", UltrasonicDistancePublisherData.timer)

    def _get_parameters(self) -> dict:
        trigger_pin = self.get_parameter("trigger_pin").get_parameter_value().integer_value
        echo_pin = self.get_parameter("echo_pin").get_parameter_value().integer_value
        speed_of_sound = self.get_parameter("speed_of_sound").get_parameter_value().double_value
        queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value
        timer = self.get_parameter("timer").get_parameter_value().double_value

        parameters = dict(
            trigger_pin=trigger_pin,
            echo_pin=echo_pin,
            speed_of_sound=speed_of_sound,
            queue_size=queue_size,
            timer=timer,
        )

        return parameters

    def timer_callback(self) -> None:
        """
        Callback function to publish the measured distance.
        """
        distance_message = Float64()
        distance_message.data = self.sensor.measure()

        self.publisher_.publish(distance_message)
        self.get_logger().info(f"Publishing measured distance in meters: {distance_message.data}")


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    ultrasonic_distance = UltrasonicDistancePublisher()

    rclpy.spin(ultrasonic_distance)

    ultrasonic_distance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
