import argparse
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pirosity.core.sensors import UltrasonicSensorHCSR04


@dataclass(froze=True)
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

    def __init__(
        self,
        trigger_pin: int,
        echo_pin: int,
        queue_size: int = UltrasonicDistancePublisherData.queue_size,
        timer: float = UltrasonicDistancePublisherData.timer,
    ) -> None:
        """
        Parameters
        ----------
        `trigger_pin` : int
            GPIO pin number used to trigger the sensor.
        `echo_pin` : int
            GPIO pin number used to receive the echo signal.
        `queue_size` : int, optional (default=10)
            The size of the message queue.
        `timer` : float, optional (default=0.5)
            The period in seconds to publish the measured distance.
        """
        self.sensor = UltrasonicSensorHCSR04(trigger_pin, echo_pin)

        super().__init__("ultrasonic_distance_publisher")
        self.publisher_ = self.create_publisher(Float64, "ultrasonic_distance", queue_size)
        self.timer = self.create_timer(timer, self.timer_callback)

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
    ultrasonic_distance = UltrasonicDistancePublisher(
        arguments.trigger_pin, arguments.echo_pin, arguments.queue_size, arguments.timer
    )

    rclpy.spin(ultrasonic_distance)

    ultrasonic_distance.destroy_node()
    rclpy.shutdown()


def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns
    -------
    argparse.Namespace
        The parsed command line arguments.
    """
    parser = argparse.ArgumentParser(description="HC-SR04 ultrasonic distance publisher")
    parser.add_argument(
        "--trigger_pin", type=int, required=True, help="GPIO pin number used to trigger the sensor"
    )
    parser.add_argument(
        "--echo_pin",
        type=int,
        required=True,
        help="GPIO pin number used to receive the echo signal",
    )
    parser.add_argument(
        "--queue_size",
        type=int,
        default=UltrasonicDistancePublisherData.queue_size,
        help="The size of the message queue",
    )
    parser.add_argument(
        "--timer",
        type=float,
        default=UltrasonicDistancePublisherData.timer,
        help="The period in seconds to publish the measured distance",
    )

    args = parser.parse_args()
    return args


if __name__ == "__main__":
    arguments = parse_command_line_arguments()
    main(arguments)
