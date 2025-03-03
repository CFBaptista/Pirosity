import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pirosity.core.sensors import UltrasonicSensorHCSR04


class UltrasonicDistancePublisher(Node):
    """
    ROS2 node to publish the distance measured with the HC-SR04 ultrasonic sensor.
    """

    def __init__(self):
        self.sensor = UltrasonicSensorHCSR04(23, 24)

        super().__init__("ultrasonic_distance_publisher")
        self.publisher_ = self.create_publisher(Float64, "ultrasonic_distance", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self) -> None:
        """
        Callback function to publish the measured distance.
        """
        distance_message = Float64()
        distance_message.data = self.sensor.measure()

        self.publisher_.publish(distance_message)
        self.get_logger().info(f"Publishing measured distance in meters: {distance_message.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    ultrasonic_distance = UltrasonicDistancePublisher()

    rclpy.spin(ultrasonic_distance)

    ultrasonic_distance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
