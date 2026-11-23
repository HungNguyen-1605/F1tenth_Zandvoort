import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):

        # Angle corresponding to the front of the car: 0 radians
        front_index = int((0.0 - msg.angle_min) / msg.angle_increment)

        distance = msg.ranges[front_index]

        self.get_logger().info(
            f'Front distance: {distance:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)

    node = LidarTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
