import math
import statistics

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class AutonomousDrive(Node):

    def __init__(self):
        super().__init__('autonomous_drive')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.speed = 0.85
        self.slow_speed = 0.55

        # Maximum steering angle
        self.max_steering = 0.45

        # Emergency distance
        self.stop_distance = 0.60

        # Start turning when front is too close
        self.turn_distance = 1.00

        # Detect left opening
        self.left_open_distance = 2.20

        # Desired distance from left wall
        self.target_left_distance = 0.90

        # Wall following gain
        self.wall_gain = 0.20

        # Ignore small wall-distance errors
        self.deadband = 0.12

        # Limit steering change per control cycle
        self.max_steering_change = 0.020

        # Current steering
        self.current_steering = 0.0

        # ============================================================
        # STATE
        # ============================================================

        self.state = 'FORWARD'

        self.scan = None

        # ============================================================
        # SUBSCRIBER
        # ============================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # ============================================================
        # TIMERS
        # ============================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.log_timer = self.create_timer(
            1.0,
            self.log_status
        )

        # ============================================================
        # STATUS
        # ============================================================

        self.last_front = float('inf')
        self.last_left = float('inf')
        self.last_right = float('inf')

        self.last_speed = 0.0
        self.last_steering = 0.0

        self.get_logger().info(
            'AutonomousDrive started.'
        )

        self.get_logger().info(
            'Waiting for /scan ...'
        )

    # ================================================================
    # LASER CALLBACK
    # ================================================================

    def scan_callback(self, msg):

        self.scan = msg

    # ================================================================
    # GET DISTANCE AROUND ONE ANGLE
    # ================================================================

    def get_distance(
        self,
        angle_deg,
        window_deg=2
    ):

        if self.scan is None:
            return float('inf')

        angle = math.radians(angle_deg)

        center_index = int(
            (angle - self.scan.angle_min)
            / self.scan.angle_increment
        )

        half_window = max(
            1,
            int(
                math.radians(window_deg)
                / self.scan.angle_increment
            )
        )

        start = max(
            0,
            center_index - half_window
        )

        end = min(
            len(self.scan.ranges),
            center_index + half_window + 1
        )

        values = []

        for i in range(start, end):

            distance = self.scan.ranges[i]

            if math.isfinite(distance):

                if (
                    self.scan.range_min
                    <= distance
                    <= self.scan.range_max
                ):
                    values.append(distance)

        if not values:
            return float('inf')

        return statistics.median(values)

    # ================================================================
    # GET DISTANCE FROM A SECTOR
    # ================================================================

    def sector_distance(
        self,
        start_deg,
        end_deg
    ):

        if self.scan is None:
            return float('inf')

        start_angle = math.radians(start_deg)
        end_angle = math.radians(end_deg)

        start_index = int(
            (start_angle - self.scan.angle_min)
            / self.scan.angle_increment
        )

        end_index = int(
            (end_angle - self.scan.angle_min)
            / self.scan.angle_increment
        )

        if start_index > end_index:

            start_index, end_index = (
                end_index,
                start_index
            )

        start_index = max(
            0,
            start_index
        )

        end_index = min(
            len(self.scan.ranges),
            end_index
        )

        values = []

        for i in range(
            start_index,
            end_index
        ):

            distance = self.scan.ranges[i]

            if math.isfinite(distance):

                if (
                    self.scan.range_min
                    <= distance
                    <= self.scan.range_max
                ):
                    values.append(distance)

        if not values:
            return float('inf')

        return statistics.median(values)

    # ================================================================
    # SMOOTH STEERING
    # ================================================================

    def smooth_steering(self, target):

        target = max(
            -self.max_steering,
            min(
                self.max_steering,
                target
            )
        )

        difference = (
            target - self.current_steering
        )

        if (
            difference
            > self.max_steering_change
        ):

            self.current_steering += (
                self.max_steering_change
            )

        elif (
            difference
            < -self.max_steering_change
        ):

            self.current_steering -= (
                self.max_steering_change
            )

        else:

            self.current_steering = target

        return self.current_steering

    # ================================================================
    # PUBLISH DRIVE COMMAND
    # ================================================================

    def publish_drive(
        self,
        speed,
        steering
    ):

        steering = self.smooth_steering(
            steering
        )

        msg = AckermannDriveStamped()

        msg.drive.speed = float(speed)

        msg.drive.steering_angle = float(
            steering
        )

        self.drive_pub.publish(msg)

        self.last_speed = speed
        self.last_steering = steering

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # Wait for LiDAR
        # ------------------------------------------------------------

        if self.scan is None:
            return

        # ============================================================
        # READ LIDAR
        # ============================================================

        front = self.get_distance(
            0,
            3
        )

        front_left = self.sector_distance(
            20,
            65
        )

        front_right = self.sector_distance(
            -65,
            -20
        )

        left = self.sector_distance(
            70,
            100
        )

        right = self.sector_distance(
            -100,
            -70
        )

        self.last_front = front
        self.last_left = left
        self.last_right = right

        # ============================================================
        # EMERGENCY OBSTACLE
        # ============================================================

        if front < self.stop_distance:

            if left > right:

                self.state = 'TURN_LEFT'

            else:

                self.state = 'TURN_RIGHT'

        # ============================================================
        # FORWARD
        # ============================================================

        if self.state == 'FORWARD':

            # --------------------------------------------------------
            # Front obstacle
            # --------------------------------------------------------

            if front < self.turn_distance:

                if left > right:

                    self.state = 'TURN_LEFT'

                else:

                    self.state = 'TURN_RIGHT'

                return

            # --------------------------------------------------------
            # LEFT JUNCTION
            #
            # Prefer left when there is a clear opening.
            # --------------------------------------------------------

            if (
                left > self.left_open_distance
                and front > 1.8
            ):

                self.state = 'TURN_LEFT'

                return

            # ========================================================
            # GENTLE WALL FOLLOWING
            # ========================================================

            steering_target = 0.0

            if math.isfinite(left):

                error = (
                    self.target_left_distance
                    - left
                )

                # ----------------------------------------------------
                # Deadband
                # ----------------------------------------------------

                if abs(error) < self.deadband:

                    error = 0.0

                steering_target = (
                    self.wall_gain * error
                )

            # --------------------------------------------------------
            # Too close to left wall
            #
            # Negative = steer right
            # --------------------------------------------------------

            if left < 0.55:

                steering_target = -0.12

            # --------------------------------------------------------
            # Too close to right wall
            #
            # Positive = steer left
            # --------------------------------------------------------

            elif right < 0.55:

                steering_target = 0.12

            # --------------------------------------------------------
            # Limit normal wall correction
            # --------------------------------------------------------

            steering_target = max(
                -0.18,
                min(
                    0.18,
                    steering_target
                )
            )

            # ========================================================
            # SPEED
            # ========================================================

            speed = self.speed

            if front < 1.8:

                speed = self.slow_speed

            if abs(steering_target) > 0.12:

                speed = min(
                    speed,
                    0.65
                )

            self.publish_drive(
                speed,
                steering_target
            )

        # ============================================================
        # TURN LEFT
        # ============================================================

        elif self.state == 'TURN_LEFT':

            # --------------------------------------------------------
            # Front is still blocked.
            #
            # Very gentle left steering.
            # --------------------------------------------------------

            if front < 1.0:

                self.publish_drive(
                    0.30,
                    0.18
                )

                return

            # --------------------------------------------------------
            # Front is opening.
            #
            # Reduce steering.
            # --------------------------------------------------------

            elif front < 1.5:

                self.publish_drive(
                    0.40,
                    0.14
                )

                return

            # --------------------------------------------------------
            # Front is clear.
            #
            # Finish the turn immediately.
            # --------------------------------------------------------

            else:

                self.state = 'FORWARD'

                self.publish_drive(
                    self.speed,
                    0.0
                )

                return

        # ============================================================
        # TURN RIGHT
        # ============================================================

        elif self.state == 'TURN_RIGHT':

            # --------------------------------------------------------
            # Front is still blocked.
            # --------------------------------------------------------

            if front < 1.0:

                self.publish_drive(
                    0.30,
                    -0.18
                )

                return

            # --------------------------------------------------------
            # Front is opening.
            # --------------------------------------------------------

            elif front < 1.5:

                self.publish_drive(
                    0.40,
                    -0.14
                )

                return

            # --------------------------------------------------------
            # Front is clear.
            # --------------------------------------------------------

            else:

                self.state = 'FORWARD'

                self.publish_drive(
                    self.speed,
                    0.0
                )

                return

    # ================================================================
    # STATUS LOG
    # ================================================================

    def log_status(self):

        if self.scan is None:

            self.get_logger().info(
                'Waiting for /scan ...'
            )

            return

        self.get_logger().info(
            f'[{self.state}] '
            f'Front={self.last_front:.2f} m | '
            f'Left={self.last_left:.2f} m | '
            f'Right={self.last_right:.2f} m | '
            f'Speed={self.last_speed:.2f} | '
            f'Steer={self.last_steering:.2f}'
        )

    # ================================================================
    # STOP
    # ================================================================

    def stop(self):

        msg = AckermannDriveStamped()

        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        self.drive_pub.publish(msg)

        self.get_logger().info(
            'Vehicle stopped.'
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = AutonomousDrive()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt received.'
        )

    finally:

        node.stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
