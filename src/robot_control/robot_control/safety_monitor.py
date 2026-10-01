#!/usr/bin/env python3

# Node to monitor the robot's safety based on laser scan data. 
# It checks for obstacles within a specified threshold distance, and publishes commands to the /cmd_vel topic. 
# It also provides a service to set the safety threshold and publishes obstacle information to the /obstacle_info topic.

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from robot_interfaces.msg import ObstacleInfo
from robot_interfaces.srv import SetThreshold


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.threshold = 0.5
        self.closest_distance = float('inf')
        self.closest_direction = 'unknown'
        self.last_user_command = Twist()
        self.scan_received = False
        self.recovering = False
        self.recovery_command = Twist()
        self.recovery_start_time = None
        self.recovery_duration = 1.0

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.obstacle_publisher = self.create_publisher(
            ObstacleInfo,
            '/obstacle_info',
            10
        )

        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/user_cmd_vel',
            self.command_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.threshold_service = self.create_service(
            SetThreshold,
            '/set_threshold',
            self.set_threshold_callback
        )

        self.recovery_timer = self.create_timer(
            0.1,
            self.recovery_timer_callback
        )

        self.get_logger().info('Safety monitor started.')
        self.get_logger().info(
            f'Initial safety threshold: {self.threshold:.2f} m'
        )


    def scan_callback(self, msg):

        self.scan_received = True

        minimum_distance = float('inf')
        minimum_index = -1

        for index, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            if distance < minimum_distance:
                minimum_distance = distance
                minimum_index = index

        if minimum_index == -1:
            self.closest_distance = float('inf')
            self.closest_direction = 'unknown'
            self.publish_obstacle_info()
            return

        self.closest_distance = minimum_distance

        obstacle_angle = (
            msg.angle_min
            + minimum_index * msg.angle_increment
        )

        self.closest_direction = self.get_direction(
            obstacle_angle
        )

        self.publish_obstacle_info()

        if (self.closest_distance < self.threshold and not self.recovering and self.robot_is_moving(self.last_user_command)):
            self.start_recovery()

    def get_direction(self, angle):

        angle_degrees = math.degrees(angle)

        if -45.0 <= angle_degrees <= 45.0:
            return 'front'

        elif 45.0 < angle_degrees <= 135.0:
            return 'left'

        elif -135.0 <= angle_degrees < -45.0:
            return 'right'

        else:
            return 'back'


    def command_callback(self, msg):

        self.last_user_command = msg

        if self.recovering:
            self.get_logger().warning(
                'Recovery active. User command temporarily ignored.'
            )
            return

        if not self.scan_received:
            self.get_logger().warning(
                'No laser scan received yet. Robot command rejected.'
            )
            self.stop_robot()
            return

        if (self.closest_distance < self.threshold and self.robot_is_moving(msg)):
            self.get_logger().warning(
                f'Unsafe command: obstacle at '
                f'{self.closest_distance:.2f} m '
                f'({self.closest_direction}).'
            )

            self.start_recovery()
            return

        self.cmd_publisher.publish(msg)

        self.get_logger().info(
            f'Safe command forwarded: '
            f'linear={msg.linear.x:.2f} m/s, '
            f'angular={msg.angular.z:.2f} rad/s'
        )

    def start_recovery(self):

        if self.recovering:
            return

        self.recovering = True

        self.recovery_command = Twist()

        self.recovery_command.linear.x = (
            -self.last_user_command.linear.x
        )

        self.recovery_command.angular.z = (
            -self.last_user_command.angular.z
        )

        if not self.robot_is_moving(self.recovery_command):
            self.stop_robot()
            self.recovering = False
            return

        self.recovery_start_time = self.get_clock().now()

        self.get_logger().warning(
            f'SAFETY TRIGGERED: obstacle at '
            f'{self.closest_distance:.2f} m '
            f'({self.closest_direction}). '
            f'Reversing previous command.'
        )

        self.cmd_publisher.publish(
            self.recovery_command
        )

    def recovery_timer_callback(self):

        if not self.recovering:
            return

        current_time = self.get_clock().now()

        elapsed_time = (
            current_time - self.recovery_start_time
        ).nanoseconds / 1e9

        if elapsed_time < self.recovery_duration:

            self.cmd_publisher.publish(
                self.recovery_command
            )

        else:

            self.stop_robot()

            self.recovering = False

            self.get_logger().info(
                'Recovery finished. Robot stopped.'
            )

    def set_threshold_callback(self, request, response):

        new_threshold = request.threshold

        if new_threshold <= 0.0:
            response.success = False
            response.message = (
                'Threshold must be greater than 0 metres.'
            )

            return response

        self.threshold = new_threshold

        response.success = True
        response.message = (
            f'Safety threshold changed to '
            f'{self.threshold:.2f} m.'
        )

        self.get_logger().info(response.message)

        return response


    def publish_obstacle_info(self):

        msg = ObstacleInfo()

        msg.threshold = float(self.threshold)
        msg.distance = float(self.closest_distance)
        msg.direction = self.closest_direction

        self.obstacle_publisher.publish(msg)


    def robot_is_moving(self, command):

        return (
            abs(command.linear.x) > 0.001
            or abs(command.angular.z) > 0.001
        )
    def stop_robot(self):

        stop_command = Twist()
        stop_command.linear.x = 0.0
        stop_command.angular.z = 0.0
        self.cmd_publisher.publish(stop_command)

def main(args=None):

    rclpy.init(args=args)

    node = SafetyMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()