#!/usr/bin/env python3

# Node to get user input: linear and angular velocity commands, publish them to the /user_cmd_vel topic, and maintain a history of the last 5 commands.
# It also provides a service to get the average of the last 5 commands.


from collections import deque
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robot_interfaces.srv import GetAverageVelocity


class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/user_cmd_vel',
            10
        )

        self.velocity_history = deque(maxlen=5)

        self.average_service = self.create_service(
            GetAverageVelocity,
            '/get_average_velocity',
            self.get_average_callback
        )

        self.get_logger().info('Velocity controller started.')

        self.input_thread = threading.Thread(
            target=self.input_loop,
            daemon=True
        )

        self.input_thread.start()

    def publish_velocity(self, linear, angular):

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_publisher.publish(msg)

        self.velocity_history.append(
            (float(linear), float(angular))
        )

        self.get_logger().info(
            f'Command: linear={linear:.2f} m/s, '
            f'angular={angular:.2f} rad/s'
        )

    def get_average_callback(self, request, response):

        sample_count = len(self.velocity_history)

        if sample_count == 0:
            response.average_linear = 0.0
            response.average_angular = 0.0
            response.sample_count = 0
            return response

        total_linear = 0.0 
        for command in self.velocity_history:
            total_linear += command[0]

        total_angular = 0.0
        for command in self.velocity_history:
            total_angular += command[1]

        response.average_linear = total_linear / sample_count
        response.average_angular = total_angular / sample_count
        response.sample_count = sample_count

        return response

    def input_loop(self):

        while rclpy.ok():

            try:
                print('\nEnter robot velocity command')
                print("Type 'q' to quit.")

                linear_input = input('Linear velocity [m/s]: ')

                if linear_input.lower() == 'q':
                    rclpy.shutdown()
                    break

                angular_input = input('Angular velocity [rad/s]: ')

                if angular_input.lower() == 'q':
                    rclpy.shutdown()
                    break

                linear = float(linear_input)
                angular = float(angular_input)

                self.publish_velocity(linear, angular)

            except ValueError:
                self.get_logger().warning(
                    'Invalid input. Please enter numeric values.'
                )


def main(args=None):

    rclpy.init(args=args)

    node = VelocityController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()