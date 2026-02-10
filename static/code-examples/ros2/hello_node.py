#!/usr/bin/env python3
"""
Hello World ROS 2 Node
======================

This is the simplest possible ROS 2 node. It demonstrates:
- Creating a node
- Using the ROS 2 logging system
- Running a node with rclpy

To run this node:
    ros2 run <package_name> hello_node

Or directly with Python:
    python3 hello_node.py
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """
    A minimal ROS 2 node that prints a greeting message.

    This node demonstrates the basic structure of a ROS 2 node:
    - Inheriting from rclpy.node.Node
    - Calling the parent constructor with a node name
    - Using the node's logger for output
    """

    def __init__(self):
        # Initialize the parent Node class with a unique node name
        super().__init__('hello_node')

        # Log an info message using the node's built-in logger
        # This is better than print() because it integrates with ROS 2's logging system
        self.get_logger().info('Hello from ROS 2! Node initialized successfully.')

        # Create a timer that calls our callback function every 2 seconds
        # This demonstrates how to create periodic behaviors in ROS 2
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """
        This function is called every 2 seconds by the timer.
        It demonstrates periodic execution and logging.
        """
        self.counter += 1
        self.get_logger().info(f'Hello World #{self.counter}')


def main(args=None):
    """
    Main function that initializes ROS 2, creates the node, and spins.

    The spin() call blocks and processes callbacks (like our timer_callback)
    until the node is shut down (e.g., with Ctrl+C).
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our HelloNode
    node = HelloNode()

    try:
        # Spin the node - this processes callbacks until shutdown
        # The node will keep running and calling timer_callback every 2 seconds
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Node stopped by user')
    finally:
        # Clean up: destroy the node and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
