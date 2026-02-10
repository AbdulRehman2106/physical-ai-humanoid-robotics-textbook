#!/usr/bin/env python3
"""
Simple ROS 2 Publisher
======================

This node demonstrates the publisher pattern in ROS 2:
- Creating a publisher on a topic
- Publishing messages at a regular rate
- Using standard ROS 2 message types

To run this node:
    ros2 run <package_name> simple_publisher

To see the published messages:
    ros2 topic echo /chatter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    A ROS 2 node that publishes string messages to a topic.

    This demonstrates:
    - Creating a publisher
    - Publishing messages periodically
    - Using standard message types (String)
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('simple_publisher')

        # Create a publisher
        # Parameters:
        #   - Message type: String (from std_msgs)
        #   - Topic name: 'chatter'
        #   - Queue size: 10 (how many messages to buffer)
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a timer that publishes every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to make each message unique
        self.counter = 0

        self.get_logger().info('Simple Publisher started. Publishing to /chatter')

    def timer_callback(self):
        """
        Called periodically by the timer to publish a message.
        """
        # Create a new String message
        msg = String()

        # Set the message data
        msg.data = f'Hello ROS 2! Message #{self.counter}'

        # Publish the message
        self.publisher.publish(msg)

        # Log what we published (useful for debugging)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter for next message
        self.counter += 1


def main(args=None):
    """
    Main function to initialize and run the publisher node.
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the publisher node
    node = SimplePublisher()

    try:
        # Spin the node to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
