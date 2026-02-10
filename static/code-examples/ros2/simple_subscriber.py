#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber
=======================

This node demonstrates the subscriber pattern in ROS 2:
- Creating a subscriber on a topic
- Receiving and processing messages
- Callback-based message handling

To run this node:
    ros2 run <package_name> simple_subscriber

This subscriber listens to the /chatter topic published by simple_publisher.py
Run both nodes together to see publisher-subscriber communication in action.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    A ROS 2 node that subscribes to string messages on a topic.

    This demonstrates:
    - Creating a subscriber
    - Handling incoming messages with a callback
    - Processing message data
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('simple_subscriber')

        # Create a subscriber
        # Parameters:
        #   - Message type: String (must match publisher's type)
        #   - Topic name: 'chatter' (must match publisher's topic)
        #   - Callback function: called when a message arrives
        #   - Queue size: 10 (how many messages to buffer)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        # (subscription is stored but not explicitly used after creation)
        self.subscription

        # Counter to track received messages
        self.message_count = 0

        self.get_logger().info('Simple Subscriber started. Listening to /chatter')

    def listener_callback(self, msg):
        """
        Callback function that processes incoming messages.

        This function is called automatically whenever a message arrives
        on the /chatter topic.

        Args:
            msg (String): The received message
        """
        # Increment message counter
        self.message_count += 1

        # Log the received message
        self.get_logger().info(f'Received [{self.message_count}]: "{msg.data}"')

        # You can process the message data here
        # For example, extract information, trigger actions, etc.

        # Example: Check if message contains specific text
        if 'Hello' in msg.data:
            self.get_logger().debug('Message contains greeting!')


def main(args=None):
    """
    Main function to initialize and run the subscriber node.
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the subscriber node
    node = SimpleSubscriber()

    try:
        # Spin the node to process callbacks
        # This blocks and waits for messages to arrive
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
