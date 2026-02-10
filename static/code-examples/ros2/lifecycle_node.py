#!/usr/bin/env python3
"""
Lifecycle Node Example
======================

This demonstrates ROS 2 lifecycle (managed) nodes.

Lifecycle nodes have explicit states and transitions:
- Unconfigured: Initial state
- Inactive: Configured but not active
- Active: Fully operational
- Finalized: Shutdown

This provides better control over node startup, shutdown, and error recovery.
"""

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String


class LifecycleNode(Node):
    """
    A lifecycle node that demonstrates state management.

    Lifecycle callbacks are called during state transitions:
    - on_configure: Set up resources
    - on_activate: Start operation
    - on_deactivate: Pause operation
    - on_cleanup: Release resources
    - on_shutdown: Final cleanup
    """

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        # Create a lifecycle publisher (only publishes when active)
        self.publisher = self.create_lifecycle_publisher(String, 'lifecycle_chatter', 10)

        self.timer = None
        self.counter = 0

        self.get_logger().info('Lifecycle node created (Unconfigured state)')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Called when transitioning from Unconfigured to Inactive.
        Set up resources here (open files, initialize hardware, etc.)
        """
        self.get_logger().info('on_configure() called')

        # Create timer but don't start it yet
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel()  # Don't run until activated

        self.get_logger().info('Node configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Called when transitioning from Inactive to Active.
        Start operation here (begin publishing, start processing, etc.)
        """
        self.get_logger().info('on_activate() called')

        # Activate the publisher
        self.publisher.on_activate(state)

        # Start the timer
        self.timer.reset()

        self.get_logger().info('Node activated - now publishing')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Called when transitioning from Active to Inactive.
        Pause operation here (stop publishing, pause processing, etc.)
        """
        self.get_logger().info('on_deactivate() called')

        # Deactivate the publisher
        self.publisher.on_deactivate(state)

        # Stop the timer
        self.timer.cancel()

        self.get_logger().info('Node deactivated - stopped publishing')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Called when transitioning from Inactive to Unconfigured.
        Release resources here (close files, release hardware, etc.)
        """
        self.get_logger().info('on_cleanup() called')

        # Destroy timer
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None

        self.counter = 0

        self.get_logger().info('Node cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Called when shutting down from any state.
        Final cleanup here.
        """
        self.get_logger().info(f'on_shutdown() called from {state.label}')

        # Clean up any remaining resources
        if self.timer:
            self.destroy_timer(self.timer)

        self.get_logger().info('Node shutdown complete')
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """
        Called periodically when node is active.
        """
        if self.publisher.is_activated:
            msg = String()
            msg.data = f'Lifecycle message #{self.counter}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = LifecycleNode('lifecycle_node')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
To control this lifecycle node, use ros2 lifecycle commands:

# Get current state
ros2 lifecycle get /lifecycle_node

# Configure the node
ros2 lifecycle set /lifecycle_node configure

# Activate the node (starts publishing)
ros2 lifecycle set /lifecycle_node activate

# Deactivate the node (stops publishing)
ros2 lifecycle set /lifecycle_node deactivate

# Cleanup (back to unconfigured)
ros2 lifecycle set /lifecycle_node cleanup

# Shutdown
ros2 lifecycle set /lifecycle_node shutdown
"""
