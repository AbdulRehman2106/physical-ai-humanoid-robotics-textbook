#!/usr/bin/env python3
"""
QoS Configuration Examples
==========================

This file demonstrates different Quality of Service (QoS) configurations
for various robotics scenarios in ROS 2.

QoS policies control how messages are delivered, including:
- Reliability: Best effort vs reliable delivery
- Durability: Transient local (keep last for late joiners) vs volatile
- History: Keep last N vs keep all
- Lifespan: How long messages are valid
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# Scenario 1: Sensor Data (High-frequency, lossy OK)
# Use case: Camera images, lidar scans
# Rationale: Losing occasional messages is acceptable; latest data is most important
SENSOR_DATA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Don't retry lost messages
    durability=DurabilityPolicy.VOLATILE,        # Don't keep for late joiners
    history=HistoryPolicy.KEEP_LAST,             # Only keep latest messages
    depth=1                                       # Keep only the most recent message
)

# Scenario 2: Robot Commands (Must be reliable)
# Use case: Velocity commands, gripper commands
# Rationale: Commands must arrive; missing commands could be dangerous
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,      # Retry until delivered
    durability=DurabilityPolicy.VOLATILE,        # Don't keep for late joiners
    history=HistoryPolicy.KEEP_LAST,
    depth=10                                      # Buffer last 10 commands
)

# Scenario 3: System State (Late joiners need last value)
# Use case: Robot pose, battery level, system status
# Rationale: New subscribers should immediately get the current state
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep last message for late joiners
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Scenario 4: Logging/Recording (Keep all messages)
# Use case: Data recording, diagnostics
# Rationale: Need complete history for analysis
LOGGING_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL,               # Keep all messages
    depth=1000                                     # Large buffer
)

# Scenario 5: Real-time Control (Low latency critical)
# Use case: Motor control loops, balance control
# Rationale: Latest data only; old data is worse than no data
REALTIME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


def print_qos_info(name, qos):
    """Print QoS profile information."""
    print(f"\n{name}:")
    print(f"  Reliability: {qos.reliability}")
    print(f"  Durability: {qos.durability}")
    print(f"  History: {qos.history}")
    print(f"  Depth: {qos.depth}")


if __name__ == '__main__':
    print("ROS 2 QoS Configuration Examples")
    print("=" * 50)

    print_qos_info("Sensor Data QoS", SENSOR_DATA_QOS)
    print_qos_info("Command QoS", COMMAND_QOS)
    print_qos_info("State QoS", STATE_QOS)
    print_qos_info("Logging QoS", LOGGING_QOS)
    print_qos_info("Real-time QoS", REALTIME_QOS)

    print("\n" + "=" * 50)
    print("QoS Selection Guide:")
    print("  - Sensor data: BEST_EFFORT, VOLATILE, KEEP_LAST(1)")
    print("  - Commands: RELIABLE, VOLATILE, KEEP_LAST(10)")
    print("  - State: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(1)")
    print("  - Logging: RELIABLE, TRANSIENT_LOCAL, KEEP_ALL")
    print("  - Real-time: BEST_EFFORT, VOLATILE, KEEP_LAST(1)")
