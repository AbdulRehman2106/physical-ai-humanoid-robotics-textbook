# Code Example Template

**Purpose**: Standardized structure for code examples
**Version**: 1.0
**Last Updated**: 2026-02-09

## Code Example Metadata

```yaml
code_example:
  id: "EX-[X].[Y]"
  title: "[Descriptive Title]"
  chapter: [X]
  language: [python|cpp|bash|yaml]
  difficulty: [beginner|intermediate|advanced]
  learning_outcomes:
    - "LO-[X].[Y]"
  dependencies:
    - "[package-name==version]"
  file_path: "code-examples/[category]/[filename]"
  tested: true
  test_date: "YYYY-MM-DD"
```

## Code Example Structure

### 1. Context and Purpose

```markdown
### Example [X].[Y]: [Title]

**Purpose**: [What this example demonstrates]

**Learning Objective**: [Which LO this supports]

**Difficulty**: [Beginner|Intermediate|Advanced]

**Prerequisites**:
- [Required knowledge or previous examples]
```

### 2. Complete Code

```markdown
**Code**:

```[language]
# Filename: [filename]
# Purpose: [Brief description]
# Dependencies: [list]

[Complete, runnable code with strategic comments]
```
```

### 3. Explanation

```markdown
**Understanding the Code**:

[Section-by-section or line-by-line explanation]

**Key Concepts**:
- **[Concept 1]**: [Explanation]
- **[Concept 2]**: [Explanation]

**Design Decisions**:
- [Why this approach was chosen]
- [Trade-offs considered]
```

### 4. Running the Example

```markdown
**Running the Example**:

```bash
[Exact commands to execute]
```

**Expected Output**:
```
[What you should see]
```

**Troubleshooting**:
- **Issue**: [Common problem]
  **Solution**: [How to fix]
```

### 5. Experimentation

```markdown
**Try It Yourself**:

Modify the code to:
- [ ] [Modification 1 - simple change]
- [ ] [Modification 2 - moderate change]
- [ ] [Modification 3 - challenging change]

**Hints**:
- For modification 1: [Hint]
- For modification 2: [Hint]
```

---

## Example Types

### Type 1: Minimal Example (Beginner)

**Purpose**: Show simplest possible implementation
**Characteristics**:
- 10-30 lines of code
- Single concept focus
- Heavily commented
- No error handling (unless that's the concept)
- Clear, obvious behavior

**Template**:
```markdown
### Example 3.1: Minimal ROS 2 Publisher

**Purpose**: Demonstrate the simplest possible ROS 2 publisher

**Learning Objective**: LO-3.1 (Create a basic ROS 2 publisher)

**Difficulty**: Beginner

**Code**:

```python
# minimal_publisher.py
# Purpose: Publish a simple message every second
# Dependencies: rclpy, std_msgs

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('minimal_publisher')

        # Create a publisher: message type, topic name, queue size
        self.publisher = self.create_publisher(String, 'topic', 10)

        # Create a timer that calls our function every second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Counter for our messages
        self.count = 0

    def timer_callback(self):
        # Create a message
        msg = String()
        msg.data = f'Hello {self.count}'

        # Publish the message
        self.publisher.publish(msg)

        # Log what we published
        self.get_logger().info(f'Published: "{msg.data}"')

        # Increment counter
        self.count += 1

def main():
    # Initialize ROS 2
    rclpy.init()

    # Create our node
    node = MinimalPublisher()

    # Keep the node running
    rclpy.spin(node)

    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Understanding the Code**:

1. **Imports**: We need `rclpy` for ROS 2 functionality and `String` for our message type
2. **Node Class**: Inherits from `Node` to get ROS 2 capabilities
3. **Publisher Creation**: `create_publisher(type, topic, queue_size)` sets up publishing
4. **Timer**: Calls our callback function every 1.0 seconds
5. **Callback**: Creates and publishes a message each time it's called
6. **Main Function**: Standard ROS 2 pattern - init, spin, shutdown

**Key Concepts**:
- **Node**: Independent process in ROS 2
- **Publisher**: Sends messages to a topic
- **Timer**: Triggers periodic actions
- **Spin**: Keeps node running and processing callbacks

**Running the Example**:

```bash
# Run the publisher
python3 minimal_publisher.py

# In another terminal, see the messages
ros2 topic echo /topic
```

**Expected Output**:
```
[minimal_publisher]: Published: "Hello 0"
[minimal_publisher]: Published: "Hello 1"
[minimal_publisher]: Published: "Hello 2"
```

**Try It Yourself**:
- [ ] Change the message to include your name
- [ ] Modify the timer to publish every 0.5 seconds
- [ ] Add a maximum count and stop after 10 messages
```

---

### Type 2: Practical Example (Intermediate)

**Purpose**: Show realistic, production-ready code
**Characteristics**:
- 50-150 lines of code
- Multiple concepts integrated
- Moderate comments (key decisions explained)
- Error handling included
- Best practices demonstrated

**Template**:
```markdown
### Example 6.2: Robot Controller with Sensor Feedback

**Purpose**: Demonstrate a complete robot controller that processes sensor data and publishes control commands

**Learning Objective**: LO-6.2 (Integrate multiple ROS 2 communication patterns)

**Difficulty**: Intermediate

**Prerequisites**:
- Understanding of publishers and subscribers
- Familiarity with ROS 2 message types
- Basic control theory concepts

**Code**:

```python
# robot_controller.py
# Purpose: Control robot based on sensor feedback
# Dependencies: rclpy, geometry_msgs, sensor_msgs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Control parameters
        self.max_speed = 0.5  # m/s
        self.safe_distance = 1.0  # meters

        # State
        self.obstacle_detected = False
        self.min_distance = float('inf')

        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data and detect obstacles"""
        # Find minimum distance in front of robot (center 60 degrees)
        center_start = len(msg.ranges) // 2 - 30
        center_end = len(msg.ranges) // 2 + 30
        center_ranges = msg.ranges[center_start:center_end]

        # Filter out invalid readings
        valid_ranges = [r for r in center_ranges if msg.range_min < r < msg.range_max]

        if valid_ranges:
            self.min_distance = min(valid_ranges)
            self.obstacle_detected = self.min_distance < self.safe_distance
        else:
            self.min_distance = float('inf')
            self.obstacle_detected = False

        # Update control based on sensor data
        self.update_control()

    def update_control(self):
        """Calculate and publish control commands"""
        cmd = Twist()

        if self.obstacle_detected:
            # Stop if obstacle too close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn to avoid
            self.get_logger().warn(
                f'Obstacle at {self.min_distance:.2f}m - avoiding')
        else:
            # Move forward at safe speed
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutting down
        stop_cmd = Twist()
        controller.cmd_pub.publish(stop_cmd)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Understanding the Code**:

**Architecture**:
- Subscribes to laser scan data
- Processes sensor data to detect obstacles
- Publishes velocity commands based on sensor input

**Key Sections**:
1. **Initialization**: Sets up publishers, subscribers, and parameters
2. **Scan Callback**: Processes laser data, detects obstacles
3. **Control Update**: Calculates appropriate velocity commands
4. **Main Function**: Includes proper cleanup on shutdown

**Design Decisions**:
- **Center-focused detection**: Only checks front 60 degrees for obstacles (robot moving forward)
- **Safe distance threshold**: 1.0m provides time to react
- **Simple avoidance**: Stops and turns when obstacle detected (could be more sophisticated)
- **Graceful shutdown**: Stops robot before exiting

**Running the Example**:

```bash
# Start Gazebo with a robot
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Run the controller
python3 robot_controller.py

# Monitor commands
ros2 topic echo /cmd_vel
```

**Expected Behavior**:
- Robot moves forward at 0.5 m/s
- Stops and turns when obstacle within 1.0m
- Resumes forward motion when clear

**Try It Yourself**:
- [ ] Add proportional control (speed decreases as obstacle gets closer)
- [ ] Implement different behaviors for left vs right obstacles
- [ ] Add a service to change max_speed dynamically
```

---

### Type 3: Advanced Example (Advanced)

**Purpose**: Demonstrate production patterns and best practices
**Characteristics**:
- 150+ lines of code
- Complex integration
- Minimal comments (self-documenting code)
- Comprehensive error handling
- Performance optimized

[Similar template structure with more complex code]

---

## Code Quality Standards

### All Examples Must:
- [ ] Be complete and runnable (no fragments)
- [ ] Include all necessary imports
- [ ] Follow language style guide (PEP 8 for Python)
- [ ] Have been tested successfully
- [ ] Include expected output
- [ ] List all dependencies with versions
- [ ] Use descriptive variable and function names
- [ ] Include docstrings for functions/classes

### Comments Should:
- [ ] Explain "why" not "what"
- [ ] Clarify non-obvious decisions
- [ ] Provide context for complex logic
- [ ] Be up-to-date with code
- [ ] Not state the obvious

### Error Handling:
- [ ] Beginner: Optional (focus on core concept)
- [ ] Intermediate: Basic error handling shown
- [ ] Advanced: Comprehensive error handling

---

## Testing Checklist

Before publishing a code example:
- [ ] Code runs without errors
- [ ] Output matches expected output
- [ ] All dependencies listed
- [ ] Tested on clean environment
- [ ] ROS 2 version specified
- [ ] File path correct
- [ ] Comments accurate
- [ ] Style guide followed

---

**Version History**:
- 1.0 (2026-02-09): Initial template created
