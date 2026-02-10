# Skill: Exercise and Assessment Design

## Purpose
Design effective exercises and assessments that reinforce learning, provide practice opportunities, and accurately measure mastery of learning outcomes.

## Responsibility
Create varied assessment types that balance formative and summative evaluation while supporting learning through practice and feedback.

## When to Use
- Creating practice exercises
- Designing quizzes and tests
- Building hands-on labs
- Creating project milestones
- Developing self-assessment tools

## Core Capabilities

### 1. Exercise Design
- Create practice problems
- Design hands-on activities
- Build coding challenges
- Develop debugging exercises
- Create design problems

### 2. Assessment Types
- Design multiple-choice questions
- Create short-answer prompts
- Build coding assessments
- Design project evaluations
- Develop peer review rubrics

### 3. Difficulty Calibration
- Match difficulty to learning stage
- Create progressive challenges
- Balance accessibility and rigor
- Provide scaffolding options
- Enable differentiation

### 4. Feedback Design
- Provide immediate feedback
- Create explanatory feedback
- Design hint systems
- Build solution walkthroughs
- Enable self-correction

### 5. Alignment Verification
- Map to learning outcomes
- Ensure construct validity
- Check difficulty appropriateness
- Verify coverage completeness
- Validate assessment fairness

## Exercise Types

### Type 1: Concept Check
**Purpose**: Verify basic understanding
**Format**: Multiple choice or short answer
**Timing**: After each major concept
**Difficulty**: Low to medium

**Example**:
```markdown
### Concept Check: ROS 2 Topics

**Question 1**: What is a ROS 2 topic?
- [ ] A) A type of ROS 2 node
- [ ] B) A named channel for streaming data
- [ ] C) A configuration file
- [ ] D) A debugging tool

**Correct Answer**: B

**Explanation**: A topic is a named channel that enables publish-subscribe communication between nodes. Publishers send messages to topics, and subscribers receive them.

**Question 2**: Can multiple subscribers receive messages from the same topic?
- [ ] A) Yes, topics support multiple subscribers
- [ ] B) No, only one subscriber per topic
- [ ] C) Only if using special QoS settings
- [ ] D) Only in simulation

**Correct Answer**: A

**Explanation**: Topics naturally support one-to-many communication. Any number of subscribers can listen to the same topic.
```

### Type 2: Code Completion
**Purpose**: Practice implementation skills
**Format**: Fill-in-the-blank code
**Timing**: During skill-building phase
**Difficulty**: Medium

**Example**:
```markdown
### Exercise: Complete the Publisher

Fill in the missing code to create a working ROS 2 publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # TODO: Create a publisher for the 'robot_status' topic
        # Message type: String, Queue size: 10
        self.publisher = ___________________________

        # TODO: Create a timer that fires every 1.0 seconds
        self.timer = ___________________________

    def timer_callback(self):
        msg = String()
        # TODO: Set the message data to "Robot is running"
        msg.data = ___________________________

        # TODO: Publish the message
        ___________________________

        self.get_logger().info(f'Published: {msg.data}')

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

**Solution**:
```python
self.publisher = self.create_publisher(String, 'robot_status', 10)
self.timer = self.create_timer(1.0, self.timer_callback)
msg.data = "Robot is running"
self.publisher.publish(msg)
```

**Verification**:
```bash
# Run your node
ros2 run my_package publisher_node

# In another terminal, check the topic
ros2 topic echo /robot_status
# Should see: data: 'Robot is running'
```
```

### Type 3: Debugging Challenge
**Purpose**: Develop troubleshooting skills
**Format**: Find and fix errors
**Timing**: After core concepts mastered
**Difficulty**: Medium to high

**Example**:
```markdown
### Debugging Challenge: Fix the Broken Subscriber

The following code should subscribe to a topic and print messages, but it doesn't work. Find and fix all the errors.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback
        )

    def listener_callback(msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = SubscriberNode()
    rclpy.shutdown()
```

**Errors to Find**:
1. Missing `self` parameter in `listener_callback`
2. Missing queue size in `create_subscription`
3. Missing `rclpy.spin(node)` - node exits immediately

**Corrected Code**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10  # Added queue size
        )

    def listener_callback(self, msg):  # Added self parameter
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = SubscriberNode()
    rclpy.spin(node)  # Added spin to keep node running
    rclpy.shutdown()
```

**Learning Points**:
- Callback methods need `self` parameter
- Subscriptions require queue size
- Nodes need `spin()` to process callbacks
```

### Type 4: Design Problem
**Purpose**: Apply concepts to new scenarios
**Format**: Open-ended design task
**Timing**: After multiple concepts learned
**Difficulty**: High

**Example**:
```markdown
### Design Challenge: Robot Communication Architecture

**Scenario**: You're building a mobile robot with the following components:
- Camera (publishes images at 30 Hz)
- Lidar (publishes scans at 10 Hz)
- IMU (publishes orientation at 100 Hz)
- Motor controller (receives velocity commands)
- Navigation planner (computes paths)
- Obstacle detector (processes sensor data)

**Task**: Design the ROS 2 communication architecture.

**Requirements**:
1. Identify all nodes needed
2. Define topics for communication
3. Specify message types
4. Choose appropriate QoS settings
5. Justify your design decisions

**Deliverable**: Create a diagram showing:
- All nodes (as circles)
- All topics (as arrows)
- Message types and QoS policies
- Brief justification for each choice

**Evaluation Criteria**:
- [ ] Appropriate node separation
- [ ] Correct communication patterns
- [ ] Suitable QoS choices
- [ ] Clear justification
- [ ] Scalable architecture

**Example Solution Approach**:
```
Nodes:
- camera_node: Publishes raw images
- lidar_node: Publishes laser scans
- imu_node: Publishes IMU data
- obstacle_detector_node: Subscribes to sensors, publishes obstacles
- navigation_node: Subscribes to obstacles, publishes paths
- motor_controller_node: Subscribes to velocity commands

Topics:
- /camera/image (sensor_msgs/Image, best-effort, depth=1)
- /scan (sensor_msgs/LaserScan, best-effort, depth=1)
- /imu (sensor_msgs/Imu, best-effort, depth=1)
- /obstacles (custom_msgs/ObstacleArray, reliable, depth=10)
- /cmd_vel (geometry_msgs/Twist, reliable, depth=10)

Justification:
- Sensor data uses best-effort (high frequency, some loss acceptable)
- Control commands use reliable (critical for safety)
- Separate nodes for modularity and testing
```
```

### Type 5: Hands-On Lab
**Purpose**: Build complete working system
**Format**: Multi-step guided project
**Timing**: End of chapter or module
**Difficulty**: Medium to high

**Example**:
```markdown
### Lab: Build a Simple Robot Controller

**Objective**: Create a ROS 2 system that reads keyboard input and controls a simulated robot.

**Time**: 60-90 minutes

**Prerequisites**:
- ROS 2 installed and configured
- Gazebo simulator installed
- Completed Chapters 3-4

**Part 1: Setup (10 minutes)**
1. Create a new package:
   ```bash
   ros2 pkg create robot_controller --build-type ament_python --dependencies rclpy geometry_msgs
   ```

2. Launch the robot simulation:
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

**Part 2: Create Keyboard Reader (20 minutes)**
Create `keyboard_reader.py`:
- Read keyboard input (w/a/s/d for movement)
- Publish to `/keyboard_commands` topic
- Use String messages

**Checkpoint 1**: Verify keyboard commands are published:
```bash
ros2 topic echo /keyboard_commands
```

**Part 3: Create Velocity Converter (20 minutes)**
Create `velocity_converter.py`:
- Subscribe to `/keyboard_commands`
- Convert to Twist messages
- Publish to `/cmd_vel`

**Checkpoint 2**: Verify velocity commands:
```bash
ros2 topic echo /cmd_vel
```

**Part 4: Test Complete System (10 minutes)**
1. Run all nodes
2. Press keys and observe robot movement
3. Verify smooth control

**Part 5: Enhancements (30 minutes)**
Add these features:
- [ ] Speed adjustment (faster/slower)
- [ ] Emergency stop (spacebar)
- [ ] Status display (current speed)
- [ ] Smooth acceleration/deceleration

**Submission**:
- Source code (all Python files)
- README with setup instructions
- Demo video (1-2 minutes)
- Reflection (what you learned, challenges faced)

**Grading Rubric**:
- Functionality (40%): System works as specified
- Code Quality (30%): Clean, commented, follows best practices
- Documentation (20%): Clear README and comments
- Enhancements (10%): Additional features implemented
```

## Assessment Design Principles

### Principle 1: Alignment
Every assessment must map to specific learning outcomes:

```markdown
## Assessment Alignment Matrix

| Assessment | Learning Outcome | Bloom's Level | Weight |
|------------|------------------|---------------|--------|
| Quiz 3.1 | LO-3.1: Explain pub-sub | Understand | 10% |
| Exercise 3.2 | LO-3.2: Create publisher | Apply | 20% |
| Lab 3.1 | LO-3.3: Build system | Create | 40% |
| Project | LO-3.1, 3.2, 3.3 | Evaluate | 30% |
```

### Principle 2: Progressive Difficulty
Assessments should increase in complexity:

```markdown
## Difficulty Progression

**Week 1**: Concept checks (remember/understand)
- Multiple choice questions
- Short definitions
- Simple code reading

**Week 2**: Application exercises (apply)
- Code completion
- Simple implementations
- Guided tutorials

**Week 3**: Problem-solving (analyze)
- Debugging challenges
- Design problems
- System analysis

**Week 4**: Integration projects (evaluate/create)
- Complete systems
- Original designs
- Capstone work
```

### Principle 3: Immediate Feedback
Provide feedback that promotes learning:

```markdown
## Feedback Types

### Correct Answer Feedback
"✅ Correct! Topics enable decoupled communication, which is essential for distributed robotics systems."

### Incorrect Answer Feedback
"❌ Not quite. While nodes do communicate, topics are specifically the channels they use, not the nodes themselves. Review Section 3.1 for clarification."

### Partial Credit Feedback
"⚠️ Partially correct. You identified the publisher correctly, but the queue size should be 10, not 1. A larger queue prevents message loss during processing delays."

### Hint System
- **Hint 1** (after 1 attempt): "Think about what creates the topic..."
- **Hint 2** (after 2 attempts): "Look at the create_publisher() method..."
- **Hint 3** (after 3 attempts): "The answer involves self.create_publisher()..."
```

### Principle 4: Authentic Assessment
Assess skills in realistic contexts:

```markdown
## Authentic Assessment Example

**Traditional Assessment**:
"Write a function that calculates the distance between two points."

**Authentic Assessment**:
"Your robot needs to navigate to a goal position. Implement the distance calculation function that the navigation system will use to determine when the robot has reached its destination. Your function must handle edge cases like negative coordinates and return values in meters."

**Why Better**:
- Real-world context
- Practical application
- Meaningful constraints
- Professional relevance
```

## Quality Standards

### Validity
- Assesses intended learning outcomes
- Measures what it claims to measure
- Appropriate difficulty level
- Fair and unbiased
- Authentic to domain

### Reliability
- Consistent results
- Clear scoring criteria
- Objective when possible
- Rubrics well-defined
- Inter-rater agreement

### Fairness
- Accessible to all learners
- Multiple demonstration paths
- Accommodations available
- Bias-free content
- Transparent expectations

### Educational Value
- Promotes learning
- Provides useful feedback
- Encourages reflection
- Builds confidence
- Prepares for next steps

## Integration Points
- Implements learning outcomes
- Reinforces technical content
- Provides practice opportunities
- Measures mastery
- Guides learning path
