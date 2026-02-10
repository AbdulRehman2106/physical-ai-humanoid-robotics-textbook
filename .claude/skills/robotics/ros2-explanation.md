# Skill: ROS 2 Explanation

## Purpose
Explain ROS 2 (Robot Operating System 2) concepts, architecture, and patterns in a clear, pedagogically sound manner.

## Responsibility
Create accurate, accessible explanations of ROS 2 fundamentals, from basic concepts to advanced patterns, suitable for learners at various levels.

## When to Use
- Introducing ROS 2 concepts
- Explaining ROS 2 architecture
- Teaching communication patterns
- Describing lifecycle management
- Demonstrating best practices

## Core Capabilities

### 1. Foundational Concepts
- Explain what ROS 2 is and why it exists
- Describe the publish-subscribe pattern
- Introduce nodes, topics, services, and actions
- Explain the DDS middleware layer
- Describe the ROS 2 ecosystem

### 2. Architecture Explanation
- Describe the layered architecture
- Explain node composition
- Introduce the ROS 2 graph
- Describe discovery mechanisms
- Explain Quality of Service (QoS)

### 3. Communication Patterns
- Explain topics (pub-sub)
- Describe services (request-response)
- Introduce actions (goal-based)
- Compare patterns and use cases
- Demonstrate pattern selection

### 4. Lifecycle Management
- Explain managed nodes
- Describe lifecycle states
- Introduce state transitions
- Explain lifecycle benefits
- Demonstrate lifecycle usage

### 5. Best Practices
- Explain naming conventions
- Describe package organization
- Introduce launch files
- Explain parameter management
- Demonstrate testing approaches

## Explanation Framework

### Level 1: What It Is (Essence)
Start with the simplest accurate description:

**Example - ROS 2 Node**:
"A ROS 2 node is an independent program that performs a specific task in a robot system."

### Level 2: Why It Exists (Motivation)
Explain the problem it solves:

**Example - ROS 2 Node**:
"Robots need to do many things simultaneously—read sensors, plan paths, control motors. Nodes let you break this complexity into manageable, independent pieces that can run on different computers."

### Level 3: How It Works (Mechanism)
Describe the actual implementation:

**Example - ROS 2 Node**:
"Each node is a separate process with its own memory space. Nodes communicate by publishing and subscribing to topics, which are named channels for messages. The DDS middleware handles the actual message delivery between nodes."

### Level 4: When to Use It (Application)
Provide practical guidance:

**Example - ROS 2 Node**:
"Create a separate node for each major subsystem: one for sensor processing, one for path planning, one for motor control. This separation makes testing easier and allows you to run components on different machines."

## Key Concepts to Explain

### 1. Nodes
```markdown
## ROS 2 Nodes

**What**: Independent programs that perform specific tasks

**Why**: Breaking robot functionality into nodes makes systems:
- Easier to develop (work on one piece at a time)
- Easier to test (test components independently)
- More flexible (swap implementations without changing others)
- Distributed (run on multiple computers)

**How**: Each node:
- Runs as a separate process
- Has a unique name in the ROS 2 graph
- Communicates via topics, services, or actions
- Can be written in Python, C++, or other languages

**Example**:
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

**When to Use**:
- Create one node per major subsystem
- Keep nodes focused on single responsibilities
- Use multiple nodes for distributed systems
```

### 2. Topics (Publish-Subscribe)
```markdown
## ROS 2 Topics

**What**: Named channels for streaming data between nodes

**Why**: Topics enable:
- Decoupled communication (publishers don't know about subscribers)
- One-to-many broadcasting (multiple subscribers)
- Continuous data streams (sensor readings, state updates)

**Analogy**: Like a radio station—publishers broadcast on a frequency (topic name), and any number of radios (subscribers) can tune in.

**How**:
1. Publisher creates a topic with a name and message type
2. Publisher sends messages to the topic
3. Subscribers register interest in the topic
4. DDS middleware delivers messages to all subscribers

**Example**:
```python
# Publisher
publisher = node.create_publisher(String, 'robot_status', 10)
msg = String()
msg.data = 'Moving forward'
publisher.publish(msg)

# Subscriber
def callback(msg):
    print(f'Received: {msg.data}')

subscription = node.create_subscription(
    String, 'robot_status', callback, 10)
```

**When to Use**:
- Continuous data streams (sensor readings)
- State broadcasts (robot position)
- Event notifications (button presses)
- High-frequency updates (camera frames)
```

### 3. Services (Request-Response)
```markdown
## ROS 2 Services

**What**: Synchronous request-response communication

**Why**: Services are ideal for:
- Operations that need confirmation
- Configuration changes
- Queries that need answers
- Infrequent operations

**Analogy**: Like calling a function on another computer—you send a request and wait for a response.

**How**:
1. Service server advertises a service with a name and type
2. Client sends a request and blocks waiting for response
3. Server processes request and sends response
4. Client receives response and continues

**Example**:
```python
# Service Server
def handle_request(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(
    AddTwoInts, 'add_two_ints', handle_request)

# Service Client
client = node.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 5
request.b = 3
future = client.call_async(request)
# Wait for response...
```

**When to Use**:
- Configuration changes (set parameter)
- Queries (get current state)
- Infrequent operations (save map)
- Operations requiring confirmation
```

### 4. Quality of Service (QoS)
```markdown
## Quality of Service (QoS)

**What**: Policies that control message delivery behavior

**Why**: Different data has different requirements:
- Control commands must be reliable
- Sensor data can tolerate some loss
- Some data needs history buffering
- Some connections need durability

**Key Policies**:
- **Reliability**: Best-effort vs Reliable
- **Durability**: Volatile vs Transient-local
- **History**: Keep-last vs Keep-all
- **Deadline**: Maximum time between messages

**Example**:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable delivery for control commands
qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

publisher = node.create_publisher(
    Twist, '/cmd_vel', qos_reliable)

# Best-effort for high-frequency sensor data
qos_sensor = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)

subscription = node.create_subscription(
    LaserScan, '/scan', callback, qos_sensor)
```

**When to Use**:
- Reliable: Control commands, critical state
- Best-effort: High-frequency sensors, video
- Transient-local: Late-joining subscribers need history
- Deadline: Detect communication failures
```

### 5. Launch Files
```markdown
## ROS 2 Launch Files

**What**: Python scripts that start and configure multiple nodes

**Why**: Launch files enable:
- Starting entire robot systems with one command
- Configuring nodes with parameters
- Setting up node relationships
- Managing complex systems

**Example**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='sensor_node',
            name='lidar',
            parameters=[{'frequency': 10}]
        ),
        Node(
            package='my_package',
            executable='control_node',
            name='controller'
        )
    ])
```

**When to Use**:
- Starting multiple related nodes
- Configuring node parameters
- Setting up complex robot systems
- Creating reusable system configurations
```

## Common Misconceptions

### Misconception 1: "ROS 2 is an operating system"
**Reality**: ROS 2 is middleware—a set of libraries and tools for building robot applications. It runs on top of Linux, Windows, or macOS.

### Misconception 2: "Topics are faster than services"
**Reality**: Both use the same underlying DDS transport. Services add request-response semantics, which introduces minimal overhead. Choose based on communication pattern, not performance.

### Misconception 3: "All nodes must run on the same computer"
**Reality**: ROS 2 nodes can run on different computers on the same network. DDS handles discovery and communication automatically.

### Misconception 4: "QoS doesn't matter for simple robots"
**Reality**: Even simple robots benefit from appropriate QoS. Using reliable delivery for control commands prevents dangerous behavior from dropped messages.

## Quality Standards
- Explanations must be technically accurate
- Use analogies that illuminate, not mislead
- Provide concrete code examples
- Address common misconceptions
- Connect concepts to practical applications
- Build from simple to complex
- Maintain consistent terminology

## Integration Points
- Supports technical chapter writing
- Provides content for concept simplification
- Creates material for code examples
- Establishes foundation for robotics content
- Enables hands-on exercises
