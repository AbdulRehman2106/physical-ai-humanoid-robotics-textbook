# Skill: Cognitive Load Management

## Purpose
Design content and interfaces that optimize cognitive load, enabling learners to process information effectively without overwhelming their working memory.

## Responsibility
Apply cognitive load theory principles to content structure, visual design, and interaction patterns to maximize learning effectiveness.

## When to Use
- Designing chapter structure
- Creating complex explanations
- Designing UI layouts
- Planning information architecture
- Optimizing learning sequences

## Core Capabilities

### 1. Load Type Identification
- Distinguish intrinsic, extraneous, and germane load
- Assess cognitive demands of content
- Identify load reduction opportunities
- Balance different load types
- Optimize total cognitive load

### 2. Chunking Strategy
- Break complex information into chunks
- Group related concepts
- Create meaningful units
- Establish chunk hierarchies
- Support progressive elaboration

### 3. Scaffolding Design
- Provide temporary support structures
- Fade support as mastery develops
- Create worked examples
- Design completion problems
- Enable gradual independence

### 4. Multimedia Optimization
- Apply dual coding theory
- Balance text and visuals
- Avoid redundancy
- Optimize modality
- Reduce split attention

### 5. Progressive Disclosure
- Reveal information incrementally
- Match disclosure to readiness
- Provide expansion options
- Support different learning paces
- Enable review and reinforcement

## Cognitive Load Types

### Intrinsic Load
**Definition**: Inherent difficulty of the material

**Cannot be reduced**, but can be managed:
- Ensure prerequisites are met
- Build from simple to complex
- Use analogies to leverage prior knowledge
- Break into smaller learning units
- Provide conceptual scaffolding

**Example**:
```markdown
<!-- High intrinsic load topic -->
## Understanding Inverse Kinematics

**Prerequisites** (reduce load by ensuring readiness):
- Forward kinematics (Chapter 4)
- Matrix transformations (Chapter 2)
- Trigonometry (assumed)

**Scaffolding** (manage intrinsic load):
1. Start with 2D, 2-joint arm (simpler)
2. Build intuition with interactive visualization
3. Progress to 3D, multi-joint (more complex)
4. Introduce optimization methods
```

### Extraneous Load
**Definition**: Load from poor instructional design

**Should be minimized**:
- Eliminate unnecessary information
- Reduce visual clutter
- Avoid split attention
- Remove redundant explanations
- Simplify navigation

**Example**:
```markdown
<!-- High extraneous load (BAD) -->
## ROS 2 Topics

Topics are named channels. They use DDS. DDS is Data Distribution Service.
It's middleware. Middleware sits between OS and application. Topics enable
pub-sub. Pub-sub is publish-subscribe. Publishers send messages. Messages
are data structures. Subscribers receive messages...

<!-- Low extraneous load (GOOD) -->
## ROS 2 Topics

A **topic** is a named channel for streaming data between nodes.

Think of it like a radio station: publishers broadcast on a frequency
(topic name), and subscribers tune in to receive the broadcast.

[Diagram showing publisher → topic → subscriber]

**Key concept**: Topics enable decoupled communication—publishers and
subscribers don't need to know about each other.
```

### Germane Load
**Definition**: Load devoted to learning and schema construction

**Should be maximized**:
- Encourage active processing
- Prompt reflection
- Provide practice opportunities
- Support schema building
- Enable knowledge integration

**Example**:
```markdown
## Understanding ROS 2 Communication Patterns

**Active Learning** (increase germane load):

1. **Compare and Contrast**:
   Create a table comparing topics, services, and actions.
   When would you use each?

2. **Apply to Scenario**:
   Design the communication architecture for a delivery robot.
   Which patterns would you use for:
   - Sensor data?
   - Navigation commands?
   - Task requests?

3. **Reflect**:
   How does the pub-sub pattern enable distributed robotics?
   What would be difficult without it?
```

## Chunking Strategies

### The 7±2 Rule
**Principle**: Working memory holds 7±2 chunks

**Application**:
- Limit lists to 5-7 items
- Group related information
- Create meaningful categories
- Use hierarchical organization

**Example**:
```markdown
<!-- Too many items (BAD) -->
## ROS 2 Concepts
- Nodes
- Topics
- Publishers
- Subscribers
- Services
- Clients
- Servers
- Actions
- Action clients
- Action servers
- Parameters
- Launch files

<!-- Chunked appropriately (GOOD) -->
## ROS 2 Concepts

### Communication Patterns (3 chunks)
1. **Topics**: Continuous data streams (pub-sub)
2. **Services**: Request-response interactions
3. **Actions**: Long-running tasks with feedback

### System Components (3 chunks)
1. **Nodes**: Independent processes
2. **Parameters**: Configuration values
3. **Launch Files**: System startup scripts
```

### Progressive Chunking
```markdown
## Learning ROS 2 Topics

### Level 1: Core Concept (1 chunk)
Topics are named channels for data streaming.

### Level 2: Key Components (3 chunks)
1. **Publisher**: Sends messages to a topic
2. **Topic**: Named channel with message type
3. **Subscriber**: Receives messages from a topic

### Level 3: Implementation Details (expand each chunk)

#### Publishers
- Created with `create_publisher()`
- Specify message type and topic name
- Use `publish()` to send messages
- Configure QoS for reliability

[Continue expanding as learner progresses]
```

## Scaffolding Patterns

### Worked Example → Completion Problem → Independent Practice

```markdown
## Learning to Create ROS 2 Publishers

### Step 1: Worked Example (Full support)
```python
# Complete example with detailed comments
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        # Create publisher: message type, topic name, queue size
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        # Create timer: period in seconds, callback function
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # Create message
        msg = String()
        msg.data = f'Hello {self.count}'
        # Publish message
        self.publisher.publish(msg)
        self.count += 1

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Step 2: Completion Problem (Partial support)
```python
# Fill in the blanks
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        # TODO: Create publisher for 'robot_status' topic
        self.publisher = _______________
        # TODO: Create timer that fires every 0.5 seconds
        self.timer = _______________

    def timer_callback(self):
        msg = String()
        # TODO: Set message data to current robot status
        msg.data = _______________
        # TODO: Publish the message
        _______________
```

### Step 3: Independent Practice (No support)
```markdown
**Exercise**: Create a publisher that sends robot velocity commands

Requirements:
- Topic name: '/cmd_vel'
- Message type: Twist
- Publish rate: 10 Hz
- Content: Linear velocity of 0.5 m/s forward
```
```

## Multimedia Principles

### Dual Coding Theory
**Principle**: Combine verbal and visual information

```markdown
<!-- Text only (single channel) -->
ROS 2 uses a publish-subscribe pattern where publishers send messages
to topics and subscribers receive them. Multiple subscribers can listen
to the same topic.

<!-- Text + Diagram (dual coding) -->
## Publish-Subscribe Pattern

[Diagram showing:
- Publisher node (circle)
- Topic (channel)
- Multiple subscriber nodes (circles)
- Arrows showing message flow]

Publishers send messages to named topics. Any number of subscribers
can receive these messages by subscribing to the topic.
```

### Avoid Split Attention
**Principle**: Integrate related information

```markdown
<!-- Split attention (BAD) -->
[Code block]
```python
publisher = node.create_publisher(String, 'topic', 10)
```

See explanation below for parameter meanings.

[Several paragraphs of other content]

**Parameters**:
- First: Message type
- Second: Topic name
- Third: Queue size

<!-- Integrated (GOOD) -->
```python
publisher = node.create_publisher(
    String,    # Message type: what kind of data
    'topic',   # Topic name: where to publish
    10         # Queue size: how many messages to buffer
)
```
```

### Avoid Redundancy
**Principle**: Don't present same information in multiple modalities simultaneously

```markdown
<!-- Redundant (BAD) -->
[Diagram with extensive labels]
[Text repeating everything in the diagram]

<!-- Optimized (GOOD) -->
[Diagram with clear labels]

The diagram shows how messages flow from publishers through topics
to subscribers. Notice how multiple subscribers can receive the
same message.
```

## Progressive Disclosure Patterns

### Expandable Sections
```markdown
## ROS 2 Quality of Service (QoS)

### Basic Understanding
QoS policies control how messages are delivered between publishers
and subscribers.

<details>
<summary>📖 Learn More: QoS Policies</summary>

QoS includes several policies:
- **Reliability**: Best-effort vs Reliable
- **Durability**: Volatile vs Transient-local
- **History**: Keep-last vs Keep-all
- **Deadline**: Maximum time between messages

</details>

<details>
<summary>🔧 Advanced: Custom QoS Profiles</summary>

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)
```
</details>
```

### Layered Explanations
```markdown
## Understanding Coordinate Transformations

### 🟢 Beginner: The Big Picture
Robots need to convert positions between different reference frames
(like converting between your left hand and your right hand).

### 🟡 Intermediate: How It Works
Transformations use rotation matrices and translation vectors to
convert coordinates from one frame to another.

### 🔴 Advanced: Mathematical Details
[Detailed matrix mathematics and derivations]
```

## Quality Standards

### Cognitive Load Optimization
- Intrinsic load matched to learner level
- Extraneous load minimized
- Germane load maximized
- Total load within working memory capacity

### Information Architecture
- Chunked appropriately (5-7 items)
- Hierarchically organized
- Progressively disclosed
- Logically sequenced

### Multimedia Design
- Text and visuals integrated
- Split attention avoided
- Redundancy eliminated
- Modality optimized

### Scaffolding Effectiveness
- Support appropriate to level
- Fading strategy clear
- Independence gradually achieved
- Practice opportunities provided

## Integration Points
- Informs content authoring
- Guides UI/UX design
- Shapes navigation patterns
- Optimizes learning sequences
- Enhances comprehension
