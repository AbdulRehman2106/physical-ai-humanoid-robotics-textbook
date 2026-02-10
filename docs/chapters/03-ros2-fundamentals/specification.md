# Chapter 3 Specification: ROS 2 Fundamentals

**Chapter Number**: 3
**Title**: ROS 2 Fundamentals
**Estimated Reading Time**: 60-75 minutes
**Difficulty Level**: Intermediate
**Prerequisites**: Chapter 1-2, Basic Python programming

## Learning Outcomes

By the end of this chapter, students will be able to:

1. **Explain ROS 2 architecture** and its key components (nodes, topics, services, actions)
2. **Create and run ROS 2 nodes** using Python
3. **Implement publisher-subscriber communication** for data exchange
4. **Understand the ROS 2 graph** and how nodes communicate
5. **Set up a ROS 2 development environment** and run basic examples

## Chapter Structure

### 1. Introduction (300 words)
- Transition from Physical AI theory to practical implementation
- Why ROS 2 is the industry standard
- What you'll build in this chapter
- Chapter roadmap

### 2. What is ROS 2? (500 words)
- Robot Operating System overview
- ROS 1 vs ROS 2 (why the upgrade matters)
- Key features: DDS, real-time, security
- ROS 2 distributions (Humble LTS)
- Use cases and industry adoption

**Visual Content**:
- ROS 2 ecosystem diagram
- ROS 1 vs ROS 2 comparison table

### 3. ROS 2 Architecture (700 words)
- Nodes: The building blocks
- Topics: Publish-subscribe messaging
- Services: Request-response patterns
- Actions: Long-running tasks with feedback
- Parameters: Runtime configuration
- The ROS 2 graph

**Visual Content**:
- ROS 2 architecture diagram
- ROS 2 graph visualization

### 4. Setting Up Your Environment (400 words)
- Installing ROS 2 Humble
- Workspace setup
- Environment configuration
- Testing the installation
- Docker alternative (optional)

**Code Examples**:
- Installation commands
- Workspace setup script
- Environment sourcing

### 5. Your First ROS 2 Node (800 words)
- Node anatomy
- Creating a minimal node
- Running and inspecting nodes
- Node lifecycle basics
- Best practices

**Code Examples**:
- Hello World node (minimal)
- Node with logging
- Running nodes with ros2 run

### 6. Publisher-Subscriber Pattern (900 words)
- When to use pub-sub
- Creating a publisher
- Creating a subscriber
- Message types
- Quality of Service (QoS) basics
- Running publisher and subscriber together

**Code Examples**:
- Simple publisher (string messages)
- Simple subscriber (string messages)
- Custom message types (preview)

### 7. Hands-On: Building a Sensor Node (600 words)
- Practical example: Simulated sensor
- Publishing sensor data
- Subscribing to sensor data
- Visualizing with rqt
- Debugging with ros2 topic

**Code Examples**:
- Sensor publisher node
- Sensor subscriber node
- Launch file for both nodes

### 8. Summary and Next Steps (200 words)
- Key takeaways
- Preview of Chapter 4 (communication patterns)
- Further resources

## Interactive Elements

1. **Callout Boxes**:
   - Info: ROS 2 vs ROS 1 key differences
   - Tip: Workspace organization best practices
   - Warning: Common installation issues
   - Insight: When to use topics vs services

2. **CodePlayground Components**:
   - Hello World node
   - Simple publisher
   - Simple subscriber
   - Sensor simulation node

3. **Interactive Diagram**:
   - ROS 2 graph (nodes and topics)
   - Message flow visualization

4. **Quiz** (6 questions):
   - Q1: What is a ROS 2 node?
   - Q2: What is the pub-sub pattern?
   - Q3: When to use topics vs services?
   - Q4: What is QoS?
   - Q5: How to run a ROS 2 node?
   - Q6: What is the ROS 2 graph?

5. **Checkpoint** (6 items):
   - I can explain ROS 2 architecture
   - I can create a basic ROS 2 node
   - I can implement a publisher
   - I can implement a subscriber
   - I understand the ROS 2 graph
   - I can debug ROS 2 nodes

## Code Examples Required

All examples must be:
- Complete and runnable
- Tested in ROS 2 Humble
- Well-commented
- Follow ROS 2 best practices

**Files to create**:
1. `hello_node.py` - Minimal node example
2. `simple_publisher.py` - String publisher
3. `simple_subscriber.py` - String subscriber
4. `sensor_publisher.py` - Simulated sensor
5. `sensor_subscriber.py` - Sensor data consumer
6. `sensor_launch.py` - Launch file for sensor demo

## Citations Required

- ROS 2 Documentation (Humble)
- Macenski, S., et al. (2022). Robot Operating System 2
- DDS specification references
- ROS 2 design documents

## Readability Target

- Flesch-Kincaid Grade Level: 10-12
- Code comments: Clear and educational
- Technical terms: Defined with examples
- Step-by-step instructions for setup

## Accessibility Requirements

- All code examples have descriptive titles
- Diagrams have detailed alt text
- Command-line instructions clearly formatted
- Error messages explained

## Success Criteria

- Students can install ROS 2 Humble
- Students can create and run a basic node
- Students can implement pub-sub communication
- Students can visualize the ROS 2 graph
- 80%+ pass rate on chapter quiz
- All code examples run without errors
