# Chapter 4 Specification: ROS 2 Communication Patterns

**Chapter Number**: 4
**Title**: ROS 2 Communication Patterns
**Estimated Reading Time**: 65-80 minutes
**Difficulty Level**: Intermediate
**Prerequisites**: Chapter 3 (ROS 2 Fundamentals)

## Learning Outcomes

By the end of this chapter, students will be able to:

1. **Implement services** for request-response communication
2. **Create action servers and clients** for long-running tasks
3. **Configure Quality of Service (QoS)** policies for different scenarios
4. **Manage node lifecycle** for robust system behavior
5. **Design communication architectures** choosing appropriate patterns

## Chapter Structure

### 1. Introduction (250 words)
- Recap of Chapter 3 (topics and basic pub-sub)
- Why we need more communication patterns
- Chapter roadmap

### 2. Services: Request-Response Pattern (800 words)
- When to use services vs topics
- Creating a service server
- Creating a service client
- Standard service types
- Custom service definitions
- Error handling and timeouts

**Code Examples**:
- Service server (add two numbers)
- Service client (call the service)
- Async service client

### 3. Actions: Goal-Based Communication (900 words)
- When to use actions
- Action anatomy (goal, feedback, result)
- Creating an action server
- Creating an action client
- Canceling goals
- Handling feedback

**Code Examples**:
- Action server (countdown timer)
- Action client with feedback
- Action cancellation

### 4. Quality of Service (QoS) Deep Dive (700 words)
- QoS policies explained
- Reliability: Best effort vs reliable
- Durability: Transient local vs volatile
- History: Keep last vs keep all
- Lifespan and deadline
- QoS profiles for common scenarios

**Code Examples**:
- Sensor data QoS (best effort)
- Command QoS (reliable)
- Late-joiner scenario (transient local)

### 5. Node Lifecycle Management (600 words)
- Managed vs unmanaged nodes
- Lifecycle states (unconfigured, inactive, active, finalized)
- State transitions
- When to use lifecycle nodes
- Implementing lifecycle callbacks

**Code Examples**:
- Lifecycle node implementation
- Lifecycle state transitions

### 6. Practical Design Patterns (500 words)
- Choosing the right communication pattern
- Sensor data pipeline design
- Command and control architecture
- Multi-node coordination
- Best practices

### 7. Troubleshooting Guide (400 words)
- Common communication issues
- Debugging tools (ros2 topic, service, action)
- QoS mismatch problems
- Network issues

### 8. Summary and Next Steps (200 words)
- Key takeaways
- Preview of Chapter 5 (Simulation)

## Interactive Elements

1. **Callout Boxes**:
   - Info: When to use each communication pattern
   - Tip: QoS configuration best practices
   - Warning: Common QoS mismatch issues
   - Insight: Lifecycle management benefits

2. **CodePlayground Components**:
   - Service server and client
   - Action server and client
   - QoS configuration examples
   - Lifecycle node

3. **Interactive Diagram**:
   - Communication patterns comparison
   - QoS policy effects visualization

4. **Quiz** (6 questions):
   - Q1: When to use services vs topics?
   - Q2: What are the three parts of an action?
   - Q3: What is QoS reliability?
   - Q4: What are lifecycle states?
   - Q5: How to debug QoS mismatches?
   - Q6: Design pattern selection

5. **Checkpoint** (6 items):
   - I can implement services
   - I can create action servers
   - I understand QoS policies
   - I can configure QoS appropriately
   - I understand lifecycle management
   - I can choose appropriate patterns

## Code Examples Required

**Files to create**:
1. `add_service_server.py` - Service server example
2. `add_service_client.py` - Service client example
3. `countdown_action_server.py` - Action server
4. `countdown_action_client.py` - Action client
5. `qos_examples.py` - QoS configurations
6. `lifecycle_node.py` - Lifecycle management

## Citations Required

- ROS 2 Documentation (Services, Actions, QoS)
- DDS QoS specification
- Lifecycle management design docs

## Success Criteria

- Students can implement services and actions
- Students can configure QoS appropriately
- Students understand lifecycle management
- Students can choose appropriate patterns
- 80%+ pass rate on chapter quiz
- All code examples run without errors
