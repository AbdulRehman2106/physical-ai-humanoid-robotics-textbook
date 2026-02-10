# Skill: Hands-On Tutorial Design

## Purpose
Design step-by-step tutorials that guide learners through practical implementations while building understanding and confidence.

## Responsibility
Create structured, actionable tutorials that balance guidance with exploration, enabling learners to build working systems while understanding underlying concepts.

## When to Use
- Creating implementation guides
- Designing lab exercises
- Building setup instructions
- Creating walkthroughs
- Designing practice projects

## Core Capabilities

### 1. Tutorial Structure
- Define clear objectives
- Break into logical steps
- Provide checkpoints
- Include verification
- Enable troubleshooting

### 2. Instruction Clarity
- Write clear, actionable steps
- Provide exact commands
- Show expected outputs
- Explain what's happening
- Anticipate questions

### 3. Scaffolding Balance
- Provide enough guidance
- Allow for exploration
- Enable problem-solving
- Support independence
- Fade support appropriately

### 4. Error Handling
- Anticipate common errors
- Provide diagnostic steps
- Offer solutions
- Explain why errors occur
- Prevent future mistakes

### 5. Learning Integration
- Connect to concepts
- Explain the "why"
- Build mental models
- Reinforce understanding
- Enable transfer

## Tutorial Template

```markdown
# Tutorial: [Title]

## What You'll Build
[Clear description of the end result with screenshot/diagram]

## What You'll Learn
- [Skill 1]
- [Skill 2]
- [Skill 3]

## Prerequisites
- [Required knowledge]
- [Required software]
- [Required hardware]

## Time Required
[Realistic estimate: X minutes]

## Overview
[Brief explanation of what you'll do and why it matters]

---

## Part 1: [Setup/Foundation]

### Step 1: [Action]
[Clear instruction with exact command or action]

```bash
[Exact command to run]
```

**Expected Output**:
```
[What you should see]
```

**What's Happening**: [Brief explanation]

**If Something Goes Wrong**:
- Problem: [Common issue]
  Solution: [How to fix]

### Step 2: [Next Action]
[Continue pattern]

**Checkpoint 1**: [Verification step]
```bash
[Command to verify]
```
You should see: [Expected result]

---

## Part 2: [Core Implementation]

### Step 3: [Implementation Step]
[Instruction]

**Code**:
```python
# Filename: example.py
[Complete, runnable code with comments]
```

**Understanding the Code**:
- Line X: [Explanation]
- Line Y: [Explanation]

### Step 4: [Build on Previous]
[Continue building]

**Checkpoint 2**: [Verification]

---

## Part 3: [Enhancement/Integration]

### Step 5: [Add Feature]
[Instruction]

### Step 6: [Test Complete System]
[Testing instructions]

**Final Checkpoint**: [Complete verification]

---

## What You've Accomplished
- ✅ [Achievement 1]
- ✅ [Achievement 2]
- ✅ [Achievement 3]

## Next Steps
- [Suggested enhancement 1]
- [Related tutorial]
- [Advanced topic]

## Troubleshooting
### Issue: [Common Problem]
**Symptoms**: [What you see]
**Solution**: [How to fix]

## Complete Code
[Link to full working code]

## Further Reading
- [Resource 1]
- [Resource 2]
```

## Tutorial Types

### Type 1: Quick Start Tutorial
**Purpose**: Get something working fast
**Length**: 10-15 minutes
**Depth**: Minimal explanation, maximum action

**Example**:
```markdown
# Quick Start: Your First ROS 2 Node

## Goal
Get a ROS 2 node running in 10 minutes.

## Steps

### 1. Create Package (2 min)
```bash
cd ~/ros2_ws/src
ros2 pkg create my_first_node --build-type ament_python --dependencies rclpy
```

### 2. Write Node (3 min)
Create `my_first_node/my_first_node/hello_node.py`:
```python
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')

def main():
    rclpy.init()
    node = HelloNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 3. Configure Entry Point (2 min)
Edit `setup.py`, add to `entry_points`:
```python
'hello_node = my_first_node.hello_node:main',
```

### 4. Build and Run (3 min)
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_node
. install/setup.bash
ros2 run my_first_node hello_node
```

**Success**: You should see "Hello, ROS 2!" in the terminal.

## What's Next?
Now that you have a working node, learn how to make it do something useful in the [Publisher Tutorial](#).
```

### Type 2: Deep Dive Tutorial
**Purpose**: Thorough understanding with implementation
**Length**: 45-60 minutes
**Depth**: Detailed explanations, multiple concepts

**Example**:
```markdown
# Deep Dive: Building a ROS 2 Publisher-Subscriber System

## What You'll Build
A complete pub-sub system where one node publishes sensor data and another processes it.

## What You'll Learn
- ROS 2 node architecture
- Publisher creation and configuration
- Subscriber implementation
- Message types and custom messages
- QoS policies and their effects
- Testing and debugging techniques

## Time: 60 minutes

---

## Part 1: Understanding the Architecture (10 min)

Before we code, let's understand what we're building:

[Detailed diagram and explanation]

**Key Concepts**:
1. **Decoupled Communication**: Publishers and subscribers don't know about each other
2. **Message Types**: Structured data definitions
3. **QoS Policies**: Control message delivery behavior

**Why This Matters**: This pattern enables distributed robotics systems where components can run on different machines.

---

## Part 2: Creating the Publisher (20 min)

### Step 1: Package Setup
[Detailed setup with explanations]

### Step 2: Understanding Publishers
[Conceptual explanation before implementation]

### Step 3: Implementation
[Code with detailed comments and explanations]

### Step 4: Testing
[How to verify it works]

**Checkpoint**: [Verification step]

---

## Part 3: Creating the Subscriber (20 min)
[Similar detailed approach]

---

## Part 4: Integration and Testing (10 min)
[Bringing it all together]

---

## Understanding What You Built
[Conceptual review and connections]

## Experiments to Try
1. Change the publish rate - what happens?
2. Start subscriber before publisher - does it work?
3. Add a second subscriber - do both receive messages?

## Common Issues and Solutions
[Comprehensive troubleshooting]
```

### Type 3: Project-Based Tutorial
**Purpose**: Build complete working project
**Length**: 2-4 hours (can be split)
**Depth**: Full implementation with design decisions

**Example**:
```markdown
# Project Tutorial: Build a Line-Following Robot

## Project Overview
Build a complete line-following robot system using ROS 2, including vision processing, control logic, and motor commands.

## Final Result
[Video/GIF of working robot]

## Skills You'll Develop
- Computer vision with OpenCV
- PID control implementation
- ROS 2 system integration
- Real-time processing
- Debugging robotics systems

## Time: 3-4 hours (can be done over multiple sessions)

---

## Session 1: Vision Processing (60 min)

### Understanding the Problem
[Explain line detection challenge]

### Design Decisions
[Discuss approach options and trade-offs]

### Implementation
[Step-by-step with explanations]

### Testing
[How to verify vision works]

**Checkpoint**: Vision node detects line position

---

## Session 2: Control System (60 min)
[Similar structure]

---

## Session 3: Integration (60 min)
[Bringing components together]

---

## Session 4: Tuning and Enhancement (60 min)
[Optimization and improvements]

---

## Project Reflection
- What worked well?
- What was challenging?
- How would you improve it?
- What did you learn?

## Extensions
[Ideas for taking it further]
```

## Tutorial Design Principles

### Principle 1: Show, Don't Just Tell
```markdown
<!-- Weak (just telling) -->
Create a publisher for the topic.

<!-- Strong (showing) -->
Create a publisher that sends String messages to the 'robot_status' topic:

```python
self.publisher = self.create_publisher(
    String,           # Message type
    'robot_status',   # Topic name
    10                # Queue size
)
```

This creates a publisher that can send text messages about the robot's status.
```

### Principle 2: Verify at Each Step
```markdown
## Step 3: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```

**Verify**: Check for success message:
```
Summary: 1 package finished [X.XXs]
```

**If you see errors**: [Troubleshooting steps]

**Before continuing**: Make sure the build succeeded. The next steps depend on this.
```

### Principle 3: Explain the Why
```markdown
## Step 4: Source the Workspace

```bash
. install/setup.bash
```

**Why this matters**: Sourcing sets up environment variables that tell ROS 2 where to find your package. Without this step, ROS 2 won't know your package exists.

**Pro tip**: Add this to your `~/.bashrc` to source automatically:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
```

### Principle 4: Anticipate Problems
```markdown
## Common Issues

### Issue: "Package not found"
**You'll see**: `Package 'my_package' not found`

**This means**: Either the package wasn't built, or the workspace wasn't sourced.

**Fix**:
1. Check if package was built: `ls install/my_package`
2. If missing, rebuild: `colcon build --packages-select my_package`
3. Source workspace: `. install/setup.bash`
4. Try again

### Issue: "No module named 'my_package'"
[Similar detailed troubleshooting]
```

## Quality Standards

### Clarity
- Every step is actionable
- Commands are exact and copy-pasteable
- Expected outputs are shown
- Explanations are clear
- No ambiguity

### Completeness
- All prerequisites listed
- All steps included
- Verification at checkpoints
- Troubleshooting provided
- Complete code available

### Educational Value
- Concepts explained
- Design decisions justified
- Mental models built
- Transfer enabled
- Understanding deepened

### Usability
- Easy to follow
- Logical progression
- Clear formatting
- Quick reference possible
- Searchable content

## Integration Points
- Implements learning outcomes
- Applies technical concepts
- Provides hands-on practice
- Builds confidence
- Enables skill development
