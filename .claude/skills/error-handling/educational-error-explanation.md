# Skill: Educational Error Explanation

## Purpose
Transform technical errors into learning opportunities by explaining what went wrong, why it happened, how to fix it, and how to prevent it.

## Responsibility
Create clear, educational error explanations that reduce frustration, build understanding, and empower learners to solve problems independently.

## When to Use
- Documenting common errors
- Creating troubleshooting guides
- Writing error messages
- Building diagnostic tools
- Designing error recovery flows

## Core Capabilities

### 1. Error Analysis
- Identify root cause of errors
- Distinguish symptoms from causes
- Categorize error types
- Assess error severity
- Determine error context

### 2. Educational Explanation
- Explain what the error means
- Describe why it occurred
- Provide clear fix instructions
- Suggest prevention strategies
- Build conceptual understanding

### 3. Diagnostic Guidance
- Provide step-by-step diagnosis
- Suggest verification methods
- Offer debugging strategies
- Create decision trees
- Enable self-service resolution

### 4. Context-Aware Help
- Tailor explanations to user level
- Provide relevant examples
- Link to related documentation
- Suggest next steps
- Offer escalation paths

### 5. Prevention Education
- Teach underlying concepts
- Explain best practices
- Highlight common pitfalls
- Build mental models
- Foster problem-solving skills

## Error Explanation Framework

### The Four-Part Structure

Every error explanation should include:

1. **WHAT**: What the error means (clear, jargon-free)
2. **WHY**: Why it happened (root cause)
3. **FIX**: How to fix it (step-by-step)
4. **PREVENT**: How to avoid it (best practices)

### Template

```markdown
## Error: [Error Name/Code]

### What Happened
[Clear, non-technical explanation of what went wrong]

### Why It Happened
[Root cause explanation with context]

### How to Fix It
1. [Step 1]
2. [Step 2]
3. [Step 3]

### Verify the Fix
[How to confirm the problem is resolved]

### How to Prevent This
- [Prevention strategy 1]
- [Prevention strategy 2]

### Understanding the Concept
[Brief explanation of underlying concept to build knowledge]

### Related Issues
- [Link to related error 1]
- [Link to related error 2]
```

## Error Categories

### 1. Configuration Errors
**Example**: ROS 2 package not found

```markdown
## Error: Package 'my_robot' not found

### What Happened
ROS 2 cannot find the package you're trying to use. This means the package either doesn't exist, isn't built, or isn't in ROS 2's search path.

### Why It Happened
Common causes:
- Package hasn't been built with `colcon build`
- Workspace not sourced (`. install/setup.bash`)
- Package name misspelled
- Package in different workspace

### How to Fix It

**Step 1: Check if package exists**
```bash
ls src/
# Look for your package directory
```

**Step 2: Build the package**
```bash
colcon build --packages-select my_robot
```

**Step 3: Source the workspace**
```bash
. install/setup.bash
```

**Step 4: Verify package is found**
```bash
ros2 pkg list | grep my_robot
```

### Verify the Fix
Run your original command. If the package is found, you'll see it execute without the error.

### How to Prevent This
- Always source your workspace: `. install/setup.bash`
- Add to `.bashrc` for automatic sourcing:
  ```bash
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
  ```
- Build packages before using them
- Use tab completion to verify package names

### Understanding the Concept
ROS 2 uses environment variables to locate packages. When you source `setup.bash`, it sets these variables. Without sourcing, ROS 2 doesn't know where to find your packages.
```

### 2. Runtime Errors
**Example**: Node crashes with segmentation fault

```markdown
## Error: Segmentation Fault (Core Dumped)

### What Happened
Your node crashed because it tried to access memory it doesn't own. This is a serious error that indicates a bug in your code.

### Why It Happened
Common causes:
- Accessing null pointer
- Array out of bounds
- Using deleted object
- Stack overflow
- Memory corruption

### How to Fix It

**Step 1: Enable core dumps**
```bash
ulimit -c unlimited
```

**Step 2: Run with debugger**
```bash
gdb --args ros2 run my_package my_node
(gdb) run
# Wait for crash
(gdb) backtrace
```

**Step 3: Identify the problem line**
The backtrace shows where the crash occurred. Look for:
- Null pointer dereferences: `if (ptr != nullptr)`
- Array bounds: Check indices are valid
- Object lifetime: Ensure objects exist when used

**Step 4: Add safety checks**
```cpp
// Before
data->value = 10;  // Crashes if data is null

// After
if (data != nullptr) {
    data->value = 10;
} else {
    RCLCPP_ERROR(get_logger(), "Data pointer is null!");
}
```

### Verify the Fix
Run your node again. It should execute without crashing.

### How to Prevent This
- Always check pointers before dereferencing
- Initialize pointers to nullptr
- Use smart pointers (std::shared_ptr, std::unique_ptr)
- Enable compiler warnings: `-Wall -Wextra`
- Use address sanitizer during development:
  ```bash
  colcon build --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=address"
  ```

### Understanding the Concept
Memory safety is critical in C++. Unlike Python, C++ doesn't automatically check if memory access is valid. You must ensure pointers are valid before using them.
```

### 3. Communication Errors
**Example**: No messages received on topic

```markdown
## Error: Subscriber Not Receiving Messages

### What Happened
Your subscriber node isn't receiving messages even though a publisher is sending them.

### Why It Happened
Common causes:
- QoS mismatch between publisher and subscriber
- Topic name mismatch (typo or namespace issue)
- Network configuration (different ROS_DOMAIN_ID)
- Publisher started after subscriber with incompatible QoS

### How to Fix It

**Step 1: Verify topic exists**
```bash
ros2 topic list
# Check if your topic appears
```

**Step 2: Check topic info**
```bash
ros2 topic info /my_topic
# Shows publishers and subscribers
```

**Step 3: Check QoS settings**
```bash
ros2 topic info /my_topic --verbose
# Shows QoS profiles
```

**Step 4: Fix QoS mismatch**
```python
# Publisher
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)
publisher = node.create_publisher(String, '/my_topic', qos)

# Subscriber (must match)
subscriber = node.create_subscription(
    String, '/my_topic', callback, qos)
```

### Verify the Fix
```bash
ros2 topic echo /my_topic
# Should show messages
```

### How to Prevent This
- Use consistent QoS profiles across your system
- Define QoS profiles in a shared configuration
- Use `ros2 topic info --verbose` to debug
- Test publisher and subscriber independently
- Use namespaces consistently

### Understanding the Concept
QoS (Quality of Service) policies must be compatible between publishers and subscribers. Reliable publishers can't communicate with best-effort subscribers. Understanding QoS is essential for robust ROS 2 systems.

### Related Issues
- [QoS Policy Mismatch](#)
- [Topic Name Resolution](#)
- [Network Configuration](#)
```

### 4. Environment Errors
**Example**: Simulation won't start

```markdown
## Error: Gazebo Failed to Start

### What Happened
Gazebo simulation environment failed to launch or crashed immediately after starting.

### Why It Happened
Common causes:
- Missing Gazebo installation
- Graphics driver issues
- Model path not set
- Port already in use
- Insufficient system resources

### How to Fix It

**Step 1: Check Gazebo installation**
```bash
gazebo --version
# Should show version number
```

**Step 2: Test Gazebo standalone**
```bash
gazebo --verbose
# Look for error messages
```

**Step 3: Check graphics**
```bash
glxinfo | grep "OpenGL"
# Verify OpenGL support
```

**Step 4: Set model path**
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/my_robot/models
```

**Step 5: Kill existing Gazebo processes**
```bash
killall gzserver gzclient
```

### Verify the Fix
```bash
ros2 launch my_robot simulation.launch.py
# Gazebo should start successfully
```

### How to Prevent This
- Install Gazebo properly: `sudo apt install ros-humble-gazebo-ros-pkgs`
- Update graphics drivers
- Add model paths to `.bashrc`
- Close Gazebo properly (don't kill forcefully)
- Allocate sufficient RAM (4GB minimum)

### Understanding the Concept
Gazebo is a complex 3D simulator that requires proper graphics support and system resources. It runs as two processes: gzserver (physics) and gzclient (visualization).
```

## Quality Standards

### Clarity
- Use plain language
- Avoid unnecessary jargon
- Provide concrete examples
- Show actual commands and code
- Include expected output

### Completeness
- Cover all four parts (What, Why, Fix, Prevent)
- Provide verification steps
- Link to related issues
- Offer escalation path
- Include conceptual explanation

### Actionability
- Give step-by-step instructions
- Show exact commands to run
- Provide copy-pasteable code
- Include verification methods
- Offer multiple solutions when applicable

### Educational Value
- Explain underlying concepts
- Build mental models
- Teach debugging skills
- Prevent future errors
- Foster independence

## Integration Points
- Supports technical content
- Enhances code examples
- Improves user experience
- Reduces support burden
- Builds learner confidence
