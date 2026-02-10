# Skill: Troubleshooting Guide Creation

## Purpose
Create comprehensive troubleshooting guides that help learners diagnose and resolve issues independently through systematic problem-solving approaches.

## Responsibility
Design diagnostic workflows, decision trees, and troubleshooting procedures that empower learners to identify and fix problems efficiently.

## When to Use
- Creating support documentation
- Building diagnostic tools
- Designing error recovery flows
- Documenting common issues
- Enabling self-service support

## Core Capabilities

### 1. Problem Categorization
- Identify common problem types
- Group related issues
- Establish problem hierarchies
- Define symptom patterns
- Create issue taxonomies

### 2. Diagnostic Workflow Design
- Create decision trees
- Design step-by-step diagnostics
- Establish verification points
- Define escalation paths
- Build systematic approaches

### 3. Solution Documentation
- Provide clear fix instructions
- Include verification steps
- Offer multiple solutions
- Explain why fixes work
- Prevent recurrence

### 4. Tool Integration
- Leverage diagnostic commands
- Use logging and debugging tools
- Integrate monitoring systems
- Provide automated checks
- Enable self-diagnosis

### 5. Knowledge Building
- Teach diagnostic thinking
- Build mental models
- Explain root causes
- Foster independence
- Develop problem-solving skills

## Troubleshooting Guide Structure

```markdown
# Troubleshooting Guide: [System/Component]

## Quick Diagnostic

### Is your issue here?
- [ ] [Common symptom 1] → [Jump to section]
- [ ] [Common symptom 2] → [Jump to section]
- [ ] [Common symptom 3] → [Jump to section]
- [ ] None of the above → [Start systematic diagnosis]

## Systematic Diagnosis

### Step 1: Verify Basic Setup
**Check**: [What to verify]
**Command**:
```bash
[Diagnostic command]
```
**Expected Output**: [What you should see]
**If different**: [What it means and where to go]

### Step 2: Check Dependencies
[Repeat structure]

### Step 3: Verify Configuration
[Repeat structure]

## Common Issues

### Issue 1: [Problem Name]

**Symptoms**:
- [Observable symptom 1]
- [Observable symptom 2]

**Diagnosis**:
```bash
# Check for this issue
[diagnostic command]
```

**Root Cause**: [Why this happens]

**Solution**:
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Verify Fix**:
```bash
[verification command]
```

**Prevention**: [How to avoid this]

---

### Issue 2: [Problem Name]
[Repeat structure]

## Decision Tree

```
Problem: Robot not responding
    ↓
Is ROS 2 running?
    ├─ No → Start ROS 2 → [Instructions]
    └─ Yes
        ↓
    Are nodes visible?
        ├─ No → Check network → [Instructions]
        └─ Yes
            ↓
        Are messages flowing?
            ├─ No → Check QoS → [Instructions]
            └─ Yes → Check application logic
```

## Advanced Diagnostics

### Using Logs
[How to access and interpret logs]

### Using Debugging Tools
[How to use debuggers and profilers]

### Performance Analysis
[How to diagnose performance issues]

## Getting Help

### Before Asking
- [ ] Followed all diagnostic steps
- [ ] Collected relevant logs
- [ ] Documented what you tried
- [ ] Identified exact error messages

### Where to Ask
- [Forum/Discord link]
- [GitHub Issues link]
- [Stack Overflow tag]

### How to Ask
[Template for effective questions]
```

## Example: ROS 2 Troubleshooting Guide

```markdown
# ROS 2 Troubleshooting Guide

## Quick Diagnostic

### Is your issue here?
- [ ] Nodes not communicating → [Jump to Communication Issues](#communication-issues)
- [ ] Package not found → [Jump to Package Issues](#package-issues)
- [ ] Build errors → [Jump to Build Issues](#build-issues)
- [ ] Runtime crashes → [Jump to Runtime Issues](#runtime-issues)

## Systematic Diagnosis

### Step 1: Verify ROS 2 Installation

**Check**: ROS 2 is installed and sourced

```bash
# Check ROS 2 version
ros2 --version

# Check if environment is sourced
echo $ROS_DISTRO
```

**Expected Output**:
```
ros2 --version: ros2 cli version 0.x.x
$ROS_DISTRO: humble (or your distro)
```

**If different**:
- No output → ROS 2 not installed → [Install ROS 2](link)
- ROS_DISTRO empty → Not sourced → Run `. /opt/ros/humble/setup.bash`

### Step 2: Verify Workspace Setup

**Check**: Workspace is built and sourced

```bash
# Check if workspace exists
ls ~/ros2_ws/src

# Check if built
ls ~/ros2_ws/install

# Check if sourced
echo $AMENT_PREFIX_PATH
```

**Expected Output**: Should show workspace paths

**If different**:
- src/ doesn't exist → Create workspace → [Instructions](link)
- install/ doesn't exist → Build workspace → `colcon build`
- AMENT_PREFIX_PATH doesn't include workspace → Source it → `. install/setup.bash`

### Step 3: Verify Network Configuration

**Check**: ROS_DOMAIN_ID is set correctly

```bash
echo $ROS_DOMAIN_ID
```

**Expected Output**: A number (0-101) or empty (defaults to 0)

**If different**: Nodes on different domains can't communicate

## Common Issues

### Communication Issues

#### Issue: Nodes Not Seeing Each Other

**Symptoms**:
- `ros2 node list` doesn't show expected nodes
- `ros2 topic list` missing topics
- Subscribers not receiving messages

**Diagnosis**:
```bash
# Check if nodes are running
ros2 node list

# Check if topics exist
ros2 topic list

# Check topic info
ros2 topic info /my_topic --verbose
```

**Root Causes**:
1. Different ROS_DOMAIN_ID
2. Network firewall blocking DDS
3. QoS mismatch
4. Nodes not actually running

**Solutions**:

**Solution 1: Check ROS_DOMAIN_ID**
```bash
# On all machines, check domain
echo $ROS_DOMAIN_ID

# Set to same value
export ROS_DOMAIN_ID=0
```

**Solution 2: Check Firewall**
```bash
# Ubuntu: Allow DDS ports
sudo ufw allow from 224.0.0.0/4
sudo ufw allow from 239.0.0.0/8
```

**Solution 3: Check QoS**
```bash
# View QoS settings
ros2 topic info /my_topic --verbose

# Ensure publisher and subscriber QoS are compatible
```

**Verify Fix**:
```bash
# Should see both publisher and subscriber
ros2 topic info /my_topic

# Should receive messages
ros2 topic echo /my_topic
```

**Prevention**:
- Set ROS_DOMAIN_ID in .bashrc
- Use consistent QoS profiles
- Document network requirements
- Test communication early

---

#### Issue: QoS Mismatch

**Symptoms**:
- Topic exists but no messages received
- `ros2 topic info` shows publisher and subscriber
- No error messages

**Diagnosis**:
```bash
# Check QoS settings
ros2 topic info /my_topic --verbose
```

**Root Cause**: Incompatible QoS policies between publisher and subscriber

**QoS Compatibility Rules**:
- Reliable publisher ✅ Reliable subscriber
- Reliable publisher ❌ Best-effort subscriber
- Best-effort publisher ✅ Best-effort subscriber
- Best-effort publisher ✅ Reliable subscriber (with warnings)

**Solution**:
```python
# Make QoS compatible
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Option 1: Both use reliable
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

# Option 2: Both use best-effort
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

# Use same QoS for publisher and subscriber
publisher = node.create_publisher(String, '/topic', qos)
subscription = node.create_subscription(String, '/topic', callback, qos)
```

**Verify Fix**:
```bash
ros2 topic echo /my_topic
# Should now receive messages
```

**Prevention**:
- Define QoS profiles in shared config
- Document QoS requirements
- Use system default when possible
- Test with `ros2 topic echo`

### Package Issues

#### Issue: Package Not Found

[Full diagnostic as shown in previous skill]

### Build Issues

#### Issue: Colcon Build Fails

**Symptoms**:
- `colcon build` exits with errors
- Missing dependencies
- Compilation errors

**Diagnosis**:
```bash
# Build with verbose output
colcon build --event-handlers console_direct+

# Build single package
colcon build --packages-select my_package
```

**Common Causes**:
1. Missing dependencies
2. Syntax errors in code
3. CMakeLists.txt errors
4. package.xml errors

**Solutions**: [Detailed solutions for each cause]

## Decision Tree

```
Problem: ROS 2 System Not Working
    ↓
Can you run `ros2 --version`?
    ├─ No → ROS 2 not installed/sourced
    │       ├─ Install ROS 2
    │       └─ Source setup.bash
    └─ Yes
        ↓
    Can you see your nodes with `ros2 node list`?
        ├─ No → Nodes not running or network issue
        │       ├─ Check if nodes started
        │       ├─ Check ROS_DOMAIN_ID
        │       └─ Check firewall
        └─ Yes
            ↓
        Can you see topics with `ros2 topic list`?
            ├─ No → Topics not created
            │       └─ Check node implementation
            └─ Yes
                ↓
            Do messages flow with `ros2 topic echo`?
                ├─ No → QoS mismatch or publisher issue
                │       ├─ Check QoS compatibility
                │       └─ Verify publisher is publishing
                └─ Yes → Issue is in application logic
```

## Advanced Diagnostics

### Using ROS 2 Introspection Tools

```bash
# View node details
ros2 node info /my_node

# Monitor topic bandwidth
ros2 topic bw /my_topic

# Monitor topic frequency
ros2 topic hz /my_topic

# View service list
ros2 service list

# Call service manually
ros2 service call /my_service std_srvs/srv/Trigger
```

### Using Logs

```bash
# View node logs
ros2 run my_package my_node --ros-args --log-level debug

# Save logs to file
ros2 run my_package my_node --ros-args --log-level debug 2>&1 | tee log.txt
```

### Using Debugger

```bash
# Run with GDB
gdb --args ros2 run my_package my_node

# Run with Valgrind
valgrind ros2 run my_package my_node
```

## Getting Help

### Information to Collect

Before asking for help, gather:
- ROS 2 distro: `echo $ROS_DISTRO`
- OS version: `lsb_release -a`
- Exact error message
- Steps to reproduce
- What you've tried
- Relevant code snippets

### Where to Get Help

- **ROS Answers**: https://answers.ros.org
- **ROS Discourse**: https://discourse.ros.org
- **GitHub Issues**: For package-specific issues
- **Discord/Slack**: Community channels

### How to Ask Effectively

```markdown
**Problem**: [One-sentence description]

**Environment**:
- ROS 2 Distro: humble
- OS: Ubuntu 22.04
- Hardware: [if relevant]

**What I'm trying to do**:
[Clear description of goal]

**What's happening**:
[Actual behavior with error messages]

**What I've tried**:
1. [Step 1]
2. [Step 2]

**Code**:
```python
[Minimal reproducible example]
```

**Logs**:
```
[Relevant log output]
```
```
```

## Quality Standards

### Completeness
- Covers common issues comprehensively
- Provides systematic diagnostic approach
- Includes verification steps
- Offers multiple solutions
- Explains root causes

### Usability
- Clear, step-by-step instructions
- Copy-pasteable commands
- Expected outputs shown
- Decision points clear
- Quick reference available

### Educational Value
- Teaches diagnostic thinking
- Explains why problems occur
- Builds troubleshooting skills
- Fosters independence
- Prevents future issues

### Maintainability
- Easy to update
- Modular structure
- Version-specific notes
- Links to official docs
- Community contributions welcome

## Integration Points
- Supports error explanation skill
- Enhances technical content
- Reduces support burden
- Builds learner confidence
- Enables self-service resolution
