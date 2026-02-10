# Skill: Code Example Design

## Purpose
Create effective, pedagogical code examples that teach concepts clearly while demonstrating best practices and real-world patterns.

## Responsibility
Design code examples that are complete, runnable, well-commented, and aligned with learning objectives, balancing simplicity with practical relevance.

## When to Use
- Demonstrating technical concepts in chapters
- Teaching programming patterns and idioms
- Showing practical implementation of theory
- Providing hands-on learning opportunities

## Core Capabilities

### 1. Example Purpose Definition
- Identify the specific learning objective
- Determine what the example should demonstrate
- Establish the appropriate complexity level
- Define success criteria for the example
- Connect to broader chapter narrative

### 2. Code Structure Design
- Create minimal, focused examples
- Remove unnecessary complexity
- Highlight the key concept clearly
- Use meaningful variable and function names
- Follow language/framework conventions

### 3. Progressive Complexity
- Start with simplest working example
- Build complexity incrementally
- Show evolution of solutions
- Demonstrate refactoring and improvement
- Connect simple examples to production patterns

### 4. Annotation Strategy
- Add comments that explain "why" not "what"
- Highlight key lines or patterns
- Explain non-obvious decisions
- Point out common pitfalls
- Reference related concepts

### 5. Context Integration
- Provide setup instructions
- Show expected output
- Explain how to run the code
- Demonstrate testing approach
- Connect to real-world use cases

## Code Example Template

```markdown
### Example: [Descriptive Name]

**Learning Objective**: [What this example teaches]

**Scenario**: [Real-world context for the example]

#### Setup
[Prerequisites, dependencies, environment setup]

#### Implementation

```[language]
# [Brief description of what this code does]

[Code with strategic comments]

# Key concept: [Explanation of important pattern or decision]
[Code demonstrating the concept]

# Note: [Important caveat or consideration]
[Related code]
```

#### Running the Example
```bash
[Commands to execute the code]
```

#### Expected Output
```
[What the learner should see]
```

#### Understanding the Code
[Line-by-line or section-by-section explanation]

1. **[Section 1]**: [What it does and why]
2. **[Section 2]**: [What it does and why]

#### Try It Yourself
[Suggested modifications or experiments]
- [ ] [Modification 1]
- [ ] [Modification 2]

#### Common Issues
- **[Issue 1]**: [Cause and solution]
- **[Issue 2]**: [Cause and solution]
```

## Example Types

### 1. Minimal Demonstration
**Purpose**: Show the absolute simplest version of a concept
**Characteristics**:
- 5-15 lines of code
- Single concept focus
- No error handling or edge cases
- Clear, obvious behavior

**Example Structure**:
```python
# Minimal example: Publishing a ROS 2 message

import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('minimal_publisher')
publisher = node.create_publisher(String, 'topic', 10)

msg = String()
msg.data = 'Hello, ROS 2!'
publisher.publish(msg)
```

### 2. Practical Implementation
**Purpose**: Show realistic, production-ready code
**Characteristics**:
- 30-100 lines of code
- Error handling included
- Best practices demonstrated
- Real-world patterns

### 3. Comparative Example
**Purpose**: Show different approaches to the same problem
**Characteristics**:
- Side-by-side implementations
- Explicit trade-off discussion
- Performance or clarity comparisons
- Guidance on when to use each

### 4. Incremental Build
**Purpose**: Show evolution from simple to complex
**Characteristics**:
- Multiple versions of the same code
- Each version adds one concept
- Clear progression narrative
- Refactoring demonstrated

### 5. Anti-Pattern Warning
**Purpose**: Show what NOT to do and why
**Characteristics**:
- Clearly labeled as incorrect
- Explanation of the problem
- Correct alternative provided
- Real-world consequences explained

## Code Quality Standards

### Readability
- Use descriptive names (not `x`, `temp`, `data`)
- Follow consistent formatting
- Keep functions focused and short
- Use whitespace for visual grouping
- Avoid clever tricks that obscure meaning

### Completeness
- Include all necessary imports
- Provide required setup code
- Show complete, runnable examples
- Include cleanup/teardown when needed
- Demonstrate proper resource management

### Pedagogy
- Focus on one concept at a time
- Remove distracting details
- Highlight the learning point
- Provide context before code
- Explain outcomes after code

### Accuracy
- Test all examples thoroughly
- Use current API versions
- Follow official documentation
- Acknowledge platform differences
- Update examples when APIs change

## Comment Guidelines

### Good Comments
```python
# Create a publisher for robot velocity commands
# QoS profile ensures reliable delivery for control messages
publisher = node.create_publisher(
    Twist,
    '/cmd_vel',
    qos_profile=qos_profile_system_default
)
```

### Bad Comments
```python
# Create publisher
publisher = node.create_publisher(Twist, '/cmd_vel', 10)
```

### Strategic Comment Placement
- **Before code block**: What and why
- **Inline**: Non-obvious decisions
- **After code block**: Expected behavior or outcome
- **Callout**: Important patterns or pitfalls

## Progressive Example Series

### Level 1: Hello World
```python
# Simplest possible version
print("Hello, Robot!")
```

### Level 2: Add Structure
```python
# Organized into a function
def greet_robot():
    print("Hello, Robot!")

greet_robot()
```

### Level 3: Add Parameters
```python
# Configurable greeting
def greet_robot(name):
    print(f"Hello, {name}!")

greet_robot("Atlas")
```

### Level 4: Add Error Handling
```python
# Production-ready version
def greet_robot(name):
    if not name:
        raise ValueError("Robot name cannot be empty")
    print(f"Hello, {name}!")

try:
    greet_robot("Atlas")
except ValueError as e:
    print(f"Error: {e}")
```

## Integration Points
- Supports technical chapter writing
- Demonstrates learning outcomes
- Provides material for UI code blocks
- Creates opportunities for interactive elements
- Establishes patterns for error handling
