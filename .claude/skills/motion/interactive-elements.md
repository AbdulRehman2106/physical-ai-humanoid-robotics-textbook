# Skill: Interactive Element Design

## Purpose
Design interactive elements that engage learners, provide immediate feedback, and enable active exploration of concepts.

## Responsibility
Create specifications for interactive components that enhance learning through hands-on experimentation and dynamic visualization.

## When to Use
- Designing interactive diagrams
- Creating code playgrounds
- Building concept explorers
- Designing simulation interfaces
- Creating interactive quizzes

## Core Capabilities

### 1. Interaction Pattern Design
- Define user interactions
- Specify feedback mechanisms
- Design state management
- Create interaction flows
- Enable exploration

### 2. Real-Time Feedback
- Provide immediate responses
- Show cause-and-effect
- Visualize changes
- Highlight errors
- Celebrate success

### 3. Parameter Exploration
- Enable value adjustment
- Show parameter effects
- Provide sensible ranges
- Display current state
- Reset to defaults

### 4. Progressive Complexity
- Start simple
- Add features incrementally
- Provide guided exploration
- Enable advanced modes
- Support different skill levels

### 5. Learning Integration
- Connect to concepts
- Reinforce understanding
- Enable discovery
- Support experimentation
- Provide context

## Interactive Component Types

### Type 1: Parameter Explorer
**Purpose**: Understand how parameters affect behavior

**Specification**:
```markdown
## Interactive: ROS 2 QoS Explorer

**Concept**: Quality of Service policies affect message delivery

**Interface**:
- Sliders for parameters:
  - Reliability: [Best Effort] ←→ [Reliable]
  - History Depth: 1 ←→ 100
  - Deadline: 0ms ←→ 1000ms
- Visualization:
  - Publisher node (left)
  - Subscriber node (right)
  - Messages flowing between them
  - Dropped messages shown in red
  - Delivered messages shown in green

**Interactions**:
1. Adjust reliability slider
   - Show effect on message delivery
   - Display delivery percentage
   - Highlight dropped messages

2. Adjust history depth
   - Show queue filling
   - Demonstrate overflow behavior
   - Display queue state

3. Adjust deadline
   - Show timeout warnings
   - Highlight late messages
   - Display timing statistics

**Learning Goals**:
- Understand QoS policy effects
- See trade-offs between reliability and performance
- Explore parameter combinations
- Discover optimal settings for scenarios

**Implementation Notes**:
- Use React for UI
- Animate message flow
- Update in real-time
- Provide preset scenarios
- Include "Why?" explanations
```

### Type 2: Code Playground
**Purpose**: Experiment with code in safe environment

**Specification**:
```markdown
## Interactive: ROS 2 Publisher Playground

**Concept**: Creating and configuring publishers

**Interface**:
```
┌─────────────────────────────────────────────┐
│ Code Editor                    │ Output     │
│                                │            │
│ import rclpy                   │ > Running  │
│ from std_msgs.msg import String│            │
│                                │ Published: │
│ # Your code here               │ "Hello"    │
│                                │            │
│                                │ Topic Info:│
│                                │ /my_topic  │
│                                │ String     │
│                                │ 10 Hz      │
│                                │            │
│ [Run Code] [Reset] [Hints]     │            │
└─────────────────────────────────────────────┘
```

**Features**:
- Syntax highlighting
- Auto-completion
- Error highlighting
- Real-time execution
- Output display
- Topic visualization

**Interactions**:
1. Edit code
   - Syntax checking
   - Error messages
   - Suggestions

2. Run code
   - Execute in sandbox
   - Show output
   - Display topic info
   - Visualize messages

3. Get hints
   - Progressive hints
   - Code snippets
   - Documentation links

**Challenges**:
- Level 1: Modify message content
- Level 2: Change publish rate
- Level 3: Add QoS configuration
- Level 4: Create custom message type

**Learning Goals**:
- Practice publisher creation
- Experiment with parameters
- See immediate results
- Build confidence through experimentation
```

### Type 3: Concept Visualizer
**Purpose**: Make abstract concepts concrete

**Specification**:
```markdown
## Interactive: ROS 2 Graph Visualizer

**Concept**: Understanding the ROS 2 computation graph

**Interface**:
- Canvas showing nodes and topics
- Nodes as circles (draggable)
- Topics as connecting lines
- Messages as animated dots
- Control panel for adding/removing elements

**Interactions**:
1. Add Node
   - Click "Add Node" button
   - Choose node type (publisher/subscriber/both)
   - Node appears on canvas
   - Can be dragged to position

2. Create Topic
   - Click on publisher node
   - Drag to subscriber node
   - Topic line appears
   - Name and configure topic

3. Send Messages
   - Click "Start" to begin message flow
   - See messages animate along topics
   - Adjust message rate
   - Pause/resume flow

4. Modify QoS
   - Click on topic line
   - Adjust QoS settings
   - See effect on message delivery
   - Visualize dropped messages

**Scenarios**:
- Scenario 1: Simple pub-sub
- Scenario 2: Multiple subscribers
- Scenario 3: Bidirectional communication
- Scenario 4: Complex multi-node system

**Learning Goals**:
- Visualize ROS 2 architecture
- Understand node relationships
- See message flow
- Experiment with topologies
```

### Type 4: Simulation Controller
**Purpose**: Control and observe simulated systems

**Specification**:
```markdown
## Interactive: Robot Motion Controller

**Concept**: Controlling robot movement with velocity commands

**Interface**:
```
┌─────────────────────────────────────────────┐
│ Simulation View          │ Controls         │
│                          │                  │
│  [Robot visualization]   │ Linear Velocity: │
│  [Environment]           │ [-1.0] ←→ [1.0]  │
│  [Obstacles]             │                  │
│                          │ Angular Velocity:│
│                          │ [-2.0] ←→ [2.0]  │
│                          │                  │
│                          │ [Start] [Stop]   │
│                          │ [Reset]          │
│                          │                  │
│ Status:                  │ Telemetry:       │
│ Position: (x, y, θ)      │ Speed: 0.5 m/s   │
│ Velocity: (vx, vy, ω)    │ Distance: 2.3 m  │
└─────────────────────────────────────────────┘
```

**Interactions**:
1. Adjust velocities
   - Drag sliders
   - Robot responds in real-time
   - See trajectory
   - Display telemetry

2. Preset motions
   - Forward
   - Turn in place
   - Circle
   - Figure-8

3. Obstacle avoidance
   - Place obstacles
   - Enable auto-avoidance
   - See planning in action
   - Adjust parameters

**Challenges**:
- Navigate to goal point
- Follow a path
- Avoid obstacles
- Park in tight space

**Learning Goals**:
- Understand velocity control
- See motion dynamics
- Practice trajectory planning
- Develop intuition for robot behavior
```

### Type 5: Interactive Quiz
**Purpose**: Assess understanding with immediate feedback

**Specification**:
```markdown
## Interactive: ROS 2 Concepts Quiz

**Format**: Progressive question flow with branching

**Question Types**:
1. Multiple choice with explanation
2. Code completion with validation
3. Drag-and-drop matching
4. Interactive diagram labeling

**Example Question**:
```
Question: Match each ROS 2 concept to its description

[Drag items from left to right]

Concepts:              Descriptions:
┌─────────┐           ┌──────────────────────────┐
│ Node    │           │ Named channel for data   │
│ Topic   │           │ Independent process      │
│ Message │           │ Data structure           │
│ QoS     │           │ Delivery policy          │
└─────────┘           └──────────────────────────┘

[Check Answer]

Feedback:
✅ Node → Independent process: Correct!
✅ Topic → Named channel for data: Correct!
❌ Message → Delivery policy: Not quite. Messages are data structures.
   Try again or see hint.
```

**Features**:
- Immediate feedback
- Explanatory responses
- Hint system
- Progress tracking
- Score display
- Review mode

**Learning Goals**:
- Reinforce concepts
- Identify gaps
- Build confidence
- Enable self-assessment
```

## Design Principles

### Principle 1: Immediate Feedback
Every interaction should provide instant response:

```markdown
## Feedback Timing

**Instant** (< 100ms):
- Button presses
- Slider adjustments
- Hover effects
- Selection highlights

**Quick** (< 500ms):
- Code validation
- Simple calculations
- State updates
- Visual changes

**Responsive** (< 2s):
- Code execution
- Simulation updates
- Complex calculations
- Data fetching
```

### Principle 2: Discoverability
Make interactions obvious and inviting:

```markdown
## Discoverability Techniques

**Visual Cues**:
- Buttons look clickable (shadows, borders)
- Sliders have clear handles
- Interactive areas highlighted on hover
- Cursor changes appropriately

**Affordances**:
- Draggable items have grab cursor
- Clickable elements have pointer cursor
- Disabled elements are grayed out
- Active elements are highlighted

**Instructions**:
- Brief inline instructions
- Tooltips on hover
- Example interactions shown
- "Try it" prompts
```

### Principle 3: Error Prevention
Design to prevent mistakes:

```markdown
## Error Prevention Strategies

**Input Validation**:
- Constrain values to valid ranges
- Provide sensible defaults
- Show valid input examples
- Disable invalid options

**Confirmation**:
- Confirm destructive actions
- Provide undo capability
- Save state automatically
- Warn before data loss

**Guidance**:
- Show valid next steps
- Highlight required fields
- Provide inline help
- Offer suggestions
```

### Principle 4: Progressive Disclosure
Reveal complexity gradually:

```markdown
## Complexity Levels

**Level 1: Basic**
- Simple interface
- Core functionality only
- Guided experience
- Limited options

**Level 2: Intermediate**
- Additional controls
- More parameters
- Less guidance
- More freedom

**Level 3: Advanced**
- Full control
- All parameters
- Expert mode
- Maximum flexibility

**Toggle**: "Show Advanced Options"
```

## Quality Standards

### Usability
- Intuitive interactions
- Clear feedback
- Responsive performance
- Error-tolerant
- Accessible

### Educational Value
- Supports learning goals
- Enables discovery
- Reinforces concepts
- Builds intuition
- Memorable experience

### Technical Quality
- Smooth animations
- Fast response times
- No bugs or glitches
- Works across devices
- Degrades gracefully

### Accessibility
- Keyboard navigable
- Screen reader compatible
- High contrast mode
- Adjustable text size
- Alternative interactions

## Integration Points
- Enhances technical content
- Supports concept simplification
- Enables hands-on learning
- Provides immediate feedback
- Creates engaging experience
