# Skill: Visual Content Description

## Purpose
Create detailed specifications for diagrams, illustrations, and visual content that enhance understanding of technical concepts.

## Responsibility
Design visual content that clarifies complex ideas, shows relationships, and provides alternative representations of information.

## When to Use
- Specifying diagrams for chapters
- Designing system architecture visuals
- Creating process flow illustrations
- Designing data flow diagrams
- Specifying UI mockups

## Core Capabilities

### 1. Diagram Specification
- Define diagram purpose
- Specify visual elements
- Describe relationships
- Set visual hierarchy
- Define annotations

### 2. Visual Metaphor Selection
- Choose appropriate metaphors
- Map concepts to visuals
- Ensure clarity
- Avoid misleading representations
- Support mental models

### 3. Information Architecture
- Organize visual information
- Establish reading flow
- Create visual grouping
- Define emphasis
- Support scanning

### 4. Accessibility Design
- Provide text alternatives
- Ensure color contrast
- Use patterns with color
- Support screen readers
- Enable keyboard navigation

### 5. Style Consistency
- Define visual language
- Establish color palette
- Set typography rules
- Create icon system
- Maintain coherence

## Diagram Types

### Type 1: System Architecture Diagram
**Purpose**: Show component relationships and data flow

**Specification Template**:
```markdown
## Diagram: ROS 2 System Architecture

**Purpose**: Illustrate the layered architecture of ROS 2

**Dimensions**: 1200px × 800px

**Layout**: Horizontal layers, bottom to top

**Components**:

1. **Bottom Layer: DDS Middleware**
   - Rectangle: 1000px × 150px
   - Color: #E5E7EB (light gray)
   - Label: "DDS Middleware (Data Distribution Service)"
   - Font: 18px, semibold
   - Icon: Network symbol

2. **Middle Layer: ROS 2 Core**
   - Rectangle: 1000px × 250px
   - Color: #DBEAFE (light blue)
   - Contains 4 boxes:
     - "Nodes" (circle icon)
     - "Topics" (broadcast icon)
     - "Services" (request-response icon)
     - "Actions" (goal icon)
   - Each box: 200px × 180px
   - Spacing: 50px between boxes

3. **Top Layer: Application**
   - Rectangle: 1000px × 250px
   - Color: #D1FAE5 (light green)
   - Contains:
     - "Custom Nodes"
     - "Launch Files"
     - "Parameters"
   - Layout: 3 columns

**Connections**:
- Arrows from DDS to ROS 2 Core (upward)
- Arrows from ROS 2 Core to Application (upward)
- Bidirectional arrows between components in same layer

**Annotations**:
- "Foundation" label on DDS layer
- "Communication Primitives" on Core layer
- "Your Code" on Application layer

**Style**:
- Rounded corners: 8px
- Drop shadow: 0 2px 4px rgba(0,0,0,0.1)
- Border: 2px solid matching darker shade
- Font: Inter, sans-serif

**Accessibility**:
- Alt text: "ROS 2 architecture showing three layers: DDS middleware at bottom, ROS 2 core components in middle, and application layer on top"
- Color-blind safe palette
- Text contrast ratio: 4.5:1 minimum
```

### Type 2: Process Flow Diagram
**Purpose**: Show sequential steps or workflows

**Specification Template**:
```markdown
## Diagram: ROS 2 Node Lifecycle

**Purpose**: Illustrate state transitions in managed nodes

**Type**: State machine diagram

**Dimensions**: 800px × 600px

**States** (circles):

1. **Unconfigured**
   - Position: (100, 300)
   - Radius: 80px
   - Color: #9CA3AF (gray)
   - Label: "Unconfigured"
   - Icon: Question mark

2. **Inactive**
   - Position: (300, 300)
   - Radius: 80px
   - Color: #FCD34D (yellow)
   - Label: "Inactive"
   - Icon: Pause symbol

3. **Active**
   - Position: (500, 300)
   - Radius: 80px
   - Color: #34D399 (green)
   - Label: "Active"
   - Icon: Play symbol

4. **Finalized**
   - Position: (700, 300)
   - Radius: 80px
   - Color: #EF4444 (red)
   - Label: "Finalized"
   - Icon: Stop symbol

**Transitions** (arrows):

1. Configure: Unconfigured → Inactive
   - Curved arrow, 3px width
   - Color: #3B82F6 (blue)
   - Label: "configure()"
   - Position: Above arrow

2. Activate: Inactive → Active
   - Curved arrow, 3px width
   - Color: #3B82F6
   - Label: "activate()"

3. Deactivate: Active → Inactive
   - Curved arrow, 3px width
   - Color: #F59E0B (orange)
   - Label: "deactivate()"
   - Position: Below arrow (return path)

4. Cleanup: Inactive → Unconfigured
   - Curved arrow, 3px width
   - Color: #F59E0B
   - Label: "cleanup()"
   - Position: Below arrow

5. Shutdown: Any → Finalized
   - Straight arrows from all states
   - Dashed line, 2px width
   - Color: #EF4444 (red)
   - Label: "shutdown()"

**Annotations**:
- "Initial State" pointer to Unconfigured
- "Normal Operation" box around Active
- "Error Recovery" note near Deactivate

**Interactivity** (if digital):
- Hover over state: Show description
- Click transition: Show code example
- Animate: Show message flow

**Accessibility**:
- Alt text: "State machine showing ROS 2 node lifecycle with four states and transitions between them"
- Textual description provided below diagram
```

### Type 3: Data Flow Diagram
**Purpose**: Show how data moves through system

**Specification Template**:
```markdown
## Diagram: Sensor Data Processing Pipeline

**Purpose**: Illustrate data flow from sensors to actuators

**Type**: Data flow diagram

**Dimensions**: 1400px × 400px

**Layout**: Left to right flow

**Components**:

1. **Sensors** (left side)
   - Camera (icon + label)
   - Lidar (icon + label)
   - IMU (icon + label)
   - Vertical stack, 100px spacing
   - Icons: 64px × 64px
   - Color: #60A5FA (blue)

2. **Processing Nodes** (center)
   - "Sensor Fusion" (rounded rectangle)
   - "Object Detection" (rounded rectangle)
   - "Localization" (rounded rectangle)
   - Size: 200px × 80px
   - Color: #34D399 (green)
   - Vertical stack, 80px spacing

3. **Decision Node** (center-right)
   - "Path Planner" (hexagon)
   - Size: 220px × 100px
   - Color: #F59E0B (orange)

4. **Actuators** (right side)
   - "Motor Controller" (rounded rectangle)
   - Size: 200px × 80px
   - Color: #EF4444 (red)

**Data Flows** (arrows):

1. Camera → Object Detection
   - Arrow with data label: "Image (30 Hz)"
   - Width: 3px
   - Color: #3B82F6
   - Animated dots moving along arrow

2. Lidar → Sensor Fusion
   - Label: "LaserScan (10 Hz)"

3. IMU → Localization
   - Label: "IMU Data (100 Hz)"

4. All Processing → Path Planner
   - Converging arrows
   - Labels: "Obstacles", "Position", "Map"

5. Path Planner → Motor Controller
   - Thick arrow (5px)
   - Label: "Velocity Commands"
   - Color: #EF4444 (red, critical path)

**Annotations**:
- "Perception" label over processing nodes
- "Control" label over decision and actuators
- Data rates shown on each arrow
- "Critical Path" highlight on final arrow

**Legend**:
- Blue: Sensor data
- Green: Processed information
- Orange: Decision making
- Red: Control commands

**Accessibility**:
- Alt text: "Data flow from sensors through processing nodes to motor controller"
- Detailed textual description of flow
```

### Type 4: Concept Illustration
**Purpose**: Visualize abstract concepts

**Specification Template**:
```markdown
## Illustration: Publish-Subscribe Pattern

**Purpose**: Make pub-sub pattern concrete and memorable

**Type**: Conceptual illustration with metaphor

**Dimensions**: 1000px × 600px

**Visual Metaphor**: Radio broadcasting

**Elements**:

1. **Publisher** (left)
   - Radio tower illustration
   - Height: 300px
   - Broadcasting waves (concentric circles)
   - Label: "Publisher Node"
   - Sublabel: "Broadcasts messages"
   - Color: #3B82F6 (blue)

2. **Topic** (center)
   - Frequency display: "FM 102.5"
   - Large, prominent
   - Label: "Topic: /robot_status"
   - Sublabel: "Named channel"
   - Color: #8B5CF6 (purple)
   - Waves emanating from tower pass through this

3. **Subscribers** (right)
   - Three radio receivers
   - Different styles (car radio, portable, home stereo)
   - Each labeled: "Subscriber Node 1", "2", "3"
   - All tuned to same frequency
   - Color: #10B981 (green)
   - Receiving waves shown

**Visual Flow**:
- Waves flow left to right
- Animated (if digital): Pulses from tower to radios
- Dotted lines show connection to topic

**Annotations**:
- "One-to-Many" label with arrow pointing to multiple subscribers
- "Decoupled" note: "Radios don't need to know about the tower"
- "Same Message" note: "All subscribers receive identical data"

**Code Connection**:
Small code snippets near each element:
```python
# Near publisher
publisher = node.create_publisher(String, '/robot_status', 10)

# Near topic
# Topic: /robot_status
# Type: String

# Near subscribers
subscription = node.create_subscription(String, '/robot_status', callback, 10)
```

**Style**:
- Friendly, approachable illustration style
- Not too technical/engineering-looking
- Warm colors
- Soft shadows

**Accessibility**:
- Alt text: "Illustration comparing ROS 2 publish-subscribe to radio broadcasting, showing one publisher sending to multiple subscribers"
- Detailed description of metaphor
```

### Type 5: Comparison Diagram
**Purpose**: Show differences between options

**Specification Template**:
```markdown
## Diagram: Topics vs Services vs Actions

**Purpose**: Compare ROS 2 communication patterns

**Type**: Side-by-side comparison

**Dimensions**: 1400px × 800px

**Layout**: Three columns

**Column 1: Topics**
- Header: "Topics" (large, bold)
- Icon: Broadcast symbol
- Diagram: Publisher → Topic → Subscribers
- Characteristics (bullet points):
  - "Continuous streaming"
  - "One-to-many"
  - "No response"
  - "Asynchronous"
- Use case box: "Sensor data, state updates"
- Example: Camera images
- Color theme: Blue

**Column 2: Services**
- Header: "Services"
- Icon: Request-response symbol
- Diagram: Client ⇄ Server (bidirectional)
- Characteristics:
  - "Request-response"
  - "One-to-one"
  - "Synchronous"
  - "Blocking"
- Use case box: "Configuration, queries"
- Example: "Get robot pose"
- Color theme: Green

**Column 3: Actions**
- Header: "Actions"
- Icon: Goal symbol
- Diagram: Client → Server (with feedback loop)
- Characteristics:
  - "Long-running tasks"
  - "Feedback provided"
  - "Cancellable"
  - "Asynchronous"
- Use case box: "Navigation, manipulation"
- Example: "Move to goal"
- Color theme: Orange

**Visual Comparison**:
- Timing diagrams below each showing message flow
- Arrows showing direction of communication
- Dotted lines for feedback/response

**Decision Tree** (bottom):
"Which to use?"
- Need continuous data? → Topics
- Need confirmation? → Services
- Long task with feedback? → Actions

**Accessibility**:
- Alt text: "Comparison of three ROS 2 communication patterns showing their characteristics and use cases"
- Table format alternative provided
```

## Quality Standards

### Clarity
- Purpose is clear
- Elements are distinct
- Labels are readable
- Flow is obvious
- Complexity is appropriate

### Accuracy
- Technically correct
- Relationships accurate
- Proportions meaningful
- Details precise
- No misleading elements

### Aesthetics
- Visually appealing
- Professional appearance
- Consistent style
- Appropriate colors
- Good composition

### Accessibility
- Alt text provided
- Color-blind safe
- High contrast
- Text alternatives
- Screen reader compatible

## Integration Points
- Supports technical content
- Enhances concept simplification
- Provides visual learning modality
- Complements text explanations
- Creates memorable representations
