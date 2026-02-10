# Skill: MDX Component Design

## Purpose
Design reusable MDX components that enhance educational content with interactive elements, rich media, and custom layouts.

## Responsibility
Create specifications for custom MDX components that extend Docusaurus capabilities while maintaining consistency and usability.

## When to Use
- Creating interactive learning elements
- Designing custom content blocks
- Building reusable educational patterns
- Enhancing standard markdown
- Adding rich media experiences

## Core Capabilities

### 1. Component Architecture
- Define component props and API
- Establish component composition patterns
- Create reusable component library
- Design component variants
- Ensure accessibility compliance

### 2. Interactive Elements
- Design quiz and assessment components
- Create code playgrounds
- Build interactive diagrams
- Design reveal/toggle patterns
- Implement progress tracking

### 3. Content Enhancement
- Design callout and admonition variants
- Create tabbed content interfaces
- Build comparison layouts
- Design timeline components
- Implement accordion patterns

### 4. Media Integration
- Design video player components
- Create image gallery patterns
- Build diagram viewers
- Design code example showcases
- Implement 3D model viewers

### 5. Learning Support
- Design checkpoint components
- Create exercise templates
- Build solution reveal patterns
- Design hint systems
- Implement feedback mechanisms

## Core Components

### 1. Enhanced Callout
**Purpose**: Highlight important information with visual distinction

**Variants**:
- `info`: General information (blue)
- `tip`: Helpful suggestions (green)
- `warning`: Cautions (yellow)
- `danger`: Critical warnings (red)
- `insight`: Key insights (purple)

**Props**:
```typescript
interface CalloutProps {
  type: 'info' | 'tip' | 'warning' | 'danger' | 'insight';
  title?: string;
  icon?: string;
  collapsible?: boolean;
  children: ReactNode;
}
```

**Usage**:
```mdx
<Callout type="insight" title="Key Insight">
ROS 2 nodes are independent processes that communicate through topics.
This decoupling enables distributed robotics systems.
</Callout>
```

**Visual Specification**:
```
┌─────────────────────────────────────┐
│ 💡 Key Insight                      │
├─────────────────────────────────────┤
│                                     │
│ ROS 2 nodes are independent         │
│ processes that communicate through  │
│ topics. This decoupling enables     │
│ distributed robotics systems.       │
│                                     │
└─────────────────────────────────────┘
```

### 2. Code Playground
**Purpose**: Interactive code editor with live execution

**Props**:
```typescript
interface CodePlaygroundProps {
  language: 'python' | 'javascript' | 'cpp';
  initialCode: string;
  readOnly?: boolean;
  showOutput?: boolean;
  testCases?: TestCase[];
}
```

**Usage**:
```mdx
<CodePlayground
  language="python"
  initialCode={`
import rclpy

def main():
    rclpy.init()
    print("Hello, ROS 2!")
    rclpy.shutdown()
  `}
  showOutput={true}
/>
```

### 3. Concept Tabs
**Purpose**: Present multiple perspectives or approaches

**Props**:
```typescript
interface ConceptTabsProps {
  defaultTab?: string;
  children: TabPanel[];
}
```

**Usage**:
```mdx
<ConceptTabs>
  <TabPanel label="Concept" icon="📖">
    ROS 2 uses a publish-subscribe pattern...
  </TabPanel>
  <TabPanel label="Code" icon="💻">
    ```python
    publisher = node.create_publisher(...)
    ```
  </TabPanel>
  <TabPanel label="Diagram" icon="📊">
    <Diagram src="pubsub-pattern.svg" />
  </TabPanel>
</ConceptTabs>
```

### 4. Interactive Quiz
**Purpose**: Check understanding with immediate feedback

**Props**:
```typescript
interface QuizProps {
  question: string;
  options: QuizOption[];
  correctAnswer: number;
  explanation: string;
  hint?: string;
}
```

**Usage**:
```mdx
<Quiz
  question="What is the primary communication pattern in ROS 2?"
  options={[
    "Client-Server",
    "Publish-Subscribe",
    "Peer-to-Peer",
    "Request-Response"
  ]}
  correctAnswer={1}
  explanation="ROS 2 primarily uses publish-subscribe for continuous data streams."
  hint="Think about how sensor data flows through the system."
/>
```

### 5. Step-by-Step Guide
**Purpose**: Break complex procedures into manageable steps

**Props**:
```typescript
interface StepGuideProps {
  title: string;
  steps: Step[];
  showProgress?: boolean;
}
```

**Usage**:
```mdx
<StepGuide title="Setting Up Your First ROS 2 Node">
  <Step number={1} title="Create Package">
    ```bash
    ros2 pkg create my_package
    ```
  </Step>
  <Step number={2} title="Write Node Code">
    Create a Python file with your node implementation...
  </Step>
  <Step number={3} title="Build and Run">
    ```bash
    colcon build
    ros2 run my_package my_node
    ```
  </Step>
</StepGuide>
```

### 6. Comparison Table
**Purpose**: Compare approaches, tools, or concepts

**Props**:
```typescript
interface ComparisonProps {
  items: ComparisonItem[];
  criteria: string[];
}
```

**Usage**:
```mdx
<Comparison
  items={[
    { name: "Topics", ...features },
    { name: "Services", ...features },
    { name: "Actions", ...features }
  ]}
  criteria={["Use Case", "Pattern", "Feedback", "Cancellable"]}
/>
```

### 7. Exercise Block
**Purpose**: Hands-on practice with solution reveal

**Props**:
```typescript
interface ExerciseProps {
  title: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  objective: string;
  starter?: string;
  solution: string;
  hints?: string[];
}
```

**Usage**:
```mdx
<Exercise
  title="Create a Simple Publisher"
  difficulty="beginner"
  objective="Create a ROS 2 node that publishes string messages"
  starter={`import rclpy\n# Your code here`}
  solution={`...complete solution...`}
  hints={[
    "Start by initializing rclpy",
    "Create a node and publisher",
    "Use a timer to publish periodically"
  ]}
/>
```

### 8. Checkpoint
**Purpose**: Verify understanding before proceeding

**Props**:
```typescript
interface CheckpointProps {
  title: string;
  questions: CheckQuestion[];
  requiredScore?: number;
}
```

**Usage**:
```mdx
<Checkpoint title="ROS 2 Basics Check">
  <CheckQuestion>
    Can you explain what a ROS 2 node is?
  </CheckQuestion>
  <CheckQuestion>
    Can you create a simple publisher?
  </CheckQuestion>
  <CheckQuestion>
    Can you describe the publish-subscribe pattern?
  </CheckQuestion>
</Checkpoint>
```

### 9. Animated Diagram
**Purpose**: Interactive, animated technical diagrams

**Props**:
```typescript
interface AnimatedDiagramProps {
  src: string;
  controls?: boolean;
  autoplay?: boolean;
  caption?: string;
}
```

**Usage**:
```mdx
<AnimatedDiagram
  src="/diagrams/ros2-architecture.svg"
  controls={true}
  autoplay={false}
  caption="ROS 2 System Architecture with Data Flow"
/>
```

### 10. Learning Path Card
**Purpose**: Show progress and next steps

**Props**:
```typescript
interface LearningPathProps {
  current: string;
  completed: string[];
  next: string[];
  capstone?: string;
}
```

**Usage**:
```mdx
<LearningPath
  current="ROS 2 Nodes"
  completed={["Introduction", "Setup"]}
  next={["Topics", "Services", "Actions"]}
  capstone="Build Autonomous Robot"
/>
```

## Component Design Principles

### 1. Consistency
- Follow Docusaurus design system
- Use consistent prop naming
- Maintain visual coherence
- Apply standard spacing
- Use theme tokens

### 2. Accessibility
- Semantic HTML structure
- ARIA labels where needed
- Keyboard navigation support
- Screen reader friendly
- Focus management

### 3. Responsiveness
- Mobile-first design
- Touch-friendly interactions
- Adaptive layouts
- Readable on all screens
- Performance optimized

### 4. Composability
- Components work together
- Nestable where appropriate
- Clear composition patterns
- Predictable behavior
- Minimal dependencies

### 5. Pedagogy
- Supports learning objectives
- Reduces cognitive load
- Provides immediate feedback
- Encourages active learning
- Tracks progress

## Quality Standards

### Code Quality
- TypeScript for type safety
- Proper prop validation
- Error handling
- Performance optimization
- Clean, documented code

### User Experience
- Intuitive interactions
- Clear visual feedback
- Helpful error messages
- Smooth animations
- Consistent behavior

### Accessibility
- WCAG 2.1 AA compliance
- Keyboard accessible
- Screen reader tested
- Color contrast verified
- Focus indicators visible

### Performance
- Fast initial render
- Lazy loading where appropriate
- Minimal bundle size
- Optimized re-renders
- Smooth interactions

## Integration Points
- Extends Docusaurus MDX capabilities
- Implements design system
- Supports learning outcomes
- Enables interactive content
- Tracks learner progress
