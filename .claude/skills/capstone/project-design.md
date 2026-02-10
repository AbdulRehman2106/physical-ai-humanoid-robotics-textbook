# Skill: Capstone Project Design

## Purpose
Design comprehensive capstone projects that integrate multiple learning outcomes into meaningful, achievable final projects.

## Responsibility
Create capstone project specifications that synthesize course content, provide clear objectives, and result in portfolio-worthy demonstrations of mastery.

## When to Use
- Designing final course projects
- Creating integrative assessments
- Building portfolio pieces
- Demonstrating comprehensive understanding
- Connecting theory to practice

## Core Capabilities

### 1. Project Scoping
- Define project objectives and outcomes
- Establish appropriate complexity level
- Set realistic time constraints
- Identify required prerequisites
- Balance challenge and achievability

### 2. Integration Design
- Connect multiple course concepts
- Require diverse skill application
- Create meaningful synthesis
- Demonstrate system thinking
- Show real-world relevance

### 3. Assessment Framework
- Define success criteria
- Create evaluation rubrics
- Establish milestones
- Design demonstration requirements
- Specify deliverables

### 4. Scaffolding Strategy
- Break project into phases
- Provide starter code/templates
- Create checkpoint assessments
- Offer progressive hints
- Enable incremental success

### 5. Narrative Framing
- Create compelling project story
- Establish meaningful context
- Define clear mission
- Build emotional engagement
- Celebrate achievement

## Capstone Project Template

```markdown
# Capstone Project: [Project Name]

## Mission
[Compelling 2-3 sentence description of what learners will build and why it matters]

## Learning Outcomes
By completing this project, you will demonstrate:
- [Outcome 1: Specific skill from Chapter X]
- [Outcome 2: Specific skill from Chapter Y]
- [Outcome 3: Integration of concepts A and B]
- [Outcome 4: System-level thinking]

## Project Overview

### What You'll Build
[Detailed description of the final system]

### Key Features
1. [Feature 1]: [What it does and why]
2. [Feature 2]: [What it does and why]
3. [Feature 3]: [What it does and why]

### System Architecture
[High-level diagram or description of components]

## Prerequisites
- Completed Chapters: [List]
- Required Knowledge: [Specific concepts]
- Technical Requirements: [Hardware/software]

## Project Phases

### Phase 1: Foundation (Week 1)
**Objective**: [What to achieve]

**Tasks**:
- [ ] [Task 1]
- [ ] [Task 2]
- [ ] [Task 3]

**Deliverable**: [What to submit/demonstrate]

**Success Criteria**:
- [Criterion 1]
- [Criterion 2]

### Phase 2: Core Functionality (Week 2)
[Repeat structure]

### Phase 3: Integration (Week 3)
[Repeat structure]

### Phase 4: Polish and Demo (Week 4)
[Repeat structure]

## Technical Specifications

### Required Components
- [Component 1]: [Purpose and requirements]
- [Component 2]: [Purpose and requirements]

### Communication Architecture
[How components interact]

### Data Flow
[How information moves through system]

## Starter Code
[Link to repository or code template]

## Milestones and Checkpoints

### Checkpoint 1: [Name] (End of Phase 1)
**Demonstrate**:
- [Capability 1]
- [Capability 2]

**Verification**:
```bash
[Commands to verify functionality]
```

### Checkpoint 2: [Name] (End of Phase 2)
[Repeat structure]

## Assessment Rubric

### Technical Implementation (40%)
- [ ] All required components implemented
- [ ] Code follows best practices
- [ ] System is robust and handles errors
- [ ] Performance meets requirements

### Integration (30%)
- [ ] Components communicate correctly
- [ ] System works as a cohesive whole
- [ ] Demonstrates understanding of architecture
- [ ] Shows system-level thinking

### Documentation (15%)
- [ ] Clear README with setup instructions
- [ ] Code is well-commented
- [ ] Architecture is documented
- [ ] Demo video is clear and complete

### Demonstration (15%)
- [ ] System performs required tasks
- [ ] Demo is well-prepared
- [ ] Explanation shows understanding
- [ ] Handles questions confidently

## Submission Requirements

### Code Repository
- Complete source code
- README with setup instructions
- Documentation of architecture
- Test cases and results

### Demo Video (3-5 minutes)
- System overview
- Live demonstration of features
- Explanation of key technical decisions
- Discussion of challenges and solutions

### Written Report (2-3 pages)
- Project overview
- Technical approach
- Challenges and solutions
- Lessons learned
- Future improvements

## Resources

### Helpful Documentation
- [Link to relevant docs]

### Example Projects
- [Link to example 1]
- [Link to example 2]

### Troubleshooting Guide
- [Common issue 1 and solution]
- [Common issue 2 and solution]

## Extensions (Optional)

### Advanced Features
- [Extension 1]: [Description and challenge]
- [Extension 2]: [Description and challenge]

### Research Directions
- [Research topic 1]
- [Research topic 2]

## Celebration
Upon completion, you will have:
- [Achievement 1]
- [Achievement 2]
- [Achievement 3]

This project demonstrates your mastery of [domain] and serves as a portfolio piece for [career goal].
```

## Project Types

### Type 1: Autonomous Navigation Robot
**Integration**: Perception + Planning + Control
**Concepts**: SLAM, path planning, obstacle avoidance, ROS 2
**Deliverable**: Robot navigates unknown environment to goal

### Type 2: Vision-Guided Manipulation
**Integration**: Computer vision + Kinematics + Control
**Concepts**: Object detection, inverse kinematics, grasp planning
**Deliverable**: Robot identifies and manipulates objects

### Type 3: Multi-Robot Coordination
**Integration**: Communication + Planning + Coordination
**Concepts**: Distributed systems, task allocation, ROS 2 topics
**Deliverable**: Multiple robots collaborate on shared task

### Type 4: Sim-to-Real Transfer
**Integration**: Simulation + Learning + Real-world deployment
**Concepts**: Digital twins, domain randomization, transfer learning
**Deliverable**: Policy trained in sim, deployed on real robot

### Type 5: Voice-Controlled Assistant Robot
**Integration**: NLP + Vision + Action
**Concepts**: Speech recognition, VLA models, task execution
**Deliverable**: Robot responds to natural language commands

## Scaffolding Strategies

### Strategy 1: Starter Template
Provide working skeleton code:
```python
# Starter code with TODOs
class AutonomousRobot:
    def __init__(self):
        # TODO: Initialize ROS 2 node
        pass

    def process_sensor_data(self, data):
        # TODO: Implement sensor processing
        pass

    def plan_path(self, goal):
        # TODO: Implement path planning
        pass

    def execute_motion(self, path):
        # TODO: Implement motion control
        pass
```

### Strategy 2: Progressive Hints
Reveal hints based on time or request:
- **Hint 1** (Day 1): "Start by getting sensor data working"
- **Hint 2** (Day 3): "Consider using A* for path planning"
- **Hint 3** (Day 5): "Check QoS settings if messages aren't received"

### Strategy 3: Reference Implementation
Provide simplified reference for one component:
```python
# Reference: Simple obstacle detection
def detect_obstacles(scan_data):
    """
    Example implementation of obstacle detection.
    Your implementation should be more robust.
    """
    min_distance = min(scan_data.ranges)
    return min_distance < SAFE_DISTANCE
```

### Strategy 4: Checkpoint Tests
Provide automated tests for each phase:
```python
# Phase 1 test
def test_sensor_reading():
    robot = AutonomousRobot()
    data = robot.get_sensor_data()
    assert data is not None
    assert len(data.ranges) > 0
```

## Quality Standards

### Project Scope
- Achievable in allocated time
- Requires synthesis of multiple concepts
- Has clear success criteria
- Provides room for creativity
- Results in portfolio-worthy work

### Educational Value
- Integrates course learning outcomes
- Requires deep understanding
- Builds practical skills
- Demonstrates system thinking
- Prepares for real-world work

### Assessment Clarity
- Rubric is specific and measurable
- Success criteria are unambiguous
- Milestones provide feedback
- Grading is fair and consistent
- Learners know what's expected

### Support Structure
- Adequate scaffolding provided
- Resources are accessible
- Help is available when needed
- Common issues are documented
- Success is achievable with effort

## Integration Points
- Synthesizes all learning outcomes
- Applies technical content
- Demonstrates mastery
- Creates portfolio piece
- Prepares for professional work
