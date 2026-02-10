# Data Model: Physical AI & Humanoid Robotics Digital Book

**Feature**: Physical AI & Humanoid Robotics Digital Book
**Branch**: 001-physical-ai-book
**Date**: 2026-02-09

## Overview

This document defines the content structure and key entities for the Physical AI & Humanoid Robotics Digital Book. The data model represents the educational content architecture, not a traditional database schema.

## Primary Entities

### 1. Chapter

Represents a complete learning module covering a specific topic.

**Attributes**:
- `number`: Integer (1-15) - Chapter sequence number
- `title`: String - Descriptive chapter title
- `slug`: String - URL-friendly identifier (e.g., "ros2-fundamentals")
- `learning_outcomes`: Array[LearningOutcome] - 3-5 measurable objectives
- `prerequisites`: Array[String] - Required prior knowledge (references to other chapters)
- `estimated_time`: Integer - Minutes for core content (180-240 minutes)
- `difficulty_level`: Enum[beginner, intermediate, advanced]
- `week_number`: Integer (1-12) - Position in 12-week course
- `word_count`: Integer (1000-3000) - Target word count
- `status`: Enum[draft, review, published]

**Sections** (ordered):
1. `introduction`: Overview and motivation
2. `theory`: Formal concepts and principles
3. `intuition`: Analogies and mental models
4. `application`: Practical implementation
5. `exercises`: Hands-on practice
6. `assessment`: Quiz or checkpoint
7. `summary`: Key takeaways
8. `references`: APA-formatted citations

**Relationships**:
- `has_many` sections
- `has_many` code_examples (3-5 minimum)
- `has_many` visual_content (2-3 diagrams minimum)
- `has_many` assessments (1 minimum)
- `has_many` citations
- `belongs_to` module (P1, P2, or P3 priority)

**Validation Rules**:
- Word count between 1,000-3,000
- Flesch-Kincaid grade level 10-12
- Minimum 2 diagrams
- Minimum 3 code examples
- All prerequisites must reference existing chapters
- Learning outcomes must be measurable

**Example**:
```yaml
chapter:
  number: 3
  title: "ROS 2 Communication Patterns"
  slug: "ros2-communication-patterns"
  learning_outcomes:
    - "Implement publish-subscribe communication using topics"
    - "Create request-response interactions using services"
    - "Design goal-based tasks using actions"
  prerequisites:
    - "Chapter 2: ROS 2 Fundamentals"
  estimated_time: 210
  difficulty_level: intermediate
  week_number: 3
  word_count: 2400
  status: draft
```

---

### 2. Learning Outcome

Measurable objective that students should achieve after completing a chapter.

**Attributes**:
- `id`: String - Unique identifier (e.g., "LO-3.1")
- `description`: String - Clear, actionable statement
- `bloom_level`: Enum[remember, understand, apply, analyze, evaluate, create]
- `assessment_method`: Enum[quiz, exercise, project, demonstration]
- `success_criteria`: String - How achievement is measured

**Relationships**:
- `belongs_to` chapter
- `assessed_by` assessments (many-to-many)

**Validation Rules**:
- Must use action verb from Bloom's taxonomy
- Must be specific and measurable
- Must align with chapter content

**Example**:
```yaml
learning_outcome:
  id: "LO-3.1"
  description: "Implement publish-subscribe communication using ROS 2 topics"
  bloom_level: apply
  assessment_method: exercise
  success_criteria: "Student creates a working publisher-subscriber pair that exchanges messages"
```

---

### 3. Code Example

Complete, runnable code snippet demonstrating a concept or technique.

**Attributes**:
- `id`: String - Unique identifier (e.g., "EX-3.2")
- `title`: String - Descriptive title
- `language`: Enum[python, cpp, bash, yaml]
- `code`: String - Complete source code
- `explanation`: String - What the code does and why
- `expected_output`: String - What students should see
- `dependencies`: Array[String] - Required packages/libraries
- `difficulty`: Enum[beginner, intermediate, advanced]
- `file_path`: String - Location in downloadable code repository

**Relationships**:
- `belongs_to` chapter
- `demonstrates` learning_outcomes (many-to-many)

**Validation Rules**:
- Code must be complete (no fragments)
- Must include all necessary imports
- Must be tested and runnable
- Must include explanatory comments
- Expected output must be provided

**Example**:
```yaml
code_example:
  id: "EX-3.2"
  title: "Simple ROS 2 Publisher"
  language: python
  code: |
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimplePublisher(Node):
        def __init__(self):
            super().__init__('simple_publisher')
            self.publisher = self.create_publisher(String, 'topic', 10)
            self.timer = self.create_timer(1.0, self.timer_callback)
            self.count = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello {self.count}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            self.count += 1

    def main():
        rclpy.init()
        node = SimplePublisher()
        rclpy.spin(node)
        rclpy.shutdown()
  explanation: "Creates a ROS 2 node that publishes string messages to a topic every second"
  expected_output: "Published: Hello 0\nPublished: Hello 1\n..."
  dependencies: ["rclpy", "std_msgs"]
  difficulty: beginner
  file_path: "code-examples/ros2/simple_publisher.py"
```

---

### 4. Visual Content

Diagram, screenshot, animation, or interactive element that illustrates a concept.

**Attributes**:
- `id`: String - Unique identifier (e.g., "VIS-3.1")
- `type`: Enum[diagram, screenshot, animation, interactive]
- `title`: String - Descriptive title
- `alt_text`: String - Accessibility description (required)
- `caption`: String - Brief explanation
- `source`: String - File path or URL
- `format`: Enum[svg, png, webp, mp4, component]
- `width`: Integer - Display width in pixels
- `height`: Integer - Display height in pixels

**Relationships**:
- `belongs_to` chapter
- `illustrates` concepts (many-to-many)

**Validation Rules**:
- Alt text required (WCAG 2.1 AA)
- Color contrast ratio 4.5:1 minimum
- SVG preferred for diagrams
- WebP preferred for photos
- Animations must be 60fps
- Must respect `prefers-reduced-motion`

**Example**:
```yaml
visual_content:
  id: "VIS-3.1"
  type: diagram
  title: "ROS 2 Communication Patterns Comparison"
  alt_text: "Diagram comparing three ROS 2 communication patterns: topics (one-to-many broadcast), services (one-to-one request-response), and actions (goal-based with feedback)"
  caption: "Topics enable continuous data streaming, services provide synchronous request-response, and actions support long-running tasks with feedback"
  source: "/img/diagrams/ros2-communication-patterns.svg"
  format: svg
  width: 800
  height: 600
```

---

### 5. Assessment

Quiz, checkpoint, or exercise to verify understanding.

**Attributes**:
- `id`: String - Unique identifier (e.g., "ASSESS-3.1")
- `type`: Enum[quiz, checkpoint, exercise, project]
- `title`: String - Assessment name
- `questions`: Array[Question] - Assessment items
- `passing_score`: Integer - Percentage required to pass (typically 80%)
- `time_limit`: Integer - Minutes (optional, null for untimed)
- `attempts_allowed`: Integer - Number of retries (null for unlimited)

**Relationships**:
- `belongs_to` chapter
- `assesses` learning_outcomes (many-to-many)

**Validation Rules**:
- Must assess at least one learning outcome
- Questions must have correct answers defined
- Feedback must be provided for all answers
- Passing score must be reasonable (70-90%)

**Example**:
```yaml
assessment:
  id: "ASSESS-3.1"
  type: quiz
  title: "ROS 2 Communication Patterns Quiz"
  questions:
    - question: "Which communication pattern should you use for continuous sensor data?"
      type: multiple_choice
      options:
        - "Topics"
        - "Services"
        - "Actions"
        - "Parameters"
      correct_answer: 0
      explanation: "Topics use publish-subscribe pattern ideal for continuous data streams like sensor readings"
    - question: "What is the key difference between services and actions?"
      type: multiple_choice
      options:
        - "Services are faster"
        - "Actions provide feedback during execution"
        - "Services can be cancelled"
        - "Actions are synchronous"
      correct_answer: 1
      explanation: "Actions support long-running tasks with periodic feedback, while services are simple request-response"
  passing_score: 80
  time_limit: null
  attempts_allowed: null
```

---

### 6. Simulation Exercise

Hands-on activity using Gazebo, Unity, or Isaac Sim.

**Attributes**:
- `id`: String - Unique identifier (e.g., "SIM-6.1")
- `title`: String - Exercise name
- `platform`: Enum[gazebo, isaac_sim, unity]
- `setup_instructions`: String - How to prepare environment
- `objectives`: Array[String] - What students should accomplish
- `starter_code`: String - Initial code provided (optional)
- `solution`: String - Complete solution (hidden from students initially)
- `estimated_time`: Integer - Minutes to complete
- `difficulty`: Enum[beginner, intermediate, advanced]

**Relationships**:
- `belongs_to` chapter
- `requires` code_examples (many-to-many)
- `demonstrates` learning_outcomes (many-to-many)

**Validation Rules**:
- Setup instructions must be complete and tested
- Objectives must be clear and achievable
- Solution must be verified to work
- Platform version must be specified

**Example**:
```yaml
simulation_exercise:
  id: "SIM-6.1"
  title: "Create a Mobile Robot in Gazebo"
  platform: gazebo
  setup_instructions: |
    1. Launch Gazebo: `gazebo --verbose`
    2. Create a new world file
    3. Prepare URDF model directory
  objectives:
    - "Create a URDF model for a differential drive robot"
    - "Add a lidar sensor to the robot"
    - "Configure physics parameters"
    - "Launch the robot in Gazebo and verify sensor data"
  starter_code: "# URDF template provided in code-examples/gazebo/robot_template.urdf"
  solution: "# Complete solution in code-examples/gazebo/mobile_robot_solution.urdf"
  estimated_time: 45
  difficulty: intermediate
```

---

### 7. Citation

APA-formatted reference to authoritative source.

**Attributes**:
- `id`: String - Unique identifier (e.g., "CIT-3.1")
- `type`: Enum[paper, documentation, book, website]
- `authors`: Array[String] - Author names
- `year`: Integer - Publication year
- `title`: String - Work title
- `source`: String - Journal, publisher, or website
- `url`: String - Direct link (if available)
- `doi`: String - Digital Object Identifier (if applicable)
- `accessed_date`: Date - When URL was accessed (for websites)

**Relationships**:
- `referenced_by` chapters (many-to-many)

**Validation Rules**:
- Must follow APA 7th edition format
- URL must be valid and accessible
- All required fields for citation type must be present

**Example**:
```yaml
citation:
  id: "CIT-3.1"
  type: documentation
  authors: ["Open Robotics"]
  year: 2023
  title: "ROS 2 Documentation: Humble"
  source: "ROS 2 Official Documentation"
  url: "https://docs.ros.org/en/humble/"
  doi: null
  accessed_date: "2026-02-09"

# Formatted output:
# Open Robotics. (2023). ROS 2 Documentation: Humble. Retrieved February 9, 2026, from https://docs.ros.org/en/humble/
```

---

### 8. Capstone Project

Integrative final project synthesizing multiple chapters.

**Attributes**:
- `id`: String - "CAPSTONE-1"
- `title`: String - Project name
- `description`: String - Project overview
- `phases`: Array[Phase] - Project stages
- `deliverables`: Array[String] - Required outputs
- `rubric`: Rubric - Evaluation criteria
- `estimated_time`: Integer - Hours to complete
- `team_size`: Integer - Students per team (1 for individual)

**Phases**:
Each phase has:
- `number`: Integer
- `title`: String
- `objectives`: Array[String]
- `deliverables`: Array[String]
- `duration`: Integer (hours)

**Relationships**:
- `integrates` chapters (many-to-many)
- `requires` learning_outcomes (many-to-many)

**Validation Rules**:
- Must integrate at least 3 chapters
- Must assess P1 and P2 learning outcomes
- Rubric must be clear and measurable
- Phases must be sequential and logical

**Example**:
```yaml
capstone_project:
  id: "CAPSTONE-1"
  title: "Autonomous Navigation Robot"
  description: "Build a complete autonomous robot system that navigates to goals, avoids obstacles, and responds to natural language commands"
  phases:
    - number: 1
      title: "Foundation Setup"
      objectives:
        - "Set up ROS 2 environment"
        - "Create basic robot model in Gazebo"
        - "Implement sensor processing"
      deliverables:
        - "Working Gazebo simulation"
        - "Sensor data visualization"
      duration: 8
    - number: 2
      title: "Navigation Implementation"
      objectives:
        - "Implement path planning"
        - "Add obstacle avoidance"
        - "Test navigation in simulation"
      deliverables:
        - "Navigation stack implementation"
        - "Test results documentation"
      duration: 10
    - number: 3
      title: "VLA Integration"
      objectives:
        - "Integrate vision-language model"
        - "Implement language-conditioned control"
        - "Test complete system"
      deliverables:
        - "VLA pipeline implementation"
        - "Demo video"
      duration: 12
    - number: 4
      title: "Final Presentation"
      objectives:
        - "Prepare demonstration"
        - "Document design decisions"
        - "Present to peers"
      deliverables:
        - "Final report"
        - "Presentation slides"
        - "Live demonstration"
      duration: 6
  deliverables:
    - "Complete source code repository"
    - "Technical documentation"
    - "Demo video (3-5 minutes)"
    - "Written report (5-10 pages)"
    - "Live demonstration"
  rubric:
    technical_implementation: 40
    integration: 30
    documentation: 15
    demonstration: 15
  estimated_time: 36
  team_size: 1
```

---

## Content Hierarchy

```
Book
├── Module (Priority Level: P1, P2, P3)
│   ├── Chapter 1
│   │   ├── Learning Outcomes (3-5)
│   │   ├── Sections (7 standard sections)
│   │   ├── Code Examples (3-5 minimum)
│   │   ├── Visual Content (2-3 minimum)
│   │   ├── Simulation Exercises (0-2)
│   │   ├── Assessments (1 minimum)
│   │   └── Citations (as needed)
│   ├── Chapter 2
│   └── ...
└── Capstone Project (integrates all modules)
```

---

## Validation Rules Summary

### Chapter-Level Validation
- [ ] Word count: 1,000-3,000
- [ ] Flesch-Kincaid: 10-12
- [ ] Learning outcomes: 3-5 defined
- [ ] Code examples: 3-5 minimum
- [ ] Visual content: 2-3 diagrams minimum
- [ ] Assessments: 1 minimum
- [ ] Prerequisites: Valid references
- [ ] Citations: APA format

### Code Example Validation
- [ ] Complete and runnable
- [ ] All imports included
- [ ] Tested successfully
- [ ] Explanatory comments
- [ ] Expected output provided
- [ ] Dependencies listed

### Visual Content Validation
- [ ] Alt text provided
- [ ] Color contrast: 4.5:1 minimum
- [ ] Appropriate format (SVG/WebP)
- [ ] Animations: 60fps
- [ ] Respects prefers-reduced-motion

### Assessment Validation
- [ ] Aligns with learning outcomes
- [ ] Correct answers defined
- [ ] Feedback provided
- [ ] Passing score reasonable (70-90%)

### Accessibility Validation
- [ ] WCAG 2.1 AA compliant
- [ ] Keyboard navigable
- [ ] Screen reader compatible
- [ ] Semantic HTML structure

---

## Content Creation Workflow

1. **Chapter Planning** → Use `spec/chapter-specification` skill
2. **Learning Outcomes** → Use `spec/learning-outcomes` skill
3. **Content Writing** → Use `content/technical-chapter-writing` skill
4. **Code Examples** → Use `content/code-example-design` skill
5. **Visual Content** → Use `content/visual-content-description` skill
6. **Assessments** → Use `content/exercise-assessment-design` skill
7. **Quality Review** → Validate against all rules
8. **Publication** → Deploy to Docusaurus

---

## Summary

This data model defines the structure for a comprehensive educational book on Physical AI and Humanoid Robotics. All entities support the skills-based architecture and ensure consistency, quality, and accessibility across the entire book. The model enforces constitution principles through validation rules and supports the 12-week course structure with clear learning progression.
