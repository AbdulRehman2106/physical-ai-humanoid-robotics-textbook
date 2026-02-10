# Assessment Template

**Purpose**: Standardized structure for quizzes, checkpoints, and exercises
**Version**: 1.0
**Last Updated**: 2026-02-09

## Assessment Types

### 1. Quiz (Knowledge Check)
**Purpose**: Verify understanding of concepts
**Format**: Multiple choice, true/false, short answer
**Timing**: End of chapter
**Grading**: Automated with immediate feedback

### 2. Checkpoint (Self-Assessment)
**Purpose**: Encourage reflection and self-evaluation
**Format**: Open-ended questions, "Can you explain..." prompts
**Timing**: Throughout chapter (every 2-3 sections)
**Grading**: Self-assessed, not graded

### 3. Exercise (Hands-On Practice)
**Purpose**: Apply concepts through coding or problem-solving
**Format**: Coding tasks, design problems, analysis tasks
**Timing**: End of chapter or section
**Grading**: Rubric-based or automated testing

### 4. Project (Integrative Assessment)
**Purpose**: Synthesize multiple concepts
**Format**: Multi-phase project with deliverables
**Timing**: Capstone or end of module
**Grading**: Comprehensive rubric

---

## Quiz Template

```yaml
assessment:
  id: "ASSESS-[X].[Y]"
  type: quiz
  title: "[Chapter Title] Quiz"
  chapter: [X]
  learning_outcomes:
    - "LO-[X].1"
    - "LO-[X].2"
  passing_score: 80
  time_limit: null  # minutes, null for untimed
  attempts_allowed: null  # null for unlimited
  questions: [5-10]
```

### Question Format

**Multiple Choice**:
```markdown
### Question [N]

[Question text - clear, unambiguous, tests understanding not memorization]

**Options**:
- A) [Option A - plausible distractor]
- B) [Option B - correct answer]
- C) [Option C - plausible distractor]
- D) [Option D - plausible distractor]

<details>
<summary>Show Answer</summary>

**Correct Answer**: B

**Explanation**: [Why B is correct. Why A, C, D are incorrect. Connect to learning outcome.]

**Related Concept**: [Link to relevant section in chapter]
</details>
```

**True/False**:
```markdown
### Question [N]

**Statement**: [Clear statement that is definitively true or false]

- [ ] True
- [ ] False

<details>
<summary>Show Answer</summary>

**Correct Answer**: [True/False]

**Explanation**: [Why this is true/false. Common misconception if applicable.]
</details>
```

**Code Completion**:
```markdown
### Question [N]

Complete the following code to [objective]:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # TODO: Complete this line
        self.publisher = _______________
```

<details>
<summary>Show Answer</summary>

**Correct Answer**:
```python
self.publisher = self.create_publisher(String, 'topic', 10)
```

**Explanation**: [Why this is correct. What each parameter means.]
</details>
```

### Quiz Example

```markdown
## Chapter 3 Quiz: ROS 2 Communication Patterns

**Instructions**: Answer the following questions to check your understanding. You can retry as many times as needed.

**Passing Score**: 80% (4 out of 5 correct)

---

### Question 1

Which ROS 2 communication pattern should you use for continuous sensor data streaming?

**Options**:
- A) Services
- B) Actions
- C) Topics
- D) Parameters

<details>
<summary>Show Answer</summary>

**Correct Answer**: C

**Explanation**: Topics use the publish-subscribe pattern, which is ideal for continuous data streams like sensor readings. Services are for request-response (not continuous), actions are for long-running tasks with feedback, and parameters are for configuration.

**Related Concept**: See "Topics: Publish-Subscribe Pattern" in Section 3.2
</details>

---

### Question 2

True or False: In ROS 2, a publisher must know which subscribers are listening to its topic.

- [ ] True
- [ ] False

<details>
<summary>Show Answer</summary>

**Correct Answer**: False

**Explanation**: This is the key benefit of the publish-subscribe pattern - publishers and subscribers are decoupled. Publishers broadcast messages without knowing who (if anyone) is listening. This enables flexible system architecture.

**Related Concept**: See "Decoupled Communication" in Section 3.1
</details>

---

[Continue with 3-8 more questions]
```

---

## Checkpoint Template

```yaml
checkpoint:
  id: "CHECK-[X].[Y]"
  type: checkpoint
  title: "[Section Title] Checkpoint"
  chapter: [X]
  section: [Y]
  learning_outcomes:
    - "LO-[X].[Y]"
```

### Checkpoint Format

```markdown
## Checkpoint: [Section Title]

Before moving on, take a moment to reflect on what you've learned:

### Self-Assessment Questions

1. **Can you explain [concept] in your own words?**
   - If yes, try explaining it to someone else or writing it down
   - If no, review [section reference]

2. **Can you identify when to use [technique A] vs [technique B]?**
   - If yes, try creating a decision tree or comparison table
   - If no, review [section reference]

3. **Can you implement [skill] without looking at the examples?**
   - If yes, try the exercise below
   - If no, review the code examples in [section reference]

### Quick Check

Try this quick exercise to verify your understanding:

**Task**: [Simple task that tests the concept]

**Expected Result**: [What should happen]

**If you're stuck**: [Hint or reference to review]

---

**Ready to continue?** If you can confidently answer "yes" to the questions above, you're ready for the next section. If not, take time to review - building a strong foundation now will make later topics easier.
```

### Checkpoint Example

```markdown
## Checkpoint: ROS 2 Topics

Before moving on to services, take a moment to reflect on what you've learned about topics:

### Self-Assessment Questions

1. **Can you explain the publish-subscribe pattern in your own words?**
   - If yes, try explaining it to someone else or writing it down
   - If no, review Section 3.1: "Understanding Topics"

2. **Can you identify when to use topics vs other communication patterns?**
   - If yes, try creating a decision tree for pattern selection
   - If no, review Section 3.4: "Choosing Communication Patterns"

3. **Can you create a publisher and subscriber without looking at the examples?**
   - If yes, try Exercise 3.1 below
   - If no, review the code examples in Section 3.2

### Quick Check

Try this quick exercise to verify your understanding:

**Task**: Create a publisher that sends the current time every second

**Expected Result**: When you run `ros2 topic echo /time`, you should see timestamp messages

**If you're stuck**: Review Example 3.2 (Simple Publisher) and adapt it to publish time instead of a counter

---

**Ready to continue?** If you can confidently answer "yes" to the questions above, you're ready to learn about services. If not, take time to review - understanding topics thoroughly will make services and actions much easier to grasp.
```

---

## Exercise Template

```yaml
exercise:
  id: "EX-[X].[Y]"
  type: exercise
  title: "[Exercise Title]"
  chapter: [X]
  difficulty: [beginner|intermediate|advanced]
  estimated_time: [15-60] # minutes
  learning_outcomes:
    - "LO-[X].[Y]"
  prerequisites:
    - "[Concept or skill]"
```

### Exercise Format

```markdown
## Exercise [X].[Y]: [Title]

**Difficulty**: [Beginner|Intermediate|Advanced]
**Estimated Time**: [15-60] minutes
**Learning Outcome**: [LO-X.Y]

### Objective

[Clear statement of what to accomplish]

### Requirements

- [ ] [Requirement 1 - specific and testable]
- [ ] [Requirement 2 - specific and testable]
- [ ] [Requirement 3 - specific and testable]

### Setup

[Any necessary setup steps]

```bash
[Setup commands]
```

### Starter Code (Optional)

```[language]
[Template or partial code to get started]
```

### Instructions

**Step 1**: [First step]
- [Guidance or hint]

**Step 2**: [Second step]
- [Guidance or hint]

**Step 3**: [Third step]
- [Guidance or hint]

### Testing Your Solution

[How to verify the solution works]

```bash
[Test commands]
```

**Expected Output**:
```
[What you should see]
```

### Hints

<details>
<summary>Hint 1 (Click if you're stuck on Step 1)</summary>

[Helpful hint without giving away the answer]
</details>

<details>
<summary>Hint 2 (Click if you're stuck on Step 2)</summary>

[Helpful hint without giving away the answer]
</details>

### Solution

<details>
<summary>Click to reveal solution</summary>

```[language]
[Complete solution with comments]
```

**Explanation**:
[Why this solution works]
[Key decisions and trade-offs]
[Alternative approaches]

</details>

### Extension Challenges (Optional)

For additional practice, try:
- [ ] [Extension 1 - adds complexity]
- [ ] [Extension 2 - explores alternative approach]
- [ ] [Extension 3 - integrates with other concepts]
```

### Exercise Example

```markdown
## Exercise 3.1: Build a Temperature Monitor

**Difficulty**: Beginner
**Estimated Time**: 30 minutes
**Learning Outcome**: LO-3.1 (Implement publish-subscribe communication using topics)

### Objective

Create a ROS 2 system with two nodes: one that publishes simulated temperature readings, and another that subscribes and displays warnings when temperature exceeds a threshold.

### Requirements

- [ ] Publisher node sends temperature readings every second
- [ ] Subscriber node receives and displays all readings
- [ ] Subscriber warns when temperature > 30°C
- [ ] Both nodes use proper ROS 2 patterns

### Setup

```bash
# Create a new package
cd ~/ros2_ws/src
ros2 pkg create temperature_monitor --build-type ament_python --dependencies rclpy std_msgs
```

### Starter Code

```python
# temperature_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        # TODO: Create publisher
        # TODO: Create timer

    def publish_temperature(self):
        # TODO: Generate random temperature (20-35°C)
        # TODO: Publish temperature
        pass
```

### Instructions

**Step 1**: Complete the publisher
- Create a publisher for Float32 messages on the '/temperature' topic
- Create a timer that calls `publish_temperature` every second
- Generate a random temperature between 20-35°C
- Publish the temperature and log it

**Step 2**: Create the subscriber
- Create a new file `temperature_subscriber.py`
- Subscribe to the '/temperature' topic
- In the callback, display the temperature
- If temperature > 30, log a warning

**Step 3**: Test your system
- Build your package
- Run both nodes in separate terminals
- Verify warnings appear when temperature exceeds 30°C

### Testing Your Solution

```bash
# Terminal 1
ros2 run temperature_monitor temperature_publisher

# Terminal 2
ros2 run temperature_monitor temperature_subscriber

# Terminal 3 (verify topic)
ros2 topic echo /temperature
```

**Expected Output**:
```
[temperature_publisher]: Publishing: 25.3°C
[temperature_subscriber]: Current temperature: 25.3°C
[temperature_publisher]: Publishing: 31.7°C
[temperature_subscriber]: WARNING: High temperature: 31.7°C
```

### Hints

<details>
<summary>Hint 1 (Publisher creation)</summary>

Use `self.create_publisher(Float32, '/temperature', 10)` to create the publisher. The 10 is the queue size.
</details>

<details>
<summary>Hint 2 (Random temperature)</summary>

Use `random.uniform(20.0, 35.0)` to generate a random float between 20 and 35.
</details>

<details>
<summary>Hint 3 (Subscriber callback)</summary>

The callback receives a Float32 message. Access the temperature with `msg.data`.
</details>

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
# temperature_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Float32, '/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)

    def publish_temperature(self):
        temp = random.uniform(20.0, 35.0)
        msg = Float32()
        msg.data = temp
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {temp:.1f}°C')

def main():
    rclpy.init()
    node = TemperaturePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

# temperature_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32, '/temperature', self.temperature_callback, 10)

    def temperature_callback(self, msg):
        temp = msg.data
        if temp > 30.0:
            self.get_logger().warn(f'WARNING: High temperature: {temp:.1f}°C')
        else:
            self.get_logger().info(f'Current temperature: {temp:.1f}°C')

def main():
    rclpy.init()
    node = TemperatureSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

**Explanation**:
- Publisher creates a Float32 publisher and timer in `__init__`
- Timer calls `publish_temperature` every second
- Random temperature generated with `random.uniform()`
- Subscriber creates subscription in `__init__`
- Callback checks temperature and logs appropriately
- Both use proper ROS 2 patterns (spin, shutdown)

**Alternative Approaches**:
- Could use custom message type instead of Float32
- Could add timestamp to messages
- Could implement moving average for smoother readings

</details>

### Extension Challenges

For additional practice, try:
- [ ] Add a third node that calculates and publishes the average temperature over the last 10 readings
- [ ] Modify to use a custom message type that includes timestamp and sensor ID
- [ ] Implement different warning levels (caution at 28°C, warning at 30°C, critical at 33°C)
```

---

## Quality Checklist

### Quiz Quality
- [ ] 5-10 questions per quiz
- [ ] Questions test understanding, not memorization
- [ ] All options plausible (no obvious wrong answers)
- [ ] Explanations provided for all answers
- [ ] Questions map to learning outcomes
- [ ] Passing score reasonable (70-90%)

### Checkpoint Quality
- [ ] Placed at logical break points
- [ ] Questions encourage reflection
- [ ] Links to review material provided
- [ ] Quick check is simple and relevant
- [ ] Self-assessment, not graded

### Exercise Quality
- [ ] Clear, achievable objective
- [ ] Requirements specific and testable
- [ ] Starter code provided (if appropriate)
- [ ] Step-by-step instructions
- [ ] Hints available without revealing solution
- [ ] Complete solution with explanation
- [ ] Testing instructions provided
- [ ] Extension challenges for advanced learners

---

## Notes

- Assessments should feel like learning opportunities, not tests
- Provide immediate, constructive feedback
- Allow unlimited attempts for quizzes
- Make solutions available after genuine attempt
- Align all assessments with learning outcomes
- Test assessments before publishing

---

**Version History**:
- 1.0 (2026-02-09): Initial template created
