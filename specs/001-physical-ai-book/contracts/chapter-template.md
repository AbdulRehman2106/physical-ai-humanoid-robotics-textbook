# Chapter Template

**Purpose**: Standardized structure for all book chapters
**Version**: 1.0
**Last Updated**: 2026-02-09

## Chapter Metadata

```yaml
chapter:
  number: [1-15]
  title: "[Descriptive Chapter Title]"
  slug: "[url-friendly-identifier]"
  learning_outcomes:
    - "[LO-X.1] [Measurable objective using action verb]"
    - "[LO-X.2] [Measurable objective using action verb]"
    - "[LO-X.3] [Measurable objective using action verb]"
  prerequisites:
    - "[Chapter Y: Title]"
    - "[Concept from Chapter Z]"
  estimated_time: [180-240] # minutes
  difficulty_level: [beginner|intermediate|advanced]
  week_number: [1-12]
  word_count_target: [1000-3000]
```

## Chapter Structure

### 1. Introduction (10-15% of word count)

**Purpose**: Hook the reader, establish context, preview content

**Required Elements**:
- Opening hook (real-world scenario, interesting question, or compelling fact)
- Why this topic matters (practical relevance)
- What you'll learn (preview of learning outcomes)
- How it connects to previous chapters (if applicable)

**Template**:
```markdown
# Chapter [X]: [Title]

[Opening hook - 2-3 sentences that grab attention]

[Why this matters - 2-3 sentences on practical relevance]

## What You'll Learn

By the end of this chapter, you will be able to:
- [Learning outcome 1]
- [Learning outcome 2]
- [Learning outcome 3]

[Connection to previous chapters - 1-2 sentences]
```

**Example**:
```markdown
# Chapter 3: ROS 2 Communication Patterns

Imagine a robot navigating a warehouse: its camera detects obstacles, its planner computes a path, and its motors execute movements. How do these independent components communicate? In ROS 2, choosing the right communication pattern—topics, services, or actions—can mean the difference between a responsive robot and a sluggish one.

Understanding ROS 2 communication patterns is essential for building efficient, scalable robotics systems. These patterns enable distributed computing, allowing robot components to run on different machines while maintaining real-time coordination.

## What You'll Learn

By the end of this chapter, you will be able to:
- Implement publish-subscribe communication using topics
- Create request-response interactions using services
- Design goal-based tasks using actions
- Choose the appropriate pattern for different scenarios

Building on Chapter 2's introduction to ROS 2 nodes, we'll now explore how nodes communicate with each other.
```

---

### 2. Theory (20-25% of word count)

**Purpose**: Present formal concepts, definitions, and principles

**Required Elements**:
- Clear definitions of key terms
- Formal explanations of concepts
- Technical specifications where applicable
- Citations to authoritative sources (APA format)

**Structure**:
```markdown
## [Section Title]: Theory

### [Concept 1]

[Formal definition]

[Technical explanation]

[Key characteristics or properties]

**Citation**: (Author, Year)

### [Concept 2]

[Continue pattern]
```

**Guidelines**:
- Define before using technical terms
- Use precise language
- Include citations for all technical claims
- Avoid jargon without explanation
- Build from simple to complex

---

### 3. Intuition (20-25% of word count)

**Purpose**: Build mental models through analogies and visual explanations

**Required Elements**:
- Analogies to familiar concepts
- Mental models or frameworks
- Visual content (diagrams, illustrations)
- "Why it works this way" explanations

**Structure**:
```markdown
## Understanding [Topic]: Intuition

### The Big Picture

[High-level overview without jargon]

### Mental Model: [Analogy]

[Analogy to familiar concept]

[How the analogy maps to the technical concept]

[Where the analogy breaks down - important!]

### Visual Explanation

[Reference to diagram]

![Diagram Title](path/to/diagram.svg)
*Figure X.Y: [Caption explaining what the diagram shows]*

### Why It Works This Way

[Explanation of underlying principles]

[Design rationale]
```

**Example**:
```markdown
## Understanding Topics: Intuition

### The Big Picture

Topics in ROS 2 are like radio stations broadcasting information. Any number of "radios" (subscribers) can tune in to listen, and the broadcaster (publisher) doesn't need to know who's listening.

### Mental Model: Radio Broadcasting

Think of a publisher as a radio station broadcasting on a specific frequency (the topic name). Subscribers are like radios tuned to that frequency—they receive whatever is broadcast. Multiple radios can listen to the same station, and the station broadcasts whether anyone is listening or not.

However, unlike radio, ROS 2 topics can have quality-of-service settings that guarantee message delivery, which is more like registered mail than broadcast radio.

### Visual Explanation

![ROS 2 Topic Communication](../../../static/img/diagrams/ros2-topic-pattern.svg)
*Figure 3.1: Publisher-subscriber pattern showing one-to-many communication*

### Why It Works This Way

The publish-subscribe pattern enables decoupled communication. Publishers and subscribers don't need to know about each other, which allows for:
- Flexible system architecture (add/remove components easily)
- Distributed computing (components on different machines)
- Scalability (multiple subscribers without publisher changes)
```

---

### 4. Application (30-35% of word count)

**Purpose**: Demonstrate practical implementation with hands-on examples

**Required Elements**:
- Complete, runnable code examples (3-5 minimum)
- Step-by-step tutorials
- Hands-on exercises
- Troubleshooting tips

**Structure**:
```markdown
## Implementing [Topic]: Application

### Example 1: [Simple Use Case]

**Learning Objective**: [What this example teaches]

**Code**:
```[language]
[Complete, runnable code with comments]
```

**Explanation**:
[Line-by-line or section-by-section explanation]

**Running the Example**:
```bash
[Commands to execute]
```

**Expected Output**:
```
[What you should see]
```

**Try It Yourself**:
- [ ] [Modification 1]
- [ ] [Modification 2]

### Example 2: [More Complex Use Case]

[Repeat structure]

### Hands-On Exercise

**Objective**: [What to build]

**Requirements**:
- [Requirement 1]
- [Requirement 2]

**Starter Code**: [Link or inline]

**Hints**:
1. [Hint 1]
2. [Hint 2]

**Solution**: [Link to solution, initially hidden]
```

**Code Example Guidelines**:
- Must be complete and runnable (no fragments)
- Include all necessary imports
- Add explanatory comments (why, not what)
- Show expected output
- Test before publishing
- Follow PEP 8 (Python) or language-specific style guide

---

### 5. Exercises (5-10% of word count)

**Purpose**: Provide practice opportunities

**Required Elements**:
- 2-3 hands-on exercises
- Clear objectives
- Starter code or templates (optional)
- Solutions (hidden initially)

**Structure**:
```markdown
## Practice Exercises

### Exercise 1: [Title]

**Difficulty**: [Beginner|Intermediate|Advanced]

**Objective**: [What to accomplish]

**Requirements**:
- [Requirement 1]
- [Requirement 2]

**Starter Code** (optional):
```[language]
[Template or partial code]
```

**Hints**:
1. [Hint 1 - revealed after first attempt]
2. [Hint 2 - revealed after second attempt]

**Solution**:
<details>
<summary>Click to reveal solution</summary>

```[language]
[Complete solution with explanation]
```

**Explanation**: [Why this solution works]
</details>
```

---

### 6. Assessment (5% of word count)

**Purpose**: Verify understanding

**Required Elements**:
- Quiz or checkpoint
- 5-10 questions
- Immediate feedback
- Explanations for all answers

**Structure**:
```markdown
## Chapter Assessment

**Instructions**: Answer the following questions to check your understanding. You can retry as many times as needed.

**Passing Score**: 80%

### Question 1

[Question text]

**Options**:
- A) [Option A]
- B) [Option B]
- C) [Option C]
- D) [Option D]

<details>
<summary>Show Answer</summary>

**Correct Answer**: [Letter]

**Explanation**: [Why this is correct and others are wrong]
</details>

[Repeat for all questions]
```

---

### 7. Summary (5-10% of word count)

**Purpose**: Reinforce key concepts and provide closure

**Required Elements**:
- Key takeaways (3-5 bullet points)
- Connections to other chapters
- Preview of next chapter
- Further reading (optional)

**Structure**:
```markdown
## Chapter Summary

In this chapter, you learned:

- **[Key Point 1]**: [Brief elaboration]
- **[Key Point 2]**: [Brief elaboration]
- **[Key Point 3]**: [Brief elaboration]

These concepts connect to:
- [Chapter X]: [How it connects]
- [Chapter Y]: [How it connects]

## What's Next

In the next chapter, you'll learn [preview of next topic]. Building on your understanding of [current topic], you'll be able to [what they'll accomplish].

## Further Reading

- [Resource 1]: [Why it's useful]
- [Resource 2]: [Why it's useful]
```

---

### 8. References

**Purpose**: Provide citations for all technical claims

**Required Elements**:
- APA 7th edition format
- All sources cited in chapter
- Accessible URLs where applicable

**Structure**:
```markdown
## References

[Author, A. A.] (Year). *Title of work*. Publisher. URL

[Author, B. B., & Author, C. C.] (Year). Title of article. *Journal Name*, *Volume*(Issue), pages. https://doi.org/xxx

[Organization.] (Year). *Documentation title*. Retrieved Month Day, Year, from URL
```

---

## Quality Checklist

Before submitting a chapter, verify:

### Content Quality
- [ ] Word count: 1,000-3,000
- [ ] Flesch-Kincaid grade level: 10-12
- [ ] All sections present and complete
- [ ] Theory → Intuition → Application structure maintained
- [ ] No implementation details in theory section

### Technical Accuracy
- [ ] All code examples tested and runnable
- [ ] Technical claims verified against authoritative sources
- [ ] Citations complete and properly formatted (APA)
- [ ] ROS 2 versions and dependencies specified

### Pedagogical Effectiveness
- [ ] Learning outcomes clearly defined (3-5)
- [ ] Prerequisites explicitly stated
- [ ] Cognitive load appropriate for target audience
- [ ] Exercises align with learning outcomes
- [ ] Assessment questions test understanding

### Visual Content
- [ ] Minimum 2-3 diagrams included
- [ ] All images have alt text (WCAG 2.1 AA)
- [ ] Color contrast ratio 4.5:1 minimum
- [ ] Diagrams referenced in text
- [ ] Captions explain what diagrams show

### Code Quality
- [ ] Minimum 3-5 complete code examples
- [ ] All imports included
- [ ] Explanatory comments present
- [ ] Expected output provided
- [ ] Code follows style guide (PEP 8 for Python)

### Accessibility
- [ ] WCAG 2.1 AA compliant
- [ ] Semantic HTML structure
- [ ] Keyboard navigable
- [ ] Screen reader compatible
- [ ] Animations respect prefers-reduced-motion

### Skills Compliance
- [ ] Content created using defined skills
- [ ] Skills documented and referenced
- [ ] Integration points verified
- [ ] Quality standards from skills applied

---

## Notes

- This template is a guide, not a rigid structure
- Adapt sections as needed for specific content
- Maintain consistency across all chapters
- Prioritize clarity and learning effectiveness
- Follow constitution principles at all times

---

**Version History**:
- 1.0 (2026-02-09): Initial template created
