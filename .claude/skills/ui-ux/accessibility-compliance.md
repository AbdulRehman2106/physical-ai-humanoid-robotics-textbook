# Skill: Accessibility Compliance

## Purpose
Ensure all content, interfaces, and interactions meet WCAG 2.1 AA accessibility standards, making the book usable by everyone.

## Responsibility
Design and verify accessibility features that enable learners with disabilities to access and benefit from all content and functionality.

## When to Use
- Designing new UI components
- Creating content
- Implementing interactions
- Reviewing existing content
- Testing user experiences

## Core Capabilities

### 1. Perceivable Content
- Provide text alternatives
- Create captions and transcripts
- Ensure color contrast
- Enable text resizing
- Support multiple modalities

### 2. Operable Interface
- Enable keyboard navigation
- Provide sufficient time
- Avoid seizure triggers
- Support navigation aids
- Enable input flexibility

### 3. Understandable Information
- Use clear language
- Provide predictable behavior
- Offer input assistance
- Explain errors clearly
- Support comprehension

### 4. Robust Compatibility
- Use semantic HTML
- Support assistive technologies
- Ensure cross-browser compatibility
- Enable graceful degradation
- Test with screen readers

### 5. Compliance Verification
- Conduct accessibility audits
- Test with assistive technologies
- Validate against WCAG criteria
- Document accessibility features
- Maintain compliance

## WCAG 2.1 AA Requirements

### Level A (Must Have)

#### 1.1 Text Alternatives
```markdown
## Implementation

**Images**:
```html
<img src="ros2-architecture.png"
     alt="ROS 2 architecture diagram showing three layers: DDS middleware at bottom, ROS 2 core in middle, and application layer on top">
```

**Complex Diagrams**:
```html
<figure>
  <img src="data-flow.png" alt="Data flow diagram">
  <figcaption>
    Detailed description: Data flows from sensors (camera, lidar, IMU)
    through processing nodes (sensor fusion, object detection) to the
    path planner, which sends commands to the motor controller.
  </figcaption>
</figure>
```

**Decorative Images**:
```html
<img src="decoration.png" alt="" role="presentation">
```

**Icons with Meaning**:
```html
<button aria-label="Close dialog">
  <span class="icon-close" aria-hidden="true">×</span>
</button>
```
```

#### 1.2 Time-Based Media
```markdown
## Video Requirements

**Captions**:
- Provide synchronized captions for all video content
- Include speaker identification
- Describe relevant sounds
- Use proper punctuation

**Transcripts**:
- Provide full text transcripts
- Include descriptions of visual content
- Note important actions or demonstrations
- Make downloadable

**Example**:
```html
<video controls>
  <source src="ros2-tutorial.mp4" type="video/mp4">
  <track kind="captions" src="captions-en.vtt" srclang="en" label="English">
  <track kind="descriptions" src="descriptions-en.vtt" srclang="en">
</video>

<details>
  <summary>Video Transcript</summary>
  <p>[00:00] Instructor: Welcome to this ROS 2 tutorial...</p>
  <p>[00:15] [Screen shows terminal with command]...</p>
</details>
```
```

#### 1.3 Adaptable Content
```markdown
## Responsive Design

**Semantic Structure**:
```html
<article>
  <header>
    <h1>Chapter 3: ROS 2 Nodes</h1>
    <p class="chapter-meta">Estimated time: 45 minutes</p>
  </header>

  <section>
    <h2>Introduction</h2>
    <p>Content...</p>
  </section>

  <section>
    <h2>Creating Nodes</h2>
    <p>Content...</p>
  </section>
</article>
```

**Reading Order**:
- Ensure logical tab order
- Maintain meaningful sequence
- Support linearization
- Test with CSS disabled
```

#### 1.4 Distinguishable
```markdown
## Color and Contrast

**Minimum Contrast Ratios**:
- Normal text: 4.5:1
- Large text (18pt+): 3:1
- UI components: 3:1
- Graphics: 3:1

**Color Usage**:
```css
/* BAD: Color only */
.error { color: red; }

/* GOOD: Color + icon + text */
.error {
  color: #DC2626;
  border-left: 4px solid #DC2626;
}
.error::before {
  content: "⚠️ Error: ";
  font-weight: bold;
}
```

**Testing**:
- Use contrast checker tools
- Test with color blindness simulators
- Verify in grayscale
- Check with high contrast mode
```

#### 2.1 Keyboard Accessible
```markdown
## Keyboard Navigation

**All Functionality**:
```javascript
// Interactive elements must be keyboard accessible
<button onClick={handleClick} onKeyPress={handleKeyPress}>
  Run Code
</button>

// Custom interactive elements need keyboard support
<div
  role="button"
  tabIndex={0}
  onClick={handleClick}
  onKeyPress={(e) => {
    if (e.key === 'Enter' || e.key === ' ') {
      handleClick();
    }
  }}
>
  Interactive Element
</div>
```

**Focus Management**:
```css
/* Visible focus indicators */
*:focus {
  outline: 2px solid #3B82F6;
  outline-offset: 2px;
}

/* Never remove focus without replacement */
button:focus {
  outline: none; /* BAD */
}

button:focus-visible {
  outline: 2px solid #3B82F6; /* GOOD */
  outline-offset: 2px;
}
```

**Keyboard Shortcuts**:
```markdown
## Documented Shortcuts

| Key | Action |
|-----|--------|
| Tab | Move to next element |
| Shift+Tab | Move to previous element |
| Enter | Activate button/link |
| Space | Activate button, toggle checkbox |
| Esc | Close dialog/modal |
| Arrow keys | Navigate within component |
```
```

#### 2.4 Navigable
```markdown
## Navigation Aids

**Skip Links**:
```html
<a href="#main-content" class="skip-link">
  Skip to main content
</a>

<main id="main-content">
  <!-- Content -->
</main>
```

**Page Titles**:
```html
<title>Chapter 3: ROS 2 Nodes | Physical AI Book</title>
```

**Headings**:
```html
<!-- Proper heading hierarchy -->
<h1>Chapter 3: ROS 2 Nodes</h1>
  <h2>Introduction</h2>
  <h2>Creating Nodes</h2>
    <h3>Node Initialization</h3>
    <h3>Node Configuration</h3>
  <h2>Communication</h2>
```

**Breadcrumbs**:
```html
<nav aria-label="Breadcrumb">
  <ol>
    <li><a href="/">Home</a></li>
    <li><a href="/chapters">Chapters</a></li>
    <li aria-current="page">ROS 2 Nodes</li>
  </ol>
</nav>
```
```

#### 3.1 Readable
```markdown
## Language and Readability

**Language Declaration**:
```html
<html lang="en">
```

**Language Changes**:
```html
<p>The ROS 2 command is <code lang="bash">ros2 run</code></p>
```

**Reading Level**:
- Use clear, concise language
- Define technical terms
- Provide glossary
- Use active voice
- Break up long paragraphs
```

#### 3.2 Predictable
```markdown
## Consistent Behavior

**Navigation**:
- Same navigation on all pages
- Consistent component placement
- Predictable interactions
- Clear current location

**Focus Order**:
```javascript
// Maintain logical focus order
// Don't use positive tabindex values
<div tabIndex={0}>  // Good
<div tabIndex={5}>  // Bad - breaks natural order
```

**Context Changes**:
```javascript
// Warn before context changes
<form onSubmit={handleSubmit}>
  <button type="submit">
    Submit (will navigate to results page)
  </button>
</form>
```
```

#### 3.3 Input Assistance
```markdown
## Error Prevention and Correction

**Labels**:
```html
<label for="node-name">Node Name:</label>
<input
  id="node-name"
  type="text"
  aria-describedby="node-name-help"
  required
>
<span id="node-name-help">
  Use lowercase letters and underscores only
</span>
```

**Error Messages**:
```html
<input
  id="email"
  type="email"
  aria-invalid="true"
  aria-describedby="email-error"
>
<span id="email-error" role="alert">
  Error: Please enter a valid email address
</span>
```

**Suggestions**:
```html
<label for="topic-name">Topic Name:</label>
<input
  id="topic-name"
  list="topic-suggestions"
  aria-describedby="topic-help"
>
<datalist id="topic-suggestions">
  <option value="/cmd_vel">
  <option value="/scan">
  <option value="/odom">
</datalist>
```
```

### Level AA (Should Have)

#### 1.4.3 Contrast (Minimum)
```markdown
## Contrast Requirements

**Text Contrast**:
- Normal text: 4.5:1 minimum
- Large text: 3:1 minimum
- Incidental text: No requirement

**Testing**:
```css
/* Check contrast ratios */
.text-primary {
  color: #1A1A1A;  /* On white: 16.1:1 ✓ */
  background: #FFFFFF;
}

.text-secondary {
  color: #6B7280;  /* On white: 5.4:1 ✓ */
  background: #FFFFFF;
}

.link {
  color: #2563EB;  /* On white: 8.6:1 ✓ */
}
```
```

#### 1.4.10 Reflow
```markdown
## Responsive Content

**No Horizontal Scrolling**:
- Content reflows at 320px width
- No loss of information
- No horizontal scrolling required
- Zoom up to 400% supported

**Implementation**:
```css
/* Responsive typography */
.content {
  max-width: 100%;
  font-size: clamp(16px, 2vw, 18px);
  line-height: 1.7;
}

/* Responsive images */
img {
  max-width: 100%;
  height: auto;
}

/* Responsive code blocks */
pre {
  max-width: 100%;
  overflow-x: auto;
}
```
```

#### 2.4.7 Focus Visible
```markdown
## Focus Indicators

**Always Visible**:
```css
/* Clear focus indicators */
*:focus-visible {
  outline: 2px solid #3B82F6;
  outline-offset: 2px;
  border-radius: 4px;
}

/* High contrast mode support */
@media (prefers-contrast: high) {
  *:focus-visible {
    outline: 3px solid currentColor;
    outline-offset: 3px;
  }
}
```
```

## Accessibility Testing Checklist

### Automated Testing
- [ ] Run axe DevTools
- [ ] Run WAVE browser extension
- [ ] Run Lighthouse accessibility audit
- [ ] Validate HTML
- [ ] Check color contrast

### Manual Testing
- [ ] Navigate with keyboard only
- [ ] Test with screen reader (NVDA/JAWS/VoiceOver)
- [ ] Zoom to 200% and 400%
- [ ] Test in high contrast mode
- [ ] Disable CSS and check structure
- [ ] Test with color blindness simulator

### Screen Reader Testing
```markdown
## Screen Reader Checklist

**NVDA (Windows)**:
- [ ] All content is announced
- [ ] Headings are navigable (H key)
- [ ] Links are clear (K key)
- [ ] Forms are labeled
- [ ] Images have alt text
- [ ] Tables are structured

**VoiceOver (Mac)**:
- [ ] Rotor navigation works
- [ ] All interactive elements accessible
- [ ] Proper ARIA labels
- [ ] Landmarks identified

**Testing Commands**:
- NVDA: Insert+Down (read next)
- VoiceOver: VO+Right (next item)
- Both: Tab (next focusable)
```

## Common Accessibility Patterns

### Modal Dialogs
```jsx
<div
  role="dialog"
  aria-labelledby="dialog-title"
  aria-describedby="dialog-description"
  aria-modal="true"
>
  <h2 id="dialog-title">Confirm Action</h2>
  <p id="dialog-description">
    Are you sure you want to delete this node?
  </p>
  <button onClick={handleConfirm}>Confirm</button>
  <button onClick={handleCancel}>Cancel</button>
</div>
```

### Tabs
```jsx
<div role="tablist" aria-label="Code examples">
  <button
    role="tab"
    aria-selected="true"
    aria-controls="panel-python"
    id="tab-python"
  >
    Python
  </button>
  <button
    role="tab"
    aria-selected="false"
    aria-controls="panel-cpp"
    id="tab-cpp"
  >
    C++
  </button>
</div>

<div
  role="tabpanel"
  id="panel-python"
  aria-labelledby="tab-python"
>
  <!-- Python code -->
</div>
```

### Accordions
```jsx
<button
  aria-expanded={isOpen}
  aria-controls="section-content"
  id="section-button"
>
  Advanced Topics
</button>

<div
  id="section-content"
  aria-labelledby="section-button"
  hidden={!isOpen}
>
  <!-- Content -->
</div>
```

## Quality Standards

### Compliance
- Meets WCAG 2.1 AA standards
- Passes automated testing
- Verified with manual testing
- Screen reader compatible
- Keyboard accessible

### Usability
- Clear and predictable
- Easy to navigate
- Forgiving of errors
- Provides helpful feedback
- Works with assistive tech

### Documentation
- Accessibility features documented
- Testing procedures defined
- Known issues tracked
- Remediation plans created
- Regular audits scheduled

## Integration Points
- Applies to all UI/UX design
- Informs content creation
- Guides interaction design
- Ensures inclusive experience
- Maintains legal compliance
