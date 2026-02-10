# Skill: Scroll-Based Animation Design

## Purpose
Design scroll-triggered animations that enhance learning by revealing content progressively and creating visual interest without distraction.

## Responsibility
Create animation specifications for scroll-based reveals, transitions, and effects that support pedagogical goals and maintain performance.

## When to Use
- Designing chapter and section reveals
- Creating progressive disclosure patterns
- Adding visual interest to long-form content
- Guiding attention to key concepts
- Enhancing reading flow

## Core Capabilities

### 1. Reveal Animation Design
- Define entrance animations for content blocks
- Specify timing and easing functions
- Create staggered reveal patterns
- Design fade, slide, and scale effects
- Maintain readability during animation

### 2. Scroll Trigger Configuration
- Define scroll thresholds for animations
- Specify viewport intersection points
- Create scroll-linked progress effects
- Design parallax and depth effects
- Handle bidirectional scrolling

### 3. Performance Optimization
- Use GPU-accelerated properties (transform, opacity)
- Avoid layout-triggering animations
- Implement intersection observers
- Debounce scroll events
- Lazy-load animation libraries

### 4. Pedagogical Alignment
- Use animation to guide attention
- Reveal content in logical order
- Support cognitive processing time
- Avoid overwhelming learners
- Enhance, don't distract

### 5. Accessibility Considerations
- Respect prefers-reduced-motion
- Provide instant-reveal fallback
- Maintain keyboard navigation
- Ensure content remains accessible
- Test with screen readers

## Animation Patterns

### Pattern 1: Fade-In on Scroll
**Use for**: Paragraphs, images, diagrams
**Effect**: Content fades from transparent to opaque
**Timing**: 400-600ms
**Easing**: ease-out

```css
.fade-in {
  opacity: 0;
  transition: opacity 0.5s ease-out;
}

.fade-in.visible {
  opacity: 1;
}
```

**Trigger**: When element enters viewport (20% visible)

### Pattern 2: Slide-Up Reveal
**Use for**: Headings, callouts, key concepts
**Effect**: Content slides up while fading in
**Timing**: 500-700ms
**Easing**: cubic-bezier(0.4, 0, 0.2, 1)

```css
.slide-up {
  opacity: 0;
  transform: translateY(30px);
  transition: opacity 0.6s ease-out,
              transform 0.6s cubic-bezier(0.4, 0, 0.2, 1);
}

.slide-up.visible {
  opacity: 1;
  transform: translateY(0);
}
```

**Trigger**: When element enters viewport (30% visible)

### Pattern 3: Staggered List Reveal
**Use for**: Bullet points, numbered lists, key takeaways
**Effect**: Items appear sequentially with delay
**Timing**: 300ms per item, 100ms stagger
**Easing**: ease-out

```css
.stagger-item {
  opacity: 0;
  transform: translateX(-20px);
  transition: opacity 0.3s ease-out,
              transform 0.3s ease-out;
}

.stagger-item:nth-child(1) { transition-delay: 0ms; }
.stagger-item:nth-child(2) { transition-delay: 100ms; }
.stagger-item:nth-child(3) { transition-delay: 200ms; }
/* ... */

.stagger-item.visible {
  opacity: 1;
  transform: translateX(0);
}
```

**Trigger**: When parent container enters viewport

### Pattern 4: Scale-In for Emphasis
**Use for**: Important callouts, warnings, key insights
**Effect**: Content scales from 95% to 100% while fading in
**Timing**: 400ms
**Easing**: ease-out

```css
.scale-in {
  opacity: 0;
  transform: scale(0.95);
  transition: opacity 0.4s ease-out,
              transform 0.4s ease-out;
}

.scale-in.visible {
  opacity: 1;
  transform: scale(1);
}
```

**Trigger**: When element is 50% visible

### Pattern 5: Code Block Reveal
**Use for**: Code examples
**Effect**: Fade in with subtle slide-up
**Timing**: 500ms
**Easing**: ease-out

```css
.code-reveal {
  opacity: 0;
  transform: translateY(20px);
  transition: opacity 0.5s ease-out,
              transform 0.5s ease-out;
}

.code-reveal.visible {
  opacity: 1;
  transform: translateY(0);
}
```

**Trigger**: When code block is 40% visible

### Pattern 6: Diagram Fade-In
**Use for**: Images, diagrams, visualizations
**Effect**: Gentle fade with slight scale
**Timing**: 600ms
**Easing**: ease-out

```css
.diagram-reveal {
  opacity: 0;
  transform: scale(0.98);
  transition: opacity 0.6s ease-out,
              transform 0.6s ease-out;
}

.diagram-reveal.visible {
  opacity: 1;
  transform: scale(1);
}
```

**Trigger**: When image is 30% visible

## Scroll Progress Indicators

### Reading Progress Bar
```css
.progress-bar {
  position: fixed;
  top: 0;
  left: 0;
  height: 3px;
  background: linear-gradient(to right, #3B82F6, #60A5FA);
  transform-origin: left;
  transform: scaleX(0);
  transition: transform 0.1s ease-out;
}
```

**Behavior**: Grows from 0% to 100% as user scrolls through chapter

### Section Progress Indicator
```css
.section-progress {
  width: 4px;
  background: #E5E7EB;
  position: fixed;
  left: 20px;
  top: 100px;
  height: 300px;
}

.section-progress-fill {
  width: 100%;
  background: #3B82F6;
  transform-origin: top;
  transform: scaleY(0);
  transition: transform 0.2s ease-out;
}
```

**Behavior**: Fills as user progresses through current section

## Animation Specifications

### Timing Guidelines
- **Micro-interactions**: 100-200ms (hover, focus)
- **Content reveals**: 400-600ms (paragraphs, images)
- **Section transitions**: 600-800ms (major content blocks)
- **Page transitions**: 300-500ms (navigation)

### Easing Functions
- **ease-out**: Default for most reveals (fast start, slow end)
- **ease-in-out**: Smooth transitions (equal acceleration/deceleration)
- **cubic-bezier(0.4, 0, 0.2, 1)**: Material Design standard
- **spring**: Natural, bouncy feel (use sparingly)

### Viewport Thresholds
- **Headings**: Trigger at 30% visible
- **Paragraphs**: Trigger at 20% visible
- **Images**: Trigger at 30% visible
- **Code blocks**: Trigger at 40% visible
- **Callouts**: Trigger at 50% visible

## Performance Best Practices

### GPU-Accelerated Properties
✅ **Use**: transform, opacity
❌ **Avoid**: width, height, top, left, margin, padding

### Intersection Observer Pattern
```javascript
const observer = new IntersectionObserver(
  (entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        entry.target.classList.add('visible');
      }
    });
  },
  {
    threshold: 0.2, // Trigger at 20% visible
    rootMargin: '0px 0px -100px 0px' // Offset trigger point
  }
);

// Observe elements
document.querySelectorAll('.fade-in').forEach(el => {
  observer.observe(el);
});
```

### Reduced Motion Support
```css
@media (prefers-reduced-motion: reduce) {
  * {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }

  .fade-in,
  .slide-up,
  .scale-in {
    opacity: 1;
    transform: none;
  }
}
```

## Quality Standards

### Performance
- Animations run at 60fps
- No layout thrashing
- Smooth on mobile devices
- Minimal JavaScript overhead
- Efficient intersection observers

### Pedagogy
- Animations support learning
- Content reveals logically
- No distraction from reading
- Timing allows comprehension
- Enhances, doesn't overwhelm

### Accessibility
- Respects prefers-reduced-motion
- Content accessible without animation
- Keyboard navigation unaffected
- Screen readers work correctly
- Focus management maintained

### Visual Quality
- Smooth, polished animations
- Consistent timing across site
- Appropriate easing functions
- No jarring movements
- Professional feel

## Integration Points
- Enhances book layout design
- Supports navigation patterns
- Highlights key content
- Guides reading flow
- Creates premium experience
