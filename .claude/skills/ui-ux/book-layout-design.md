# Skill: Book Layout Design

## Purpose
Design premium, book-quality layouts for long-form technical content that optimize readability, visual hierarchy, and learning experience.

## Responsibility
Create layout specifications that balance aesthetics, usability, and pedagogical effectiveness for digital book platforms.

## When to Use
- Designing page layouts for chapters
- Establishing visual hierarchy
- Creating reading flow patterns
- Optimizing for long-form content consumption
- Defining responsive layout behavior

## Core Capabilities

### 1. Visual Hierarchy
- Establish clear typographic scale
- Define heading levels and their visual weight
- Create visual rhythm through spacing
- Use whitespace strategically
- Guide eye movement through layout

### 2. Reading Experience
- Optimize line length for readability (45-75 characters)
- Set appropriate line height (1.5-1.8 for body text)
- Choose readable font sizes (16-18px base)
- Create comfortable margins and padding
- Minimize visual distractions

### 3. Content Structure
- Design chapter opening layouts
- Create section break patterns
- Define code block presentation
- Establish aside/callout styling
- Design navigation elements

### 4. Responsive Behavior
- Define breakpoints for different devices
- Adapt layouts for mobile, tablet, desktop
- Maintain readability across screen sizes
- Optimize touch targets for mobile
- Handle navigation on small screens

### 5. Premium Aesthetics
- Create sophisticated, modern design
- Use subtle animations and transitions
- Implement elegant dark mode
- Design beautiful typography
- Add polish through micro-interactions

## Layout Specifications

### Page Structure
```
┌─────────────────────────────────────────┐
│           Navigation Bar                 │
├─────────────────────────────────────────┤
│                                          │
│  ┌────────┐  ┌──────────────┐  ┌─────┐ │
│  │        │  │              │  │     │ │
│  │ Side   │  │   Main       │  │ TOC │ │
│  │ bar    │  │   Content    │  │     │ │
│  │        │  │              │  │     │ │
│  └────────┘  └──────────────┘  └─────┘ │
│                                          │
└─────────────────────────────────────────┘
```

### Content Width Guidelines
- **Maximum content width**: 800-900px (optimal reading)
- **Sidebar width**: 250-300px
- **TOC width**: 200-250px
- **Margins**: 40-60px on desktop, 20-30px on mobile
- **Gutter between columns**: 40-60px

### Typography Scale
```
H1 (Chapter Title):    2.5rem (40px) - Bold - 1.2 line-height
H2 (Major Section):    2rem (32px) - Bold - 1.3 line-height
H3 (Subsection):       1.5rem (24px) - Semibold - 1.4 line-height
H4 (Minor Heading):    1.25rem (20px) - Semibold - 1.4 line-height
Body Text:             1.125rem (18px) - Regular - 1.7 line-height
Code Inline:           1rem (16px) - Monospace
Code Block:            0.95rem (15px) - Monospace - 1.6 line-height
Caption:               0.95rem (15px) - Regular - 1.5 line-height
```

### Spacing System
```
XXS: 4px   - Tight spacing (inline elements)
XS:  8px   - Close spacing (related items)
S:   16px  - Default spacing (paragraphs)
M:   24px  - Section spacing
L:   40px  - Major section breaks
XL:  64px  - Chapter sections
XXL: 96px  - Chapter breaks
```

### Color Palette (Light Mode)
```
Background:     #FFFFFF
Surface:        #F8F9FA
Text Primary:   #1A1A1A
Text Secondary: #6B7280
Text Tertiary:  #9CA3AF
Accent:         #3B82F6
Border:         #E5E7EB
Code BG:        #F3F4F6
```

### Color Palette (Dark Mode)
```
Background:     #0F1419
Surface:        #1A1F2E
Text Primary:   #E5E7EB
Text Secondary: #9CA3AF
Text Tertiary:  #6B7280
Accent:         #60A5FA
Border:         #374151
Code BG:        #1E293B
```

## Layout Patterns

### Chapter Opening
```markdown
┌─────────────────────────────────────┐
│                                     │
│         Chapter 3                   │  ← Small, uppercase
│                                     │
│    Understanding ROS 2 Nodes        │  ← Large, bold title
│                                     │
│    Learn how to create independent  │  ← Subtitle/description
│    processes that communicate...    │
│                                     │
│    ⏱ 45 min read  📊 Intermediate  │  ← Metadata
│                                     │
└─────────────────────────────────────┘
```

### Section Break
```markdown
────────────────────────────────────────

## Section Title

Brief introduction to the section...
```

### Code Block Layout
```markdown
┌─────────────────────────────────────┐
│ 📄 example.py              [Copy]   │  ← Header with filename
├─────────────────────────────────────┤
│                                     │
│  1  import rclpy                    │  ← Line numbers
│  2  from std_msgs.msg import String │
│  3                                  │
│  4  # Create a simple publisher    │
│                                     │
└─────────────────────────────────────┘
```

### Callout/Admonition
```markdown
┌─────────────────────────────────────┐
│ 💡 Key Insight                      │
│                                     │
│ ROS 2 nodes are independent         │
│ processes that can run on           │
│ different machines.                 │
└─────────────────────────────────────┘
```

### Side-by-Side Comparison
```markdown
┌──────────────────┐  ┌──────────────────┐
│   Approach A     │  │   Approach B     │
│                  │  │                  │
│   [Content]      │  │   [Content]      │
│                  │  │                  │
└──────────────────┘  └─────────────────┘
```

## Responsive Breakpoints

### Desktop (1200px+)
- Three-column layout (sidebar, content, TOC)
- Full navigation visible
- Optimal reading width maintained
- Side-by-side comparisons

### Tablet (768px - 1199px)
- Two-column layout (sidebar + content OR content + TOC)
- Collapsible sidebar
- Maintained reading comfort
- Stacked comparisons

### Mobile (< 768px)
- Single column layout
- Hamburger menu for navigation
- Full-width content with padding
- Stacked all elements
- Larger touch targets (44px minimum)

## Quality Standards

### Readability
- Line length: 45-75 characters
- Line height: 1.5-1.8 for body text
- Font size: Minimum 16px for body text
- Contrast ratio: Minimum 4.5:1 (WCAG AA)
- Whitespace: Generous, not cramped

### Visual Hierarchy
- Clear distinction between heading levels
- Consistent spacing patterns
- Logical content grouping
- Visual flow guides reading
- Important elements stand out

### Performance
- Minimal layout shifts
- Fast font loading
- Optimized images
- Smooth scrolling
- No jank or flicker

### Accessibility
- Semantic HTML structure
- Keyboard navigation support
- Screen reader friendly
- Focus indicators visible
- Color not sole indicator

## Integration Points
- Implements content structure from specs
- Provides framework for motion design
- Establishes Docusaurus theme requirements
- Creates canvas for educational content
- Supports error message presentation
