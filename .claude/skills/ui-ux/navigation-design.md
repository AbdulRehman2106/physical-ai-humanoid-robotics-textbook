# Skill: Navigation Design

## Purpose
Design intuitive, efficient navigation systems for long-form technical books that help learners orient themselves and move through content effectively.

## Responsibility
Create navigation patterns that balance discoverability, context awareness, and ease of movement through complex educational content.

## When to Use
- Designing book navigation structure
- Creating chapter and section navigation
- Establishing breadcrumb patterns
- Designing table of contents
- Creating progress indicators

## Core Capabilities

### 1. Navigation Architecture
- Define primary navigation structure
- Establish navigation hierarchy
- Create navigation patterns for different contexts
- Design mobile navigation behavior
- Implement search and discovery

### 2. Context Awareness
- Show current location clearly
- Provide breadcrumb trails
- Display progress through content
- Highlight related sections
- Show learning path

### 3. Movement Patterns
- Enable quick chapter jumping
- Support sequential navigation (prev/next)
- Provide section-level navigation
- Create shortcuts to key sections
- Implement smooth scrolling to anchors

### 4. Discoverability
- Make all content findable
- Provide multiple navigation paths
- Show content structure clearly
- Enable search functionality
- Suggest related content

### 5. Progress Tracking
- Show completion status
- Display reading progress
- Track learning milestones
- Visualize learning path
- Celebrate achievements

## Navigation Components

### 1. Primary Sidebar Navigation
```
┌─────────────────────┐
│ 📚 Physical AI Book │
├─────────────────────┤
│                     │
│ ▼ Chapter 1: Intro  │ ← Expandable
│   • What is AI?     │ ← Section
│   • History         │
│                     │
│ ▶ Chapter 2: ROS 2  │ ← Collapsed
│                     │
│ ▼ Chapter 3: Nodes  │ ← Current chapter
│   • Overview        │
│   • Creating Nodes  │ ← Current section
│   • Communication   │
│                     │
│ ▶ Chapter 4: Topics │
│                     │
└─────────────────────┘
```

**Features**:
- Hierarchical structure (chapters → sections)
- Expand/collapse functionality
- Current location highlighted
- Completed sections marked
- Smooth scrolling to sections

### 2. Table of Contents (Right Sidebar)
```
┌─────────────────────┐
│ On This Page        │
├─────────────────────┤
│                     │
│ • Overview          │
│ • Creating Nodes    │ ← Current
│   - Setup           │
│   - Implementation  │
│ • Communication     │
│ • Best Practices    │
│                     │
└─────────────────────┘
```

**Features**:
- Shows current page structure
- Highlights active section
- Auto-scrolls with page
- Click to jump to section
- Nested heading support

### 3. Breadcrumb Navigation
```
Home > Physical AI > Chapter 3 > Creating Nodes
```

**Features**:
- Shows navigation path
- Each level is clickable
- Provides context
- Helps with orientation
- Compact and unobtrusive

### 4. Sequential Navigation
```
┌─────────────────────────────────────┐
│                                     │
│  ← Previous          Next →         │
│  Chapter 2           Chapter 4      │
│  ROS 2 Basics        Topics         │
│                                     │
└─────────────────────────────────────┘
```

**Features**:
- Previous/Next chapter links
- Shows chapter titles
- Placed at bottom of content
- Keyboard shortcuts (arrow keys)
- Swipe gestures on mobile

### 5. Progress Indicator
```
┌─────────────────────────────────────┐
│ Chapter 3: Understanding Nodes      │
│ ████████░░░░░░░░░░░░ 40% Complete  │
└─────────────────────────────────────┘
```

**Features**:
- Visual progress bar
- Percentage completion
- Per-chapter tracking
- Overall book progress
- Motivational feedback

### 6. Quick Navigation Menu
```
┌─────────────────────┐
│ Jump to:            │
├─────────────────────┤
│ 🏠 Home             │
│ 📖 All Chapters     │
│ 🔖 Bookmarks        │
│ 🎯 Learning Path    │
│ 🏆 Capstone         │
└─────────────────────┘
```

## Navigation Patterns

### Pattern 1: Hierarchical Sidebar
**Use for**: Main book navigation
**Behavior**:
- Chapters always visible
- Sections expand on click
- Current location highlighted
- Scroll to reveal more content
- Sticky positioning

### Pattern 2: Floating TOC
**Use for**: Current page navigation
**Behavior**:
- Shows page structure
- Auto-highlights on scroll
- Smooth scroll to sections
- Collapses on mobile
- Sticky positioning

### Pattern 3: Breadcrumb Trail
**Use for**: Context and orientation
**Behavior**:
- Shows full path
- Each level clickable
- Updates on navigation
- Compact on mobile
- Always visible

### Pattern 4: Search-First
**Use for**: Quick content discovery
**Behavior**:
- Prominent search bar
- Instant results
- Keyboard shortcuts (Cmd+K)
- Fuzzy matching
- Recent searches

### Pattern 5: Learning Path
**Use for**: Guided progression
**Behavior**:
- Shows recommended order
- Tracks completion
- Suggests next steps
- Allows skipping
- Visualizes progress

## Mobile Navigation

### Hamburger Menu
```
┌─────────────────────┐
│ ☰  Chapter 3    🔍  │ ← Header
├─────────────────────┤
│                     │
│   [Content]         │
│                     │
└─────────────────────┘

[Tap ☰]

┌─────────────────────┐
│ ✕  Menu             │
├─────────────────────┤
│ 📚 All Chapters     │
│ 🔖 Bookmarks        │
│ 🎯 Progress         │
│                     │
│ Chapter 1           │
│ Chapter 2           │
│ ▼ Chapter 3         │
│   • Section 1       │
│   • Section 2       │
└─────────────────────┘
```

### Bottom Navigation Bar
```
┌─────────────────────┐
│                     │
│   [Content]         │
│                     │
├─────────────────────┤
│ 📖  🔍  🔖  ⚙️     │ ← Fixed bottom
└─────────────────────┘
```

## Keyboard Shortcuts

```
Navigation:
- ← / → : Previous/Next chapter
- ↑ / ↓ : Scroll up/down
- Home  : Jump to top
- End   : Jump to bottom

Actions:
- Cmd+K : Open search
- Cmd+B : Toggle sidebar
- Cmd+D : Bookmark page
- Esc   : Close modals
```

## Quality Standards

### Usability
- Current location always clear
- Navigation always accessible
- Actions predictable and consistent
- Feedback immediate
- Errors prevented

### Performance
- Navigation renders instantly
- Smooth animations
- No layout shifts
- Fast search results
- Responsive interactions

### Accessibility
- Keyboard navigable
- Screen reader friendly
- Focus indicators visible
- Skip links provided
- ARIA labels present

### Mobile Experience
- Touch targets ≥ 44px
- Swipe gestures supported
- Thumb-friendly placement
- Fast tap response
- Minimal navigation chrome

## Integration Points
- Implements book structure from specs
- Supports learning path progression
- Enables progress tracking
- Facilitates content discovery
- Connects to capstone projects
