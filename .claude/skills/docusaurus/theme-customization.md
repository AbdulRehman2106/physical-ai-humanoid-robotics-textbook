# Skill: Docusaurus Theme Customization

## Purpose
Customize Docusaurus themes to create a premium, book-like aesthetic that aligns with brand identity and enhances the learning experience.

## Responsibility
Design and specify theme customizations including colors, typography, spacing, and component styling to create a cohesive visual identity.

## When to Use
- Setting up a new Docusaurus site
- Customizing visual appearance
- Creating brand-aligned design
- Implementing dark mode
- Optimizing for book-like reading

## Core Capabilities

### 1. Theme Token Configuration
- Define color palettes (light and dark)
- Establish typography scale
- Set spacing system
- Configure border radius and shadows
- Define animation timing

### 2. Component Styling
- Customize navbar appearance
- Style sidebar navigation
- Design footer layout
- Customize admonitions
- Style code blocks

### 3. Dark Mode Design
- Create dark color palette
- Ensure proper contrast
- Design smooth transitions
- Handle images and media
- Test readability

### 4. Typography System
- Select font families
- Define font sizes and weights
- Set line heights
- Configure font loading
- Optimize for readability

### 5. Custom CSS
- Write custom styles
- Override default components
- Create utility classes
- Implement responsive design
- Optimize performance

## Theme Configuration

### docusaurus.config.js Theme Settings

```javascript
module.exports = {
  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo-dark.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/your-repo',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Chapters',
              to: '/docs/chapters',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book`,
    },
    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
      additionalLanguages: ['python', 'cpp', 'bash'],
    },
  },
};
```

### Custom CSS (src/css/custom.css)

```css
/**
 * Custom Theme for Physical AI Book
 * Premium, book-like aesthetic
 */

:root {
  /* Primary Colors */
  --ifm-color-primary: #3B82F6;
  --ifm-color-primary-dark: #2563EB;
  --ifm-color-primary-darker: #1D4ED8;
  --ifm-color-primary-darkest: #1E40AF;
  --ifm-color-primary-light: #60A5FA;
  --ifm-color-primary-lighter: #93C5FD;
  --ifm-color-primary-lightest: #DBEAFE;

  /* Background Colors */
  --ifm-background-color: #FFFFFF;
  --ifm-background-surface-color: #F8F9FA;

  /* Text Colors */
  --ifm-font-color-base: #1A1A1A;
  --ifm-font-color-secondary: #6B7280;

  /* Typography */
  --ifm-font-family-base: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
  --ifm-font-family-monospace: 'JetBrains Mono', 'Fira Code', monospace;

  --ifm-font-size-base: 18px;
  --ifm-line-height-base: 1.7;

  --ifm-heading-font-weight: 700;
  --ifm-font-weight-semibold: 600;
  --ifm-font-weight-bold: 700;

  /* Spacing */
  --ifm-spacing-horizontal: 1.5rem;
  --ifm-spacing-vertical: 1.5rem;

  /* Layout */
  --ifm-container-width: 1400px;
  --ifm-container-width-xl: 1600px;
  --doc-sidebar-width: 300px;
  --ifm-toc-width: 250px;

  /* Borders and Shadows */
  --ifm-global-radius: 8px;
  --ifm-code-border-radius: 6px;
  --ifm-button-border-radius: 6px;

  --ifm-global-shadow-lw: 0 1px 3px 0 rgba(0, 0, 0, 0.1);
  --ifm-global-shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
  --ifm-global-shadow-tl: 0 10px 15px -3px rgba(0, 0, 0, 0.1);

  /* Code Blocks */
  --ifm-code-background: #F3F4F6;
  --ifm-code-font-size: 0.95rem;
  --ifm-code-padding-vertical: 0.1rem;
  --ifm-code-padding-horizontal: 0.3rem;
}

/* Dark Mode */
[data-theme='dark'] {
  --ifm-background-color: #0F1419;
  --ifm-background-surface-color: #1A1F2E;

  --ifm-font-color-base: #E5E7EB;
  --ifm-font-color-secondary: #9CA3AF;

  --ifm-color-primary: #60A5FA;
  --ifm-color-primary-dark: #3B82F6;
  --ifm-color-primary-darker: #2563EB;
  --ifm-color-primary-darkest: #1D4ED8;
  --ifm-color-primary-light: #93C5FD;
  --ifm-color-primary-lighter: #BFDBFE;
  --ifm-color-primary-lightest: #DBEAFE;

  --ifm-code-background: #1E293B;
}

/* Typography Enhancements */
.markdown h1 {
  font-size: 2.5rem;
  font-weight: 700;
  line-height: 1.2;
  margin-bottom: 1.5rem;
  letter-spacing: -0.02em;
}

.markdown h2 {
  font-size: 2rem;
  font-weight: 700;
  line-height: 1.3;
  margin-top: 3rem;
  margin-bottom: 1rem;
  letter-spacing: -0.01em;
}

.markdown h3 {
  font-size: 1.5rem;
  font-weight: 600;
  line-height: 1.4;
  margin-top: 2rem;
  margin-bottom: 0.75rem;
}

.markdown p {
  margin-bottom: 1.5rem;
  line-height: 1.7;
}

/* Code Block Styling */
.prism-code {
  font-size: 0.95rem;
  line-height: 1.6;
  border-radius: var(--ifm-code-border-radius);
  box-shadow: var(--ifm-global-shadow-md);
}

/* Inline Code */
code {
  background-color: var(--ifm-code-background);
  border: 1px solid rgba(0, 0, 0, 0.1);
  border-radius: 4px;
  padding: 0.1rem 0.3rem;
  font-size: 0.9em;
}

[data-theme='dark'] code {
  border-color: rgba(255, 255, 255, 0.1);
}

/* Admonition Styling */
.admonition {
  border-radius: var(--ifm-global-radius);
  box-shadow: var(--ifm-global-shadow-lw);
  margin-bottom: 1.5rem;
}

.admonition-heading {
  font-weight: 600;
  display: flex;
  align-items: center;
}

/* Sidebar Styling */
.theme-doc-sidebar-container {
  border-right: 1px solid var(--ifm-toc-border-color);
  background-color: var(--ifm-background-surface-color);
}

.menu__link {
  border-radius: 6px;
  transition: all 0.2s ease;
}

.menu__link:hover {
  background-color: var(--ifm-menu-color-background-hover);
  transform: translateX(2px);
}

.menu__link--active {
  font-weight: 600;
  background-color: var(--ifm-menu-color-background-active);
}

/* Table of Contents */
.table-of-contents {
  font-size: 0.95rem;
}

.table-of-contents__link {
  color: var(--ifm-font-color-secondary);
  transition: color 0.2s ease;
}

.table-of-contents__link:hover,
.table-of-contents__link--active {
  color: var(--ifm-color-primary);
  font-weight: 500;
}

/* Navbar */
.navbar {
  box-shadow: var(--ifm-global-shadow-lw);
  backdrop-filter: blur(10px);
  background-color: rgba(255, 255, 255, 0.95);
}

[data-theme='dark'] .navbar {
  background-color: rgba(15, 20, 25, 0.95);
}

/* Footer */
.footer {
  background-color: var(--ifm-background-surface-color);
  border-top: 1px solid var(--ifm-toc-border-color);
}

/* Smooth Scrolling */
html {
  scroll-behavior: smooth;
}

/* Reading Width Optimization */
.markdown > * {
  max-width: 800px;
}

.markdown > .full-width {
  max-width: 100%;
}

/* Focus Styles for Accessibility */
*:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
  border-radius: 4px;
}

/* Print Styles */
@media print {
  .navbar,
  .theme-doc-sidebar-container,
  .theme-doc-toc-desktop,
  .pagination-nav {
    display: none;
  }

  .markdown {
    max-width: 100%;
  }
}
```

## Component Customizations

### Custom Navbar Component

```jsx
// src/theme/Navbar/index.js
import React from 'react';
import Navbar from '@theme-original/Navbar';
import ProgressBar from '@site/src/components/ProgressBar';

export default function NavbarWrapper(props) {
  return (
    <>
      <ProgressBar />
      <Navbar {...props} />
    </>
  );
}
```

### Custom Footer

```jsx
// src/theme/Footer/index.js
import React from 'react';
import styles from './styles.module.css';

export default function Footer() {
  return (
    <footer className={styles.footer}>
      <div className={styles.container}>
        <div className={styles.section}>
          <h3>Physical AI Book</h3>
          <p>Learn robotics, ROS 2, and embodied intelligence</p>
        </div>
        <div className={styles.section}>
          <h4>Resources</h4>
          <ul>
            <li><a href="/docs/intro">Get Started</a></li>
            <li><a href="/docs/chapters">All Chapters</a></li>
            <li><a href="/docs/capstone">Capstone Project</a></li>
          </ul>
        </div>
        <div className={styles.section}>
          <h4>Community</h4>
          <ul>
            <li><a href="https://github.com">GitHub</a></li>
            <li><a href="https://discord.com">Discord</a></li>
            <li><a href="https://twitter.com">Twitter</a></li>
          </ul>
        </div>
      </div>
      <div className={styles.copyright}>
        © {new Date().getFullYear()} Physical AI Book. Built with Docusaurus.
      </div>
    </footer>
  );
}
```

## Quality Standards

### Visual Consistency
- All colors from defined palette
- Typography follows scale
- Spacing uses system values
- Components share visual language
- Transitions are smooth

### Readability
- Optimal line length (45-75 characters)
- Sufficient contrast ratios (WCAG AA)
- Comfortable font sizes
- Appropriate line heights
- Clear visual hierarchy

### Performance
- Minimal custom CSS
- Optimized font loading
- No layout shifts
- Fast theme switching
- Efficient animations

### Accessibility
- Keyboard navigable
- Screen reader friendly
- Focus indicators visible
- Color not sole indicator
- Semantic HTML structure

## Integration Points
- Implements book layout design
- Supports navigation patterns
- Enables motion design
- Creates premium aesthetic
- Maintains brand consistency
