import React from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import styles from './styles.module.css';
import BrowserOnly from '@docusaurus/BrowserOnly';

function ThemeToggleComponent() {
  const { colorMode, setColorMode } = useColorMode();

  const toggleTheme = () => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  };

  return (
    <button
      className={styles.themeToggle}
      onClick={toggleTheme}
      aria-label={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
      title={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
    >
      {colorMode === 'dark' ? '☀️' : '🌙'}
    </button>
  );
}

export default function ThemeToggle() {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <ThemeToggleComponent />}
    </BrowserOnly>
  );
}
