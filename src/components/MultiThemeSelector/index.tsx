import React, { useState, useEffect } from 'react';
import { themes, applyTheme, getThemeById } from '@site/src/utils/themes';
import styles from './styles.module.css';
import BrowserOnly from '@docusaurus/BrowserOnly';

function MultiThemeSelectorComponent() {
  const [isOpen, setIsOpen] = useState(false);
  const [currentTheme, setCurrentTheme] = useState('purple');

  useEffect(() => {
    // Load saved theme from localStorage
    const savedTheme = localStorage.getItem('selected-theme') || 'purple';
    setCurrentTheme(savedTheme);
    applyTheme(getThemeById(savedTheme));
  }, []);

  const handleThemeChange = (themeId: string) => {
    setCurrentTheme(themeId);
    localStorage.setItem('selected-theme', themeId);
    applyTheme(getThemeById(themeId));
    setIsOpen(false);
  };

  const selectedTheme = getThemeById(currentTheme);

  return (
    <>
      <button
        className={styles.themeButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Select theme"
        title="Select theme"
      >
        {selectedTheme.icon}
      </button>

      {isOpen && (
        <>
          <div className={styles.backdrop} onClick={() => setIsOpen(false)} />
          <div className={styles.dropdown}>
            <div className={styles.header}>
              <h3>🎨 Choose Theme</h3>
              <button className={styles.close} onClick={() => setIsOpen(false)}>
                ✕
              </button>
            </div>
            <div className={styles.grid}>
              {themes.map((theme) => (
                <button
                  key={theme.id}
                  className={`${styles.themeCard} ${
                    currentTheme === theme.id ? styles.active : ''
                  }`}
                  onClick={() => handleThemeChange(theme.id)}
                >
                  <div className={styles.themeIcon}>{theme.icon}</div>
                  <div className={styles.themeName}>{theme.name}</div>
                  {currentTheme === theme.id && (
                    <div className={styles.check}>✓</div>
                  )}
                </button>
              ))}
            </div>
          </div>
        </>
      )}
    </>
  );
}

export default function MultiThemeSelector() {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <MultiThemeSelectorComponent />}
    </BrowserOnly>
  );
}
