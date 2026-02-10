import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

const FontSizeController: React.FC = () => {
  const [fontSize, setFontSize] = useState(100);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const savedSize = localStorage.getItem('fontSize');
    if (savedSize) {
      const size = parseInt(savedSize);
      setFontSize(size);
      applyFontSize(size);
    }
  }, []);

  const applyFontSize = (size: number) => {
    document.documentElement.style.fontSize = `${size}%`;
    localStorage.setItem('fontSize', size.toString());
  };

  const increaseFontSize = () => {
    const newSize = Math.min(fontSize + 10, 150);
    setFontSize(newSize);
    applyFontSize(newSize);
  };

  const decreaseFontSize = () => {
    const newSize = Math.max(fontSize - 10, 80);
    setFontSize(newSize);
    applyFontSize(newSize);
  };

  const resetFontSize = () => {
    setFontSize(100);
    applyFontSize(100);
  };

  useEffect(() => {
    const handleKeyPress = (e: KeyboardEvent) => {
      if (e.ctrlKey || e.metaKey) {
        if (e.key === '+' || e.key === '=') {
          e.preventDefault();
          increaseFontSize();
        } else if (e.key === '-') {
          e.preventDefault();
          decreaseFontSize();
        } else if (e.key === '0') {
          e.preventDefault();
          resetFontSize();
        }
      }
    };

    window.addEventListener('keydown', handleKeyPress);
    return () => window.removeEventListener('keydown', handleKeyPress);
  }, [fontSize]);

  return (
    <div className={styles.container}>
      <button
        className={styles.toggleButton}
        onClick={() => setIsVisible(!isVisible)}
        aria-label="Font size controls"
        title="Adjust font size"
      >
        <span className={styles.icon}>Aa</span>
      </button>

      {isVisible && (
        <div className={styles.panel}>
          <div className={styles.header}>
            <span className={styles.title}>Font Size</span>
            <span className={styles.percentage}>{fontSize}%</span>
          </div>

          <div className={styles.controls}>
            <button
              className={styles.button}
              onClick={decreaseFontSize}
              disabled={fontSize <= 80}
              aria-label="Decrease font size"
              title="Decrease (Ctrl+-)"
            >
              <span className={styles.buttonIcon}>A-</span>
            </button>

            <div className={styles.slider}>
              <input
                type="range"
                min="80"
                max="150"
                step="10"
                value={fontSize}
                onChange={(e) => {
                  const size = parseInt(e.target.value);
                  setFontSize(size);
                  applyFontSize(size);
                }}
                className={styles.range}
                aria-label="Font size slider"
              />
            </div>

            <button
              className={styles.button}
              onClick={increaseFontSize}
              disabled={fontSize >= 150}
              aria-label="Increase font size"
              title="Increase (Ctrl++)"
            >
              <span className={styles.buttonIcon}>A+</span>
            </button>
          </div>

          <button
            className={styles.resetButton}
            onClick={resetFontSize}
            aria-label="Reset font size"
          >
            Reset to Default
          </button>

          <div className={styles.hint}>
            Use <kbd>Ctrl</kbd> + <kbd>+</kbd> / <kbd>-</kbd> / <kbd>0</kbd>
          </div>
        </div>
      )}
    </div>
  );
};

export default FontSizeController;
