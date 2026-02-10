import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface KeyboardShortcut {
  keys: string[];
  description: string;
  action?: () => void;
}

const KeyboardShortcutsPanel: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);

  const shortcuts: KeyboardShortcut[] = [
    { keys: ['Ctrl', 'K'], description: 'Open search' },
    { keys: ['Ctrl', '/'], description: 'Show keyboard shortcuts' },
    { keys: ['Ctrl', 'B'], description: 'Toggle sidebar' },
    { keys: ['Ctrl', 'D'], description: 'Toggle dark mode' },
    { keys: ['Ctrl', '+'], description: 'Increase font size' },
    { keys: ['Ctrl', '-'], description: 'Decrease font size' },
    { keys: ['Ctrl', '0'], description: 'Reset font size' },
    { keys: ['Ctrl', 'P'], description: 'Print page' },
    { keys: ['Ctrl', 'S'], description: 'Bookmark page' },
    { keys: ['↑', '↓'], description: 'Scroll page' },
    { keys: ['Home'], description: 'Go to top' },
    { keys: ['End'], description: 'Go to bottom' },
    { keys: ['Esc'], description: 'Close modals' },
  ];

  useEffect(() => {
    const handleKeyPress = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key === '/') {
        e.preventDefault();
        setIsOpen(!isOpen);
      }
      if (e.key === 'Escape') {
        setIsOpen(false);
      }
    };

    window.addEventListener('keydown', handleKeyPress);
    return () => window.removeEventListener('keydown', handleKeyPress);
  }, [isOpen]);

  if (!isOpen) {
    return (
      <button
        className={styles.triggerButton}
        onClick={() => setIsOpen(true)}
        aria-label="Show keyboard shortcuts"
        title="Keyboard shortcuts (Ctrl+/)"
      >
        ⌨️
      </button>
    );
  }

  return (
    <div className={styles.overlay} onClick={() => setIsOpen(false)}>
      <div className={styles.panel} onClick={(e) => e.stopPropagation()}>
        <div className={styles.header}>
          <h3 className={styles.title}>⌨️ Keyboard Shortcuts</h3>
          <button
            className={styles.closeButton}
            onClick={() => setIsOpen(false)}
            aria-label="Close"
          >
            ×
          </button>
        </div>

        <div className={styles.shortcuts}>
          {shortcuts.map((shortcut, index) => (
            <div key={index} className={styles.shortcut}>
              <div className={styles.keys}>
                {shortcut.keys.map((key, i) => (
                  <React.Fragment key={i}>
                    <kbd className={styles.key}>{key}</kbd>
                    {i < shortcut.keys.length - 1 && (
                      <span className={styles.plus}>+</span>
                    )}
                  </React.Fragment>
                ))}
              </div>
              <div className={styles.description}>{shortcut.description}</div>
            </div>
          ))}
        </div>

        <div className={styles.footer}>
          <p className={styles.hint}>
            Press <kbd className={styles.key}>Ctrl</kbd> +{' '}
            <kbd className={styles.key}>/</kbd> to toggle this panel
          </p>
        </div>
      </div>
    </div>
  );
};

export default KeyboardShortcutsPanel;
