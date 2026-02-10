import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

export interface CheckpointItem {
  id: string;
  text: string;
}

export interface CheckpointProps {
  title?: string;
  items: CheckpointItem[];
  storageKey?: string;
}

const Checkpoint: React.FC<CheckpointProps> = ({
  title = 'Learning Checkpoint',
  items,
  storageKey,
}) => {
  const [completed, setCompleted] = useState<Set<string>>(new Set());

  // Load saved progress from localStorage
  useEffect(() => {
    if (storageKey && typeof window !== 'undefined') {
      const saved = localStorage.getItem(storageKey);
      if (saved) {
        try {
          const savedIds = JSON.parse(saved);
          setCompleted(new Set(savedIds));
        } catch (e) {
          console.error('Failed to load checkpoint progress:', e);
        }
      }
    }
  }, [storageKey]);

  // Save progress to localStorage
  useEffect(() => {
    if (storageKey && typeof window !== 'undefined') {
      localStorage.setItem(storageKey, JSON.stringify(Array.from(completed)));
    }
  }, [completed, storageKey]);

  const handleToggle = (itemId: string) => {
    setCompleted((prev) => {
      const newSet = new Set(prev);
      if (newSet.has(itemId)) {
        newSet.delete(itemId);
      } else {
        newSet.add(itemId);
      }
      return newSet;
    });
  };

  const progress = (completed.size / items.length) * 100;
  const isComplete = completed.size === items.length;

  return (
    <div className={styles.checkpoint} role="region" aria-label={title}>
      <div className={styles.checkpointHeader}>
        <span className={styles.checkpointIcon} aria-hidden="true">
          🎯
        </span>
        <h4 className={styles.checkpointTitle}>{title}</h4>
      </div>

      <div className={styles.checkpointContent}>
        <ul className={styles.checkpointList}>
          {items.map((item) => {
            const isCompleted = completed.has(item.id);
            return (
              <li key={item.id} className={styles.checkpointItem}>
                <input
                  type="checkbox"
                  id={`checkpoint-${item.id}`}
                  className={styles.checkbox}
                  checked={isCompleted}
                  onChange={() => handleToggle(item.id)}
                  aria-label={`Mark "${item.text}" as ${isCompleted ? 'incomplete' : 'complete'}`}
                />
                <label
                  htmlFor={`checkpoint-${item.id}`}
                  className={`${styles.itemText} ${isCompleted ? styles.itemCompleted : ''}`}
                >
                  {item.text}
                </label>
              </li>
            );
          })}
        </ul>
      </div>

      <div className={styles.progress}>
        <div className={styles.progressBar} role="progressbar" aria-valuenow={progress} aria-valuemin={0} aria-valuemax={100}>
          <div className={styles.progressFill} style={{ width: `${progress}%` }} />
        </div>
        <div className={styles.progressText}>
          {completed.size} of {items.length} completed ({Math.round(progress)}%)
        </div>
      </div>

      {isComplete && (
        <div className={styles.completionMessage} role="status" aria-live="polite">
          🎉 Checkpoint complete! You've mastered this section.
        </div>
      )}
    </div>
  );
};

export default Checkpoint;
