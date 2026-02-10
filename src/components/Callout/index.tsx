import React from 'react';
import styles from './styles.module.css';

export type CalloutType = 'info' | 'tip' | 'warning' | 'danger' | 'insight';

export interface CalloutProps {
  type?: CalloutType;
  title?: string;
  children: React.ReactNode;
}

const Callout: React.FC<CalloutProps> = ({ type = 'info', title, children }) => {
  const icons = {
    info: '💡',
    tip: '✨',
    warning: '⚠️',
    danger: '🚨',
    insight: '🔍',
  };

  const defaultTitles = {
    info: 'Information',
    tip: 'Tip',
    warning: 'Warning',
    danger: 'Danger',
    insight: 'Insight',
  };

  return (
    <div className={`${styles.callout} ${styles[type]}`} role="note" aria-label={title || defaultTitles[type]}>
      <div className={styles.calloutHeader}>
        <span className={styles.calloutIcon} aria-hidden="true">
          {icons[type]}
        </span>
        <strong className={styles.calloutTitle}>
          {title || defaultTitles[type]}
        </strong>
      </div>
      <div className={styles.calloutContent}>{children}</div>
    </div>
  );
};

export default Callout;
