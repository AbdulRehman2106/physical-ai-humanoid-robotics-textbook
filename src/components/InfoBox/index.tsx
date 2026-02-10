import React from 'react';
import styles from './styles.module.css';

interface InfoBoxProps {
  type?: 'info' | 'success' | 'warning' | 'danger' | 'tip';
  title?: string;
  icon?: string;
  children: React.ReactNode;
}

const InfoBox: React.FC<InfoBoxProps> = ({
  type = 'info',
  title,
  icon,
  children,
}) => {
  const defaultIcons = {
    info: 'ℹ️',
    success: '✅',
    warning: '⚠️',
    danger: '🚨',
    tip: '💡',
  };

  const defaultTitles = {
    info: 'Information',
    success: 'Success',
    warning: 'Warning',
    danger: 'Danger',
    tip: 'Tip',
  };

  return (
    <div className={`${styles.container} ${styles[type]}`}>
      <div className={styles.header}>
        <div className={styles.icon}>
          {icon || defaultIcons[type]}
        </div>
        <h3 className={styles.title}>
          {title || defaultTitles[type]}
        </h3>
      </div>
      <div className={styles.content}>
        {children}
      </div>
    </div>
  );
};

export default InfoBox;
