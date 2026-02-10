import React from 'react';
import styles from './styles.module.css';

interface BadgeProps {
  type: 'new' | 'updated' | 'beta' | 'advanced' | 'experimental';
  children?: React.ReactNode;
}

const Badge: React.FC<BadgeProps> = ({ type, children }) => {
  const icons = {
    new: '✨',
    updated: '🔄',
    beta: '🧪',
    advanced: '🚀',
    experimental: '⚡',
  };

  const labels = {
    new: 'New',
    updated: 'Updated',
    beta: 'Beta',
    advanced: 'Advanced',
    experimental: 'Experimental',
  };

  return (
    <span className={`${styles.badge} ${styles[`badge${type.charAt(0).toUpperCase() + type.slice(1)}`]}`}>
      <span className={styles.badgeIcon}>{icons[type]}</span>
      {children || labels[type]}
    </span>
  );
};

export default Badge;
