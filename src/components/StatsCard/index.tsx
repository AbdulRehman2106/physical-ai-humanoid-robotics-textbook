import React from 'react';
import styles from './styles.module.css';

interface StatsCardProps {
  value: string | number;
  label: string;
  icon?: string;
  trend?: 'up' | 'down' | 'neutral';
  trendValue?: string;
  gradient?: string;
}

const StatsCard: React.FC<StatsCardProps> = ({
  value,
  label,
  icon,
  trend,
  trendValue,
  gradient = 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
}) => {
  const trendIcons = {
    up: '↗',
    down: '↘',
    neutral: '→',
  };

  return (
    <div className={styles.card}>
      <div className={styles.iconWrapper} style={{ background: gradient }}>
        {icon && <span className={styles.icon}>{icon}</span>}
      </div>
      <div className={styles.content}>
        <div className={styles.value}>{value}</div>
        <div className={styles.label}>{label}</div>
        {trend && trendValue && (
          <div className={`${styles.trend} ${styles[trend]}`}>
            <span className={styles.trendIcon}>{trendIcons[trend]}</span>
            <span className={styles.trendValue}>{trendValue}</span>
          </div>
        )}
      </div>
    </div>
  );
};

export default StatsCard;
