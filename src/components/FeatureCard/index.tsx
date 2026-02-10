import React from 'react';
import styles from './styles.module.css';

interface FeatureCardProps {
  icon: string;
  title: string;
  description: string;
  gradient?: string;
  delay?: number;
}

const FeatureCard: React.FC<FeatureCardProps> = ({
  icon,
  title,
  description,
  gradient = 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
  delay = 0,
}) => {
  return (
    <div
      className={styles.card}
      style={{ animationDelay: `${delay}s` }}
    >
      <div
        className={styles.iconWrapper}
        style={{ background: gradient }}
      >
        <span className={styles.icon}>{icon}</span>
      </div>
      <h3 className={styles.title}>{title}</h3>
      <p className={styles.description}>{description}</p>
      <div className={styles.shine} />
    </div>
  );
};

export default FeatureCard;
