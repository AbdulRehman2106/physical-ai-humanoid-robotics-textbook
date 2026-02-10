import React, { useEffect, useState } from 'react';
import styles from './styles.module.css';

const ScrollIndicator: React.FC = () => {
  const [isVisible, setIsVisible] = useState(true);

  useEffect(() => {
    const handleScroll = () => {
      if (window.scrollY > 100) {
        setIsVisible(false);
      } else {
        setIsVisible(true);
      }
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  if (!isVisible) return null;

  return (
    <div className={styles.scrollIndicator}>
      <div className={styles.mouse}>
        <div className={styles.wheel} />
      </div>
      <div className={styles.arrow}>↓</div>
    </div>
  );
};

export default ScrollIndicator;
