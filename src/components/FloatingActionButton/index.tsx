import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

const FloatingActionButton: React.FC = () => {
  const [isVisible, setIsVisible] = useState(false);
  const [showMenu, setShowMenu] = useState(false);

  useEffect(() => {
    const toggleVisibility = () => {
      if (window.scrollY > 300) {
        setIsVisible(true);
      } else {
        setIsVisible(false);
        setShowMenu(false);
      }
    };

    window.addEventListener('scroll', toggleVisibility);
    return () => window.removeEventListener('scroll', toggleVisibility);
  }, []);

  const scrollToTop = () => {
    window.scrollTo({
      top: 0,
      behavior: 'smooth',
    });
    setShowMenu(false);
  };

  const shareContent = () => {
    if (navigator.share) {
      navigator.share({
        title: document.title,
        url: window.location.href,
      });
    } else {
      navigator.clipboard.writeText(window.location.href);
      alert('Link copied to clipboard!');
    }
    setShowMenu(false);
  };

  const printPage = () => {
    window.print();
    setShowMenu(false);
  };

  if (!isVisible) return null;

  return (
    <div className={styles.fabContainer}>
      {showMenu && (
        <div className={styles.fabMenu}>
          <button
            className={styles.fabMenuItem}
            onClick={scrollToTop}
            title="Scroll to top"
          >
            <span className={styles.fabIcon}>↑</span>
            <span className={styles.fabLabel}>Top</span>
          </button>
          <button
            className={styles.fabMenuItem}
            onClick={shareContent}
            title="Share"
          >
            <span className={styles.fabIcon}>🔗</span>
            <span className={styles.fabLabel}>Share</span>
          </button>
          <button
            className={styles.fabMenuItem}
            onClick={printPage}
            title="Print"
          >
            <span className={styles.fabIcon}>🖨️</span>
            <span className={styles.fabLabel}>Print</span>
          </button>
        </div>
      )}

      <button
        className={`${styles.fab} ${showMenu ? styles.fabActive : ''}`}
        onClick={() => setShowMenu(!showMenu)}
        aria-label="Quick actions"
      >
        {showMenu ? '✕' : '⚡'}
      </button>
    </div>
  );
};

export default FloatingActionButton;
