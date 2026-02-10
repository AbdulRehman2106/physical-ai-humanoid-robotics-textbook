import React, { useState } from 'react';
import styles from './styles.module.css';

interface AccordionItem {
  title: string;
  content: React.ReactNode;
  icon?: string;
}

interface AccordionProps {
  items: AccordionItem[];
  allowMultiple?: boolean;
}

const Accordion: React.FC<AccordionProps> = ({ items, allowMultiple = false }) => {
  const [openIndexes, setOpenIndexes] = useState<number[]>([]);

  const toggleItem = (index: number) => {
    if (allowMultiple) {
      setOpenIndexes(prev =>
        prev.includes(index)
          ? prev.filter(i => i !== index)
          : [...prev, index]
      );
    } else {
      setOpenIndexes(prev =>
        prev.includes(index) ? [] : [index]
      );
    }
  };

  return (
    <div className={styles.accordion}>
      {items.map((item, index) => (
        <div
          key={index}
          className={`${styles.item} ${openIndexes.includes(index) ? styles.open : ''}`}
        >
          <button
            className={styles.header}
            onClick={() => toggleItem(index)}
            aria-expanded={openIndexes.includes(index)}
          >
            {item.icon && <span className={styles.icon}>{item.icon}</span>}
            <span className={styles.title}>{item.title}</span>
            <span className={styles.arrow}>
              {openIndexes.includes(index) ? '▼' : '▶'}
            </span>
          </button>
          <div className={styles.content}>
            <div className={styles.contentInner}>
              {item.content}
            </div>
          </div>
        </div>
      ))}
    </div>
  );
};

export default Accordion;
