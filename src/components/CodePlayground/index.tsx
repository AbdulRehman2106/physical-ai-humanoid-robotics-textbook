import React, { useState } from 'react';
import CodeBlock from '@theme/CodeBlock';
import styles from './styles.module.css';

export interface CodePlaygroundProps {
  children: string;
  language: string;
  title?: string;
  showLineNumbers?: boolean;
}

const CodePlayground: React.FC<CodePlaygroundProps> = ({
  children,
  language,
  title,
  showLineNumbers = true,
}) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(children);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy code:', err);
    }
  };

  return (
    <div className={styles.codePlayground}>
      {title && (
        <div className={styles.codeHeader}>
          <h4 className={styles.codeTitle}>{title}</h4>
          <button
            className={`${styles.copyButton} ${copied ? styles.copied : ''}`}
            onClick={handleCopy}
            aria-label="Copy code to clipboard"
          >
            {copied ? '✓ Copied!' : 'Copy'}
          </button>
        </div>
      )}
      <div className={styles.codeContent}>
        <CodeBlock language={language} showLineNumbers={showLineNumbers}>
          {children}
        </CodeBlock>
      </div>
    </div>
  );
};

export default CodePlayground;
