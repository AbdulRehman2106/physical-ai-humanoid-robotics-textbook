/**
 * Citation Component for Physical AI Textbook
 * Renders APA-style citations with proper formatting
 */

import React from 'react';
import styles from './styles.module.css';

export interface CitationProps {
  authors: string[];
  year: number;
  title: string;
  source?: string;
  volume?: string;
  issue?: string;
  pages?: string;
  doi?: string;
  url?: string;
  type?: 'article' | 'book' | 'conference' | 'website';
}

const Citation: React.FC<CitationProps> = ({
  authors,
  year,
  title,
  source,
  volume,
  issue,
  pages,
  doi,
  url,
  type = 'article',
}) => {
  const formatAuthors = () => {
    if (authors.length === 1) {
      return authors[0];
    } else if (authors.length === 2) {
      return `${authors[0]}, & ${authors[1]}`;
    } else {
      const lastAuthor = authors[authors.length - 1];
      const otherAuthors = authors.slice(0, -1).join(', ');
      return `${otherAuthors}, & ${lastAuthor}`;
    }
  };

  const renderCitation = () => {
    const authorText = formatAuthors();
    const yearText = `(${year})`;

    switch (type) {
      case 'article':
        return (
          <>
            {authorText} {yearText}. {title}. <em>{source}</em>
            {volume && `, ${volume}`}
            {issue && `(${issue})`}
            {pages && `, ${pages}`}.
            {doi && (
              <>
                {' '}
                <a href={`https://doi.org/${doi}`} target="_blank" rel="noopener noreferrer">
                  https://doi.org/{doi}
                </a>
              </>
            )}
          </>
        );

      case 'book':
        return (
          <>
            {authorText} {yearText}. <em>{title}</em>. {source}.
          </>
        );

      case 'conference':
        return (
          <>
            {authorText} {yearText}. {title}. In <em>{source}</em>
            {pages && ` (pp. ${pages})`}.
          </>
        );

      case 'website':
        return (
          <>
            {authorText} {yearText}. <em>{title}</em>. Retrieved from{' '}
            {url && (
              <a href={url} target="_blank" rel="noopener noreferrer">
                {url}
              </a>
            )}
          </>
        );

      default:
        return null;
    }
  };

  return (
    <div className={styles.citation} role="article" aria-label="Citation">
      {renderCitation()}
    </div>
  );
};

export default Citation;
