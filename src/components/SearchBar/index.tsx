import React, { useState, useEffect, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

interface SearchResult {
  title: string;
  url: string;
  excerpt: string;
}

const SearchBar: React.FC = () => {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [selectedIndex, setSelectedIndex] = useState(0);
  const searchRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const history = useHistory();

  // Mock search - in production, integrate with Docusaurus search
  const mockSearch = (searchQuery: string): SearchResult[] => {
    if (!searchQuery) return [];

    const mockData: SearchResult[] = [
      { title: 'Introduction to Physical AI', url: '/docs/intro', excerpt: 'Learn the fundamentals of Physical AI and embodied intelligence...' },
      { title: 'ROS 2 Fundamentals', url: '/docs/chapters/ros2-fundamentals', excerpt: 'Master ROS 2 architecture, nodes, topics, and services...' },
      { title: 'Gazebo Simulation', url: '/docs/chapters/gazebo-basics', excerpt: 'Build and simulate robots in Gazebo environment...' },
      { title: 'Vision-Language-Action Models', url: '/docs/chapters/vla-models', excerpt: 'Explore cutting-edge VLA models for robotics...' },
    ];

    return mockData.filter(item =>
      item.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.excerpt.toLowerCase().includes(searchQuery.toLowerCase())
    );
  };

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (searchRef.current && !searchRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  useEffect(() => {
    const handleKeyPress = (e: KeyboardEvent) => {
      // Cmd+K or Ctrl+K to focus search
      if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
        e.preventDefault();
        inputRef.current?.focus();
        setIsOpen(true);
      }
    };

    document.addEventListener('keydown', handleKeyPress);
    return () => document.removeEventListener('keydown', handleKeyPress);
  }, []);

  const handleSearch = (value: string) => {
    setQuery(value);
    const searchResults = mockSearch(value);
    setResults(searchResults);
    setIsOpen(searchResults.length > 0);
    setSelectedIndex(0);
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (!isOpen) return;

    switch (e.key) {
      case 'ArrowDown':
        e.preventDefault();
        setSelectedIndex(prev => (prev + 1) % results.length);
        break;
      case 'ArrowUp':
        e.preventDefault();
        setSelectedIndex(prev => (prev - 1 + results.length) % results.length);
        break;
      case 'Enter':
        e.preventDefault();
        if (results[selectedIndex]) {
          history.push(results[selectedIndex].url);
          setIsOpen(false);
          setQuery('');
        }
        break;
      case 'Escape':
        setIsOpen(false);
        break;
    }
  };

  const handleResultClick = (url: string) => {
    history.push(url);
    setIsOpen(false);
    setQuery('');
  };

  return (
    <div className={styles.searchContainer} ref={searchRef}>
      <div className={styles.searchInputWrapper}>
        <svg className={styles.searchIcon} width="20" height="20" viewBox="0 0 20 20" fill="none">
          <path d="M9 17A8 8 0 1 0 9 1a8 8 0 0 0 0 16zM19 19l-4.35-4.35" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
        <input
          ref={inputRef}
          type="text"
          placeholder="Search documentation... (⌘K)"
          value={query}
          onChange={(e) => handleSearch(e.target.value)}
          onKeyDown={handleKeyDown}
          onFocus={() => query && setIsOpen(true)}
          className={styles.searchInput}
        />
        {query && (
          <button
            className={styles.clearButton}
            onClick={() => {
              setQuery('');
              setResults([]);
              setIsOpen(false);
            }}
          >
            ✕
          </button>
        )}
      </div>

      {isOpen && results.length > 0 && (
        <div className={styles.searchResults}>
          {results.map((result, index) => (
            <div
              key={result.url}
              className={`${styles.searchResult} ${index === selectedIndex ? styles.selected : ''}`}
              onClick={() => handleResultClick(result.url)}
              onMouseEnter={() => setSelectedIndex(index)}
            >
              <div className={styles.resultTitle}>{result.title}</div>
              <div className={styles.resultExcerpt}>{result.excerpt}</div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default SearchBar;
