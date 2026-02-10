import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface Bookmark {
  id: string;
  title: string;
  url: string;
  timestamp: number;
}

const BookmarkManager: React.FC = () => {
  const [bookmarks, setBookmarks] = useState<Bookmark[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [currentPageBookmarked, setCurrentPageBookmarked] = useState(false);

  useEffect(() => {
    const saved = localStorage.getItem('bookmarks');
    if (saved) {
      setBookmarks(JSON.parse(saved));
    }
    checkCurrentPage();
  }, []);

  useEffect(() => {
    checkCurrentPage();
  }, [bookmarks]);

  const checkCurrentPage = () => {
    const currentUrl = window.location.pathname;
    const isBookmarked = bookmarks.some(b => b.url === currentUrl);
    setCurrentPageBookmarked(isBookmarked);
  };

  const addBookmark = () => {
    const title = document.title;
    const url = window.location.pathname;

    const newBookmark: Bookmark = {
      id: Date.now().toString(),
      title,
      url,
      timestamp: Date.now(),
    };

    const updated = [...bookmarks, newBookmark];
    setBookmarks(updated);
    localStorage.setItem('bookmarks', JSON.stringify(updated));
    setCurrentPageBookmarked(true);
  };

  const removeBookmark = (id: string) => {
    const updated = bookmarks.filter(b => b.id !== id);
    setBookmarks(updated);
    localStorage.setItem('bookmarks', JSON.stringify(updated));
  };

  const removeCurrentPage = () => {
    const currentUrl = window.location.pathname;
    const updated = bookmarks.filter(b => b.url !== currentUrl);
    setBookmarks(updated);
    localStorage.setItem('bookmarks', JSON.stringify(updated));
    setCurrentPageBookmarked(false);
  };

  const clearAll = () => {
    if (confirm('Clear all bookmarks?')) {
      setBookmarks([]);
      localStorage.removeItem('bookmarks');
      setCurrentPageBookmarked(false);
    }
  };

  useEffect(() => {
    const handleKeyPress = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 's') {
        e.preventDefault();
        if (currentPageBookmarked) {
          removeCurrentPage();
        } else {
          addBookmark();
        }
      }
    };

    window.addEventListener('keydown', handleKeyPress);
    return () => window.removeEventListener('keydown', handleKeyPress);
  }, [currentPageBookmarked, bookmarks]);

  return (
    <div className={styles.container}>
      <button
        className={`${styles.toggleButton} ${currentPageBookmarked ? styles.bookmarked : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Bookmarks"
        title={`Bookmarks (${bookmarks.length})`}
      >
        <span className={styles.icon}>
          {currentPageBookmarked ? '★' : '☆'}
        </span>
        {bookmarks.length > 0 && (
          <span className={styles.badge}>{bookmarks.length}</span>
        )}
      </button>

      {isOpen && (
        <div className={styles.panel}>
          <div className={styles.header}>
            <h3 className={styles.title}>📚 Bookmarks</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              ×
            </button>
          </div>

          <div className={styles.actions}>
            <button
              className={styles.actionButton}
              onClick={currentPageBookmarked ? removeCurrentPage : addBookmark}
            >
              {currentPageBookmarked ? '− Remove This Page' : '+ Bookmark This Page'}
            </button>
          </div>

          {bookmarks.length === 0 ? (
            <div className={styles.empty}>
              <div className={styles.emptyIcon}>📖</div>
              <p className={styles.emptyText}>No bookmarks yet</p>
              <p className={styles.emptyHint}>
                Press <kbd>Ctrl+S</kbd> to bookmark pages
              </p>
            </div>
          ) : (
            <>
              <div className={styles.bookmarks}>
                {bookmarks.map((bookmark) => (
                  <div key={bookmark.id} className={styles.bookmark}>
                    <a
                      href={bookmark.url}
                      className={styles.bookmarkLink}
                      onClick={() => setIsOpen(false)}
                    >
                      <span className={styles.bookmarkIcon}>📄</span>
                      <span className={styles.bookmarkTitle}>{bookmark.title}</span>
                    </a>
                    <button
                      className={styles.deleteButton}
                      onClick={() => removeBookmark(bookmark.id)}
                      aria-label="Remove bookmark"
                    >
                      ×
                    </button>
                  </div>
                ))}
              </div>

              <div className={styles.footer}>
                <button
                  className={styles.clearButton}
                  onClick={clearAll}
                >
                  Clear All
                </button>
              </div>
            </>
          )}

          <div className={styles.hint}>
            <kbd>Ctrl</kbd> + <kbd>S</kbd> to bookmark current page
          </div>
        </div>
      )}
    </div>
  );
};

export default BookmarkManager;
