import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface Note {
  id: string;
  pageUrl: string;
  pageTitle: string;
  content: string;
  timestamp: number;
  color: string;
}

const NotesPanel: React.FC = () => {
  const [notes, setNotes] = useState<Note[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [currentNote, setCurrentNote] = useState('');
  const [selectedColor, setSelectedColor] = useState('#ffd700');

  const colors = ['#ffd700', '#ff6b6b', '#4ecdc4', '#95e1d3', '#a8e6cf', '#dda15e'];

  useEffect(() => {
    const saved = localStorage.getItem('notes');
    if (saved) {
      setNotes(JSON.parse(saved));
    }
  }, []);

  const saveNote = () => {
    if (!currentNote.trim()) return;

    const newNote: Note = {
      id: Date.now().toString(),
      pageUrl: window.location.pathname,
      pageTitle: document.title,
      content: currentNote,
      timestamp: Date.now(),
      color: selectedColor,
    };

    const updated = [newNote, ...notes];
    setNotes(updated);
    localStorage.setItem('notes', JSON.stringify(updated));
    setCurrentNote('');
  };

  const deleteNote = (id: string) => {
    const updated = notes.filter(n => n.id !== id);
    setNotes(updated);
    localStorage.setItem('notes', JSON.stringify(updated));
  };

  const clearAll = () => {
    if (confirm('Clear all notes?')) {
      setNotes([]);
      localStorage.removeItem('notes');
    }
  };

  const currentPageNotes = notes.filter(n => n.pageUrl === window.location.pathname);
  const otherNotes = notes.filter(n => n.pageUrl !== window.location.pathname);

  return (
    <div className={styles.container}>
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Notes"
        title={`Notes (${notes.length})`}
      >
        <span className={styles.icon}>📝</span>
        {notes.length > 0 && (
          <span className={styles.badge}>{notes.length}</span>
        )}
      </button>

      {isOpen && (
        <div className={styles.panel}>
          <div className={styles.header}>
            <h3 className={styles.title}>📝 My Notes</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              ×
            </button>
          </div>

          <div className={styles.editor}>
            <textarea
              className={styles.textarea}
              placeholder="Write a note for this page..."
              value={currentNote}
              onChange={(e) => setCurrentNote(e.target.value)}
              rows={3}
            />

            <div className={styles.editorFooter}>
              <div className={styles.colorPicker}>
                {colors.map(color => (
                  <button
                    key={color}
                    className={`${styles.colorButton} ${selectedColor === color ? styles.colorButtonActive : ''}`}
                    style={{ background: color }}
                    onClick={() => setSelectedColor(color)}
                    aria-label={`Select color ${color}`}
                  />
                ))}
              </div>

              <button
                className={styles.saveButton}
                onClick={saveNote}
                disabled={!currentNote.trim()}
              >
                Save Note
              </button>
            </div>
          </div>

          <div className={styles.notes}>
            {currentPageNotes.length > 0 && (
              <div className={styles.section}>
                <h4 className={styles.sectionTitle}>📍 This Page</h4>
                {currentPageNotes.map(note => (
                  <div
                    key={note.id}
                    className={styles.note}
                    style={{ borderLeftColor: note.color }}
                  >
                    <div className={styles.noteContent}>{note.content}</div>
                    <div className={styles.noteFooter}>
                      <span className={styles.noteTime}>
                        {new Date(note.timestamp).toLocaleDateString()}
                      </span>
                      <button
                        className={styles.deleteButton}
                        onClick={() => deleteNote(note.id)}
                        aria-label="Delete note"
                      >
                        🗑️
                      </button>
                    </div>
                  </div>
                ))}
              </div>
            )}

            {otherNotes.length > 0 && (
              <div className={styles.section}>
                <h4 className={styles.sectionTitle}>📚 Other Pages</h4>
                {otherNotes.map(note => (
                  <div
                    key={note.id}
                    className={styles.note}
                    style={{ borderLeftColor: note.color }}
                  >
                    <a href={note.pageUrl} className={styles.noteLink}>
                      {note.pageTitle}
                    </a>
                    <div className={styles.noteContent}>{note.content}</div>
                    <div className={styles.noteFooter}>
                      <span className={styles.noteTime}>
                        {new Date(note.timestamp).toLocaleDateString()}
                      </span>
                      <button
                        className={styles.deleteButton}
                        onClick={() => deleteNote(note.id)}
                        aria-label="Delete note"
                      >
                        🗑️
                      </button>
                    </div>
                  </div>
                ))}
              </div>
            )}

            {notes.length === 0 && (
              <div className={styles.empty}>
                <div className={styles.emptyIcon}>📝</div>
                <p className={styles.emptyText}>No notes yet</p>
                <p className={styles.emptyHint}>
                  Start taking notes while you learn!
                </p>
              </div>
            )}
          </div>

          {notes.length > 0 && (
            <div className={styles.footer}>
              <button className={styles.clearButton} onClick={clearAll}>
                Clear All Notes
              </button>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default NotesPanel;
