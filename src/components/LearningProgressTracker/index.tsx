import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface ChapterProgress {
  [chapterId: string]: {
    completed: boolean;
    lastVisited: number;
    timeSpent: number;
  };
}

const LearningProgressTracker: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [progress, setProgress] = useState<ChapterProgress>({});
  const [currentChapter, setCurrentChapter] = useState<string>('');

  const chapters = [
    { id: '01', title: 'Physical AI Introduction', url: '/docs/chapters/01-physical-ai-intro/' },
    { id: '02', title: 'Embodied Intelligence', url: '/docs/chapters/02-embodied-intelligence/' },
    { id: '03', title: 'ROS 2 Fundamentals', url: '/docs/chapters/03-ros2-fundamentals/' },
    { id: '04', title: 'ROS 2 Communication', url: '/docs/chapters/04-ros2-communication/' },
    { id: '05', title: 'Simulation Introduction', url: '/docs/chapters/05-simulation-intro/' },
    { id: '06', title: 'Gazebo Basics', url: '/docs/chapters/06-gazebo-basics/' },
    { id: '07', title: 'NVIDIA Isaac Sim', url: '/docs/chapters/07-isaac-sim/' },
    { id: '08', title: 'VLA Models', url: '/docs/chapters/08-vla-models/' },
    { id: '09', title: 'Sim-to-Real Transfer', url: '/docs/chapters/09-sim-to-real/' },
    { id: '10', title: 'Error Handling', url: '/docs/chapters/10-error-handling/' },
    { id: '11', title: 'Capstone Project', url: '/docs/chapters/11-capstone/' },
  ];

  useEffect(() => {
    const saved = localStorage.getItem('learningProgress');
    if (saved) {
      setProgress(JSON.parse(saved));
    }

    // Track current chapter
    const path = window.location.pathname;
    const chapter = chapters.find(c => path.includes(c.url));
    if (chapter) {
      setCurrentChapter(chapter.id);
      updateProgress(chapter.id);
    }
  }, []);

  const updateProgress = (chapterId: string) => {
    const updated = {
      ...progress,
      [chapterId]: {
        completed: progress[chapterId]?.completed || false,
        lastVisited: Date.now(),
        timeSpent: (progress[chapterId]?.timeSpent || 0) + 1,
      },
    };
    setProgress(updated);
    localStorage.setItem('learningProgress', JSON.stringify(updated));
  };

  const toggleComplete = (chapterId: string) => {
    const updated = {
      ...progress,
      [chapterId]: {
        ...progress[chapterId],
        completed: !progress[chapterId]?.completed,
        lastVisited: Date.now(),
        timeSpent: progress[chapterId]?.timeSpent || 0,
      },
    };
    setProgress(updated);
    localStorage.setItem('learningProgress', JSON.stringify(updated));
  };

  const resetProgress = () => {
    if (confirm('Reset all learning progress?')) {
      setProgress({});
      localStorage.removeItem('learningProgress');
    }
  };

  const completedCount = Object.values(progress).filter(p => p.completed).length;
  const progressPercentage = Math.round((completedCount / chapters.length) * 100);

  return (
    <div className={styles.container}>
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Learning Progress"
        title={`Progress: ${progressPercentage}%`}
      >
        <span className={styles.icon}>📈</span>
        <span className={styles.percentage}>{progressPercentage}%</span>
      </button>

      {isOpen && (
        <div className={styles.panel}>
          <div className={styles.header}>
            <h3 className={styles.title}>📈 Learning Progress</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              ×
            </button>
          </div>

          <div className={styles.stats}>
            <div className={styles.stat}>
              <div className={styles.statValue}>{completedCount}</div>
              <div className={styles.statLabel}>Completed</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statValue}>{chapters.length - completedCount}</div>
              <div className={styles.statLabel}>Remaining</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statValue}>{progressPercentage}%</div>
              <div className={styles.statLabel}>Progress</div>
            </div>
          </div>

          <div className={styles.progressBar}>
            <div
              className={styles.progressFill}
              style={{ width: `${progressPercentage}%` }}
            />
          </div>

          <div className={styles.chapters}>
            {chapters.map((chapter) => {
              const chapterProgress = progress[chapter.id];
              const isCompleted = chapterProgress?.completed || false;
              const isCurrent = currentChapter === chapter.id;

              return (
                <div
                  key={chapter.id}
                  className={`${styles.chapter} ${isCompleted ? styles.chapterCompleted : ''} ${isCurrent ? styles.chapterCurrent : ''}`}
                >
                  <button
                    className={styles.checkbox}
                    onClick={() => toggleComplete(chapter.id)}
                    aria-label={`Mark chapter ${chapter.id} as ${isCompleted ? 'incomplete' : 'complete'}`}
                  >
                    {isCompleted ? '✓' : '○'}
                  </button>

                  <a
                    href={chapter.url}
                    className={styles.chapterLink}
                    onClick={() => setIsOpen(false)}
                  >
                    <span className={styles.chapterId}>Ch {chapter.id}</span>
                    <span className={styles.chapterTitle}>{chapter.title}</span>
                  </a>

                  {isCurrent && (
                    <span className={styles.currentBadge}>Current</span>
                  )}
                </div>
              );
            })}
          </div>

          <div className={styles.footer}>
            <button className={styles.resetButton} onClick={resetProgress}>
              Reset Progress
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default LearningProgressTracker;
