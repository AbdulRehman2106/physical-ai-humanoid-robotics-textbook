import React, { useState } from 'react';
import styles from './styles.module.css';

const QuickFeedback: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [submitted, setSubmitted] = useState(false);
  const [rating, setRating] = useState(0);
  const [feedback, setFeedback] = useState('');

  const handleSubmit = () => {
    if (rating === 0) {
      alert('Please select a rating');
      return;
    }

    const feedbackData = {
      pageUrl: window.location.pathname,
      pageTitle: document.title,
      rating,
      feedback,
      timestamp: Date.now(),
    };

    // Save to localStorage
    const existingFeedback = JSON.parse(localStorage.getItem('pageFeedback') || '[]');
    existingFeedback.push(feedbackData);
    localStorage.setItem('pageFeedback', JSON.stringify(existingFeedback));

    setSubmitted(true);
    setTimeout(() => {
      setIsOpen(false);
      setSubmitted(false);
      setRating(0);
      setFeedback('');
    }, 2000);
  };

  return (
    <div className={styles.container}>
      {!isOpen ? (
        <button
          className={styles.trigger}
          onClick={() => setIsOpen(true)}
          aria-label="Give feedback"
          title="How was this page?"
        >
          💬
        </button>
      ) : (
        <div className={styles.panel}>
          {!submitted ? (
            <>
              <div className={styles.header}>
                <h4 className={styles.title}>How was this page?</h4>
                <button
                  className={styles.closeButton}
                  onClick={() => setIsOpen(false)}
                  aria-label="Close"
                >
                  ×
                </button>
              </div>

              <div className={styles.content}>
                <div className={styles.stars}>
                  {[1, 2, 3, 4, 5].map((star) => (
                    <button
                      key={star}
                      className={`${styles.star} ${rating >= star ? styles.starActive : ''}`}
                      onClick={() => setRating(star)}
                      aria-label={`Rate ${star} stars`}
                    >
                      {rating >= star ? '★' : '☆'}
                    </button>
                  ))}
                </div>

                <textarea
                  className={styles.textarea}
                  placeholder="Any suggestions? (optional)"
                  value={feedback}
                  onChange={(e) => setFeedback(e.target.value)}
                  rows={3}
                />

                <button
                  className={styles.submitButton}
                  onClick={handleSubmit}
                  disabled={rating === 0}
                >
                  Submit Feedback
                </button>
              </div>
            </>
          ) : (
            <div className={styles.success}>
              <div className={styles.successIcon}>✓</div>
              <p className={styles.successText}>Thank you for your feedback!</p>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default QuickFeedback;
