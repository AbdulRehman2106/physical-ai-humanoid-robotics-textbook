import React, { useState } from 'react';
import styles from './styles.module.css';

export interface QuizQuestion {
  id: string;
  question: string;
  options: string[];
  correctAnswer: number;
  explanation?: string;
}

export interface QuizProps {
  title?: string;
  questions: QuizQuestion[];
}

const Quiz: React.FC<QuizProps> = ({ title = 'Quiz', questions }) => {
  const [answers, setAnswers] = useState<Record<string, number>>({});
  const [submitted, setSubmitted] = useState(false);
  const [showFeedback, setShowFeedback] = useState<Record<string, boolean>>({});

  const handleAnswerChange = (questionId: string, answerIndex: number) => {
    if (!submitted) {
      setAnswers({ ...answers, [questionId]: answerIndex });
    }
  };

  const handleSubmit = () => {
    setSubmitted(true);
    const feedback: Record<string, boolean> = {};
    questions.forEach((q) => {
      feedback[q.id] = true;
    });
    setShowFeedback(feedback);
  };

  const calculateScore = () => {
    let correct = 0;
    questions.forEach((q) => {
      if (answers[q.id] === q.correctAnswer) {
        correct++;
      }
    });
    return { correct, total: questions.length };
  };

  const score = submitted ? calculateScore() : null;

  return (
    <div className={styles.quiz} role="region" aria-label={title}>
      <h3 className={styles.quizTitle}>{title}</h3>

      {questions.map((question, qIndex) => {
        const userAnswer = answers[question.id];
        const isCorrect = userAnswer === question.correctAnswer;
        const showQuestionFeedback = submitted && showFeedback[question.id];

        return (
          <div key={question.id} className={styles.question}>
            <p className={styles.questionText}>
              {qIndex + 1}. {question.question}
            </p>

            <div className={styles.options} role="radiogroup" aria-label={`Question ${qIndex + 1}`}>
              {question.options.map((option, oIndex) => {
                const isSelected = userAnswer === oIndex;
                const isCorrectOption = oIndex === question.correctAnswer;

                let optionClass = styles.option;
                if (submitted && isSelected) {
                  optionClass += isCorrect ? ` ${styles.optionCorrect}` : ` ${styles.optionIncorrect}`;
                } else if (submitted && isCorrectOption) {
                  optionClass += ` ${styles.optionCorrect}`;
                }

                return (
                  <div key={oIndex} className={optionClass}>
                    <input
                      type="radio"
                      id={`${question.id}-${oIndex}`}
                      name={question.id}
                      value={oIndex}
                      checked={isSelected}
                      onChange={() => handleAnswerChange(question.id, oIndex)}
                      disabled={submitted}
                      aria-label={`Option ${oIndex + 1}`}
                    />
                    <label htmlFor={`${question.id}-${oIndex}`}>{option}</label>
                  </div>
                );
              })}
            </div>

            {showQuestionFeedback && question.explanation && (
              <div
                className={`${styles.feedback} ${
                  isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect
                }`}
                role="alert"
              >
                <strong>{isCorrect ? '✓ Correct!' : '✗ Incorrect.'}</strong>{' '}
                {question.explanation}
              </div>
            )}
          </div>
        );
      })}

      {!submitted && (
        <button
          className={styles.submitButton}
          onClick={handleSubmit}
          disabled={Object.keys(answers).length !== questions.length}
          aria-label="Submit quiz answers"
        >
          Submit Answers
        </button>
      )}

      {score && (
        <div className={styles.score} role="status" aria-live="polite">
          Score: {score.correct} / {score.total} ({Math.round((score.correct / score.total) * 100)}%)
        </div>
      )}
    </div>
  );
};

export default Quiz;
