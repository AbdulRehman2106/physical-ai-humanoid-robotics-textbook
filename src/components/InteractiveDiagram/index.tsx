import React, { useState } from 'react';
import styles from './styles.module.css';

export interface DiagramStep {
  id: string;
  label: string;
  description: string;
  imageSrc?: string;
}

export interface InteractiveDiagramProps {
  title: string;
  steps: DiagramStep[];
  defaultImageSrc?: string;
}

const InteractiveDiagram: React.FC<InteractiveDiagramProps> = ({
  title,
  steps,
  defaultImageSrc,
}) => {
  const [activeStep, setActiveStep] = useState<string>(steps[0]?.id || '');
  const [isModalOpen, setIsModalOpen] = useState(false);

  const currentStep = steps.find((s) => s.id === activeStep) || steps[0];
  const imageSrc = currentStep?.imageSrc || defaultImageSrc;

  const handleImageClick = () => {
    if (imageSrc) {
      setIsModalOpen(true);
    }
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  return (
    <div className={styles.diagram} role="region" aria-label={title}>
      <h4 className={styles.diagramTitle}>{title}</h4>

      <div className={styles.diagramContent}>
        {imageSrc && (
          <img
            src={imageSrc}
            alt={`${title} - ${currentStep?.label || 'Diagram'}`}
            className={styles.diagramImage}
            onClick={handleImageClick}
            role="button"
            tabIndex={0}
            onKeyPress={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                handleImageClick();
              }
            }}
          />
        )}

        {steps.length > 1 && (
          <div className={styles.diagramControls} role="tablist">
            {steps.map((step) => (
              <button
                key={step.id}
                className={`${styles.controlButton} ${
                  activeStep === step.id ? styles.controlButtonActive : ''
                }`}
                onClick={() => setActiveStep(step.id)}
                role="tab"
                aria-selected={activeStep === step.id}
                aria-controls={`diagram-panel-${step.id}`}
              >
                {step.label}
              </button>
            ))}
          </div>
        )}

        {currentStep && (
          <div
            className={styles.diagramDescription}
            id={`diagram-panel-${currentStep.id}`}
            role="tabpanel"
          >
            {currentStep.description}
          </div>
        )}
      </div>

      {isModalOpen && imageSrc && (
        <div
          className={styles.modal}
          onClick={handleCloseModal}
          role="dialog"
          aria-modal="true"
          aria-label="Enlarged diagram view"
        >
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <button
              className={styles.closeButton}
              onClick={handleCloseModal}
              aria-label="Close enlarged view"
            >
              ×
            </button>
            <img
              src={imageSrc}
              alt={`${title} - ${currentStep?.label || 'Diagram'} (enlarged)`}
              className={styles.modalImage}
            />
          </div>
        </div>
      )}
    </div>
  );
};

export default InteractiveDiagram;
