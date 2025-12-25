import React from 'react';
import styles from './styles.module.css';

interface LearningObjectivesProps {
  objectives: string[];
}

/**
 * Learning Objectives component for chapter headers.
 * Displays a list of what students will learn.
 */
export default function LearningObjectives({
  objectives,
}: LearningObjectivesProps): JSX.Element {
  return (
    <div className={styles.learningObjectives}>
      <div className={styles.header}>
        <TargetIcon />
        <h4 className={styles.title}>Learning Objectives</h4>
      </div>
      <ul className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <li key={index} className={styles.objective}>
            <CheckCircleIcon />
            <span>{objective}</span>
          </li>
        ))}
      </ul>
    </div>
  );
}

function TargetIcon() {
  return (
    <svg
      className={styles.icon}
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <circle cx="12" cy="12" r="10" />
      <circle cx="12" cy="12" r="6" />
      <circle cx="12" cy="12" r="2" />
    </svg>
  );
}

function CheckCircleIcon() {
  return (
    <svg
      className={styles.checkIcon}
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
      <polyline points="22 4 12 14.01 9 11.01" />
    </svg>
  );
}
