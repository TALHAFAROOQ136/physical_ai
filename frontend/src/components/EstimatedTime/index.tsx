import React from 'react';
import styles from './styles.module.css';

interface EstimatedTimeProps {
  minutes: number;
}

/**
 * Estimated Time component showing reading duration.
 * Displays how long it will take to complete a chapter.
 */
export default function EstimatedTime({
  minutes,
}: EstimatedTimeProps): JSX.Element {
  const formatTime = (mins: number): string => {
    if (mins < 60) {
      return `${mins} min`;
    }
    const hours = Math.floor(mins / 60);
    const remainingMins = mins % 60;
    if (remainingMins === 0) {
      return `${hours} hr`;
    }
    return `${hours} hr ${remainingMins} min`;
  };

  const getDifficulty = (mins: number): { label: string; color: string } => {
    if (mins <= 15) {
      return { label: 'Quick Read', color: 'success' };
    }
    if (mins <= 30) {
      return { label: 'Medium', color: 'warning' };
    }
    if (mins <= 60) {
      return { label: 'In-depth', color: 'info' };
    }
    return { label: 'Deep Dive', color: 'danger' };
  };

  const difficulty = getDifficulty(minutes);

  return (
    <div className={styles.estimatedTime}>
      <ClockIcon />
      <span className={styles.time}>{formatTime(minutes)}</span>
      <span className={`${styles.badge} ${styles[difficulty.color]}`}>
        {difficulty.label}
      </span>
    </div>
  );
}

function ClockIcon() {
  return (
    <svg
      className={styles.icon}
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <circle cx="12" cy="12" r="10" />
      <polyline points="12 6 12 12 16 14" />
    </svg>
  );
}
