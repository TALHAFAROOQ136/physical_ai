/**
 * Progress bar component showing module completion.
 */

import React from 'react';
import styles from './ProgressBar.module.css';

interface ProgressBarProps {
  completed: number;
  total: number;
  label?: string;
  showPercentage?: boolean;
  size?: 'small' | 'medium' | 'large';
  color?: 'primary' | 'success' | 'warning';
}

export function ProgressBar({
  completed,
  total,
  label,
  showPercentage = true,
  size = 'medium',
  color = 'primary',
}: ProgressBarProps): JSX.Element {
  const percentage = total > 0 ? Math.round((completed / total) * 100) : 0;

  return (
    <div className={`${styles.container} ${styles[size]}`}>
      {label && <span className={styles.label}>{label}</span>}
      <div className={styles.barContainer}>
        <div className={styles.barBackground}>
          <div
            className={`${styles.barFill} ${styles[color]}`}
            style={{ width: `${percentage}%` }}
          />
        </div>
        {showPercentage && (
          <span className={styles.percentage}>{percentage}%</span>
        )}
      </div>
      <span className={styles.count}>
        {completed} / {total}
      </span>
    </div>
  );
}

export default ProgressBar;
