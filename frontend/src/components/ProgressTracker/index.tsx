/**
 * Progress tracker component for chapter completion.
 */

import React, { useState, useCallback } from 'react';
import { progressService, ChapterProgress } from '../../services/progress';
import styles from './ProgressTracker.module.css';

interface ProgressTrackerProps {
  chapterId: string;
  initialProgress?: ChapterProgress;
  onProgressChange?: (progress: ChapterProgress) => void;
}

export function ProgressTracker({
  chapterId,
  initialProgress,
  onProgressChange,
}: ProgressTrackerProps): JSX.Element {
  const [progress, setProgress] = useState<ChapterProgress | undefined>(
    initialProgress
  );
  const [loading, setLoading] = useState(false);

  const isCompleted = progress?.status === 'completed';

  const handleToggleComplete = useCallback(async () => {
    setLoading(true);
    try {
      let newProgress: ChapterProgress;
      if (isCompleted) {
        newProgress = await progressService.unmarkChapterComplete(chapterId);
      } else {
        newProgress = await progressService.markChapterComplete(chapterId);
      }
      setProgress(newProgress);
      onProgressChange?.(newProgress);
    } catch (error) {
      console.error('Failed to update progress:', error);
    } finally {
      setLoading(false);
    }
  }, [chapterId, isCompleted, onProgressChange]);

  return (
    <div className={styles.tracker}>
      <button
        className={`${styles.completeButton} ${
          isCompleted ? styles.completed : ''
        }`}
        onClick={handleToggleComplete}
        disabled={loading}
      >
        {loading ? (
          <span className={styles.spinner} />
        ) : isCompleted ? (
          <>
            <span className={styles.checkIcon}>✓</span>
            <span>Completed</span>
          </>
        ) : (
          <>
            <span className={styles.circleIcon}>○</span>
            <span>Mark Complete</span>
          </>
        )}
      </button>

      {progress && progress.readingTimeSeconds > 0 && (
        <span className={styles.readingTime}>
          {Math.round(progress.readingTimeSeconds / 60)} min read
        </span>
      )}
    </div>
  );
}

export default ProgressTracker;
