/**
 * Next chapter recommendation component.
 * Shows the recommended next chapter based on user progress and background.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Recommendations.module.css';

interface NextChapterData {
  chapterId: string;
  title: string;
  moduleId: string;
  reason: string;
}

interface NextChapterProps {
  recommendation: NextChapterData;
  variant?: 'card' | 'inline' | 'compact';
}

export function NextChapter({
  recommendation,
  variant = 'card',
}: NextChapterProps): JSX.Element {
  const chapterPath = `/docs/${recommendation.moduleId}/${recommendation.chapterId.replace(`${recommendation.moduleId}-`, '')}`;

  if (variant === 'inline') {
    return (
      <div className={styles.nextChapterInline}>
        <span className={styles.nextLabel}>Next:</span>
        <Link to={chapterPath} className={styles.nextChapterLink}>
          {recommendation.title}
        </Link>
      </div>
    );
  }

  if (variant === 'compact') {
    return (
      <Link to={chapterPath} className={styles.nextChapterCompact}>
        <span className={styles.nextIcon}>→</span>
        <span className={styles.nextTitle}>{recommendation.title}</span>
      </Link>
    );
  }

  // Default card variant
  return (
    <div className={styles.nextChapterCard}>
      <div className={styles.nextChapterHeader}>
        <span className={styles.nextChapterIcon}>📖</span>
        <span className={styles.nextChapterLabel}>Recommended Next</span>
      </div>
      <div className={styles.nextChapterBody}>
        <h3 className={styles.nextChapterTitle}>
          <Link to={chapterPath}>{recommendation.title}</Link>
        </h3>
        <p className={styles.nextChapterReason}>{recommendation.reason}</p>
      </div>
      <div className={styles.nextChapterFooter}>
        <Link to={chapterPath} className={styles.nextChapterButton}>
          Continue Learning →
        </Link>
      </div>
    </div>
  );
}

export default NextChapter;
