/**
 * Skip recommendation banner component.
 * Shows suggestions for chapters the user may want to skip based on their background.
 */

import React, { useState } from 'react';
import styles from './Recommendations.module.css';

interface SkipSuggestion {
  chapterId: string;
  title: string;
  reason: string;
}

interface SkipBannerProps {
  suggestion: SkipSuggestion;
  onDismiss?: () => void;
  onSkip?: () => void;
  onContinue?: () => void;
}

export function SkipBanner({
  suggestion,
  onDismiss,
  onSkip,
  onContinue,
}: SkipBannerProps): JSX.Element {
  const [dismissed, setDismissed] = useState(false);

  if (dismissed) {
    return <></>;
  }

  const handleDismiss = () => {
    setDismissed(true);
    onDismiss?.();
  };

  return (
    <div className={styles.skipBanner}>
      <div className={styles.skipBannerIcon}>
        <span role="img" aria-label="lightbulb">💡</span>
      </div>
      <div className={styles.skipBannerContent}>
        <div className={styles.skipBannerTitle}>
          You might be able to skip this chapter
        </div>
        <div className={styles.skipBannerReason}>
          {suggestion.reason}
        </div>
        <div className={styles.skipBannerActions}>
          <button
            className={styles.skipButton}
            onClick={onSkip}
          >
            Skip to next chapter
          </button>
          <button
            className={styles.continueButton}
            onClick={onContinue}
          >
            Read anyway
          </button>
          <button
            className={styles.dismissButton}
            onClick={handleDismiss}
            aria-label="Dismiss suggestion"
          >
            ×
          </button>
        </div>
      </div>
    </div>
  );
}

export default SkipBanner;
