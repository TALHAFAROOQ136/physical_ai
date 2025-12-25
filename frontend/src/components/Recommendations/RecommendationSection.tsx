/**
 * Recommendation section component for dashboard.
 * Combines next chapter, skip suggestions, and focus areas into a unified view.
 */

import React, { useEffect, useState } from 'react';
import { NextChapter } from './NextChapter';
import { FocusAreas } from './FocusAreas';
import { SkipBanner } from './SkipBanner';
import { progressService, Recommendations } from '../../services/progress';
import { useAuth } from '../../context/AuthContext';
import styles from './Recommendations.module.css';

interface RecommendationSectionProps {
  showSkipBanner?: boolean;
  showFocusAreas?: boolean;
  maxFocusAreas?: number;
}

export function RecommendationSection({
  showSkipBanner = true,
  showFocusAreas = true,
  maxFocusAreas = 4,
}: RecommendationSectionProps): JSX.Element | null {
  const { user, isLoading: authLoading } = useAuth();
  const [recommendations, setRecommendations] = useState<Recommendations | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!user || authLoading) {
      setLoading(false);
      return;
    }

    setLoading(true);
    progressService
      .getRecommendations()
      .then((data) => {
        setRecommendations(data);
        setError(null);
      })
      .catch((err) => {
        console.error('Failed to fetch recommendations:', err);
        setError('Failed to load recommendations');
      })
      .finally(() => {
        setLoading(false);
      });
  }, [user, authLoading]);

  // Don't render for non-authenticated users
  if (!user && !authLoading) {
    return null;
  }

  if (loading) {
    return (
      <div className={styles.recommendationSection}>
        <div className={styles.recommendationSectionTitle}>
          <span className={styles.recommendationSectionIcon}>🎯</span>
          Personalized Recommendations
        </div>
        <div>Loading recommendations...</div>
      </div>
    );
  }

  if (error || !recommendations) {
    return null;
  }

  const hasRecommendations =
    recommendations.nextChapter ||
    recommendations.skipSuggestions.length > 0 ||
    recommendations.focusAreas.length > 0;

  if (!hasRecommendations) {
    return null;
  }

  return (
    <div className={styles.recommendationSection}>
      <div className={styles.recommendationSectionTitle}>
        <span className={styles.recommendationSectionIcon}>🎯</span>
        Personalized Recommendations
      </div>

      {/* Skip suggestions */}
      {showSkipBanner && recommendations.skipSuggestions.length > 0 && (
        <SkipBanner
          suggestion={recommendations.skipSuggestions[0]}
          onSkip={() => {
            // Navigate to next chapter
            if (recommendations.nextChapter) {
              window.location.href = `/docs/${recommendations.nextChapter.moduleId}/${recommendations.nextChapter.chapterId.replace(`${recommendations.nextChapter.moduleId}-`, '')}`;
            }
          }}
          onContinue={() => {
            // Just dismiss the banner
          }}
        />
      )}

      {/* Next chapter recommendation */}
      {recommendations.nextChapter && (
        <NextChapter
          recommendation={recommendations.nextChapter}
          variant="card"
        />
      )}

      {/* Focus areas */}
      {showFocusAreas && recommendations.focusAreas.length > 0 && (
        <>
          <h4 style={{ margin: '1.5rem 0 0.75rem', fontSize: '0.875rem' }}>
            Focus Areas
          </h4>
          <FocusAreas areas={recommendations.focusAreas} maxItems={maxFocusAreas} />
        </>
      )}
    </div>
  );
}

export default RecommendationSection;
