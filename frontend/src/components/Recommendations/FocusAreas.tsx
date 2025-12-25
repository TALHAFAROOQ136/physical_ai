/**
 * Focus areas recommendation component.
 * Shows learning focus areas based on user's goals.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Recommendations.module.css';

interface FocusArea {
  moduleId: string;
  moduleTitle: string;
  priority: 'high' | 'medium' | 'low';
}

interface FocusAreasProps {
  areas: FocusArea[];
  maxItems?: number;
}

export function FocusAreas({
  areas,
  maxItems = 4,
}: FocusAreasProps): JSX.Element {
  const displayAreas = areas.slice(0, maxItems);

  return (
    <div className={styles.focusAreas}>
      {displayAreas.map((area) => (
        <Link
          key={area.moduleId}
          to={`/docs/${area.moduleId}`}
          className={`${styles.focusAreaItem} ${styles[area.priority]}`}
        >
          <span className={`${styles.focusAreaPriority} ${styles[area.priority]}`}>
            {area.priority}
          </span>
          <span className={styles.focusAreaTitle}>{area.moduleTitle}</span>
        </Link>
      ))}
    </div>
  );
}

export default FocusAreas;
