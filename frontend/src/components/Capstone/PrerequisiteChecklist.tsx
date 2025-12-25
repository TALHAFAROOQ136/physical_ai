/**
 * Prerequisite checklist component for capstone milestones.
 * Shows required knowledge and setup before starting a milestone.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Capstone.module.css';

interface Prerequisite {
  id: string;
  label: string;
  description?: string;
  link?: string;
  required: boolean;
}

interface PrerequisiteChecklistProps {
  title?: string;
  prerequisites: Prerequisite[];
  completedIds?: string[];
  onToggle?: (id: string, completed: boolean) => void;
  readOnly?: boolean;
}

export function PrerequisiteChecklist({
  title = 'Prerequisites',
  prerequisites,
  completedIds = [],
  onToggle,
  readOnly = true,
}: PrerequisiteChecklistProps): JSX.Element {
  const requiredPrereqs = prerequisites.filter((p) => p.required);
  const optionalPrereqs = prerequisites.filter((p) => !p.required);

  const completedRequired = requiredPrereqs.filter((p) =>
    completedIds.includes(p.id)
  ).length;
  const allRequiredComplete = completedRequired === requiredPrereqs.length;

  const handleToggle = (id: string) => {
    if (readOnly || !onToggle) return;
    onToggle(id, !completedIds.includes(id));
  };

  const renderPrerequisite = (prereq: Prerequisite) => {
    const isCompleted = completedIds.includes(prereq.id);

    return (
      <li
        key={prereq.id}
        className={`${styles.prereqItem} ${isCompleted ? styles.completed : ''}`}
      >
        <label className={styles.prereqLabel}>
          <input
            type="checkbox"
            checked={isCompleted}
            onChange={() => handleToggle(prereq.id)}
            disabled={readOnly}
            className={styles.prereqCheckbox}
          />
          <span className={styles.prereqText}>
            {prereq.link ? (
              <Link to={prereq.link}>{prereq.label}</Link>
            ) : (
              prereq.label
            )}
            {!prereq.required && (
              <span className={styles.optionalTag}>Optional</span>
            )}
          </span>
        </label>
        {prereq.description && (
          <p className={styles.prereqDescription}>{prereq.description}</p>
        )}
      </li>
    );
  };

  return (
    <div className={styles.checklist}>
      <div className={styles.checklistHeader}>
        <h4 className={styles.checklistTitle}>{title}</h4>
        <span className={styles.checklistProgress}>
          {completedRequired} / {requiredPrereqs.length} required
        </span>
      </div>

      {!allRequiredComplete && requiredPrereqs.length > 0 && (
        <div className={styles.checklistWarning}>
          Complete all required prerequisites before starting this milestone.
        </div>
      )}

      {requiredPrereqs.length > 0 && (
        <div className={styles.prereqSection}>
          <h5 className={styles.prereqSectionTitle}>Required</h5>
          <ul className={styles.prereqList}>
            {requiredPrereqs.map(renderPrerequisite)}
          </ul>
        </div>
      )}

      {optionalPrereqs.length > 0 && (
        <div className={styles.prereqSection}>
          <h5 className={styles.prereqSectionTitle}>Recommended</h5>
          <ul className={styles.prereqList}>
            {optionalPrereqs.map(renderPrerequisite)}
          </ul>
        </div>
      )}

      {allRequiredComplete && (
        <div className={styles.checklistSuccess}>
          All prerequisites complete! You are ready to start this milestone.
        </div>
      )}
    </div>
  );
}

export default PrerequisiteChecklist;
