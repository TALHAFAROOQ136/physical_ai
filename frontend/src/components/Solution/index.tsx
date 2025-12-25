import React, { useState, ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface SolutionProps {
  children: ReactNode;
  title?: string;
}

/**
 * Collapsible Solution component for exercises.
 * Hides solution content until the user clicks to reveal.
 */
export default function Solution({
  children,
  title = 'Show Solution',
}: SolutionProps): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(false);

  return (
    <div className={clsx(styles.solution, isExpanded && styles.expanded)}>
      <button
        className={styles.solutionToggle}
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
      >
        <ChevronIcon className={clsx(styles.chevron, isExpanded && styles.chevronExpanded)} />
        <span>{isExpanded ? 'Hide Solution' : title}</span>
      </button>
      <div
        className={styles.solutionContent}
        style={{ display: isExpanded ? 'block' : 'none' }}
      >
        {children}
      </div>
    </div>
  );
}

function ChevronIcon({ className }: { className?: string }) {
  return (
    <svg
      className={className}
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <polyline points="9 18 15 12 9 6" />
    </svg>
  );
}
