import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface ModuleCardProps {
  title: string;
  subtitle: string;
  description: string;
  icon: string;
  link: string;
  color: string;
  duration: string;
}

/**
 * Module card component for displaying course modules.
 * Shows module info with icon, description, and link.
 */
export default function ModuleCard({
  title,
  subtitle,
  description,
  icon,
  link,
  color,
  duration,
}: ModuleCardProps): JSX.Element {
  return (
    <Link to={link} className={styles.card}>
      <div className={styles.cardHeader} style={{ borderColor: color }}>
        <span className={styles.icon}>{icon}</span>
        <span className={styles.duration}>{duration}</span>
      </div>
      <div className={styles.cardBody}>
        <h3 className={styles.title}>{title}</h3>
        <p className={styles.subtitle} style={{ color }}>{subtitle}</p>
        <p className={styles.description}>{description}</p>
      </div>
      <div className={styles.cardFooter}>
        <span className={styles.link}>
          Start Module
          <ArrowIcon />
        </span>
      </div>
    </Link>
  );
}

function ArrowIcon() {
  return (
    <svg
      className={styles.arrowIcon}
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <line x1="5" y1="12" x2="19" y2="12" />
      <polyline points="12 5 19 12 12 19" />
    </svg>
  );
}
