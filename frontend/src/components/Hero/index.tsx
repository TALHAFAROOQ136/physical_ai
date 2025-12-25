import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

/**
 * Hero section component for the landing page.
 * Displays course title, description, and call-to-action buttons.
 */
export default function Hero(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.heroGrid} />
      </div>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <span className={styles.badge}>Open Source Textbook</span>
            <h1 className={styles.title}>
              Physical AI &<br />
              <span className={styles.highlight}>Humanoid Robotics</span>
            </h1>
            <p className={styles.subtitle}>{siteConfig.tagline}</p>
            <div className={styles.heroActions}>
              <Link
                className={clsx('button button--primary button--lg', styles.primaryButton)}
                to="/docs/intro"
              >
                Start Learning
              </Link>
              <Link
                className={clsx('button button--secondary button--lg', styles.secondaryButton)}
                to="/docs/module-1-ros2"
              >
                Browse Modules
              </Link>
            </div>
            <div className={styles.stats}>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>4</span>
                <span className={styles.statLabel}>Modules</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>20+</span>
                <span className={styles.statLabel}>Chapters</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>50+</span>
                <span className={styles.statLabel}>Exercises</span>
              </div>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.robotIcon}>
              <RobotSVG />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function RobotSVG() {
  return (
    <svg
      viewBox="0 0 200 200"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
      className={styles.robotSvg}
    >
      {/* Head */}
      <rect x="60" y="30" width="80" height="60" rx="10" fill="currentColor" opacity="0.9" />
      {/* Eyes */}
      <circle cx="85" cy="55" r="10" fill="white" />
      <circle cx="115" cy="55" r="10" fill="white" />
      <circle cx="85" cy="55" r="5" fill="#2563eb" />
      <circle cx="115" cy="55" r="5" fill="#2563eb" />
      {/* Antenna */}
      <rect x="95" y="15" width="10" height="20" rx="5" fill="currentColor" opacity="0.7" />
      <circle cx="100" cy="10" r="8" fill="#2563eb" />
      {/* Body */}
      <rect x="50" y="100" width="100" height="70" rx="8" fill="currentColor" opacity="0.85" />
      {/* Chest lights */}
      <rect x="70" y="115" width="20" height="8" rx="4" fill="#10b981" />
      <rect x="110" y="115" width="20" height="8" rx="4" fill="#10b981" />
      <rect x="85" y="130" width="30" height="25" rx="4" fill="white" opacity="0.3" />
      {/* Arms */}
      <rect x="25" y="105" width="20" height="50" rx="10" fill="currentColor" opacity="0.75" />
      <rect x="155" y="105" width="20" height="50" rx="10" fill="currentColor" opacity="0.75" />
      {/* Hands */}
      <circle cx="35" cy="165" r="12" fill="currentColor" opacity="0.7" />
      <circle cx="165" cy="165" r="12" fill="currentColor" opacity="0.7" />
      {/* Legs */}
      <rect x="60" y="175" width="25" height="15" rx="4" fill="currentColor" opacity="0.8" />
      <rect x="115" y="175" width="25" height="15" rx="4" fill="currentColor" opacity="0.8" />
    </svg>
  );
}
