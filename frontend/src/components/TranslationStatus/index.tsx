/**
 * Translation status indicator component.
 * Shows the translation status and quality for content in non-default locales.
 */

import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './TranslationStatus.module.css';

type TranslationQuality = 'complete' | 'partial' | 'machine' | 'pending';

interface TranslationStatusProps {
  quality?: TranslationQuality;
  lastUpdated?: string;
  showOnDefaultLocale?: boolean;
}

const statusConfig = {
  complete: {
    label: 'مکمل ترجمہ',
    labelEn: 'Fully Translated',
    icon: '✓',
    className: 'complete',
  },
  partial: {
    label: 'جزوی ترجمہ',
    labelEn: 'Partially Translated',
    icon: '◐',
    className: 'partial',
  },
  machine: {
    label: 'مشینی ترجمہ',
    labelEn: 'Machine Translated',
    icon: '🤖',
    className: 'machine',
  },
  pending: {
    label: 'ترجمہ زیر التوا',
    labelEn: 'Translation Pending',
    icon: '⏳',
    className: 'pending',
  },
};

export function TranslationStatus({
  quality = 'pending',
  lastUpdated,
  showOnDefaultLocale = false,
}: TranslationStatusProps): JSX.Element | null {
  const { i18n } = useDocusaurusContext();
  const { pathname } = useLocation();

  // Don't show on default locale unless explicitly requested
  if (i18n.currentLocale === i18n.defaultLocale && !showOnDefaultLocale) {
    return null;
  }

  const config = statusConfig[quality];
  const isUrdu = i18n.currentLocale === 'ur';

  return (
    <div className={`${styles.container} ${styles[config.className]}`}>
      <span className={styles.icon}>{config.icon}</span>
      <span className={styles.label}>
        {isUrdu ? config.label : config.labelEn}
      </span>
      {quality === 'machine' && (
        <span className={styles.disclaimer}>
          {isUrdu
            ? '- اس ترجمے کا جائزہ جاری ہے'
            : '- This translation is under review'}
        </span>
      )}
      {lastUpdated && (
        <span className={styles.updated}>
          {isUrdu ? `آخری اپ ڈیٹ: ${lastUpdated}` : `Last updated: ${lastUpdated}`}
        </span>
      )}
    </div>
  );
}

export default TranslationStatus;
