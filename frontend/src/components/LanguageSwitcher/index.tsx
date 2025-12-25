/**
 * Language switcher component.
 * Provides a UI for switching between available locales.
 */

import React, { useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '../../context/AuthContext';
import styles from './LanguageSwitcher.module.css';

interface LanguageSwitcherProps {
  variant?: 'dropdown' | 'buttons';
}

export function LanguageSwitcher({
  variant = 'buttons',
}: LanguageSwitcherProps): JSX.Element {
  const { i18n, siteConfig } = useDocusaurusContext();
  const { pathname } = useLocation();
  const { user, updatePreferences } = useAuth();

  const localeConfigs = siteConfig.i18n?.localeConfigs || {};
  const currentLocale = i18n.currentLocale;

  const handleLocaleChange = useCallback(
    async (locale: string) => {
      // Build the new path
      const pathWithoutLocale = pathname.replace(/^\/[a-z]{2}(-[A-Z]{2})?\//, '/');
      const newPath = locale === i18n.defaultLocale
        ? pathWithoutLocale
        : `/${locale}${pathWithoutLocale}`;

      // Save preference if user is logged in
      if (user) {
        try {
          await updatePreferences({ languagePreference: locale });
        } catch (error) {
          console.debug('Failed to save language preference:', error);
        }
      }

      // Navigate to the new locale
      window.location.href = newPath;
    },
    [pathname, i18n.defaultLocale, user, updatePreferences]
  );

  if (variant === 'dropdown') {
    return (
      <select
        className={styles.select}
        value={currentLocale}
        onChange={(e) => handleLocaleChange(e.target.value)}
        aria-label="Select language"
      >
        {Object.entries(localeConfigs).map(([locale, config]) => (
          <option key={locale} value={locale}>
            {config.label}
          </option>
        ))}
      </select>
    );
  }

  // Buttons variant
  return (
    <div className={styles.buttons}>
      {Object.entries(localeConfigs).map(([locale, config]) => (
        <button
          key={locale}
          className={`${styles.button} ${
            locale === currentLocale ? styles.active : ''
          }`}
          onClick={() => handleLocaleChange(locale)}
          aria-label={`Switch to ${config.label}`}
          aria-pressed={locale === currentLocale}
          dir={config.direction}
        >
          {config.label}
        </button>
      ))}
    </div>
  );
}

export default LanguageSwitcher;
