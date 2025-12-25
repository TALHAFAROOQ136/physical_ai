/**
 * Validation script component for testing capstone progress.
 * Displays validation scripts and their expected outputs.
 */

import React, { useState } from 'react';
import styles from './Capstone.module.css';

interface ValidationStep {
  id: string;
  command: string;
  description: string;
  expectedOutput?: string;
  successCriteria: string;
}

interface ValidationScriptProps {
  title?: string;
  steps: ValidationStep[];
}

export function ValidationScript({
  title = 'Validation Steps',
  steps,
}: ValidationScriptProps): JSX.Element {
  const [expandedSteps, setExpandedSteps] = useState<Set<string>>(new Set());

  const toggleStep = (id: string) => {
    setExpandedSteps((prev) => {
      const next = new Set(prev);
      if (next.has(id)) {
        next.delete(id);
      } else {
        next.add(id);
      }
      return next;
    });
  };

  const copyToClipboard = async (text: string) => {
    try {
      await navigator.clipboard.writeText(text);
    } catch (err) {
      console.error('Failed to copy:', err);
    }
  };

  return (
    <div className={styles.validation}>
      <h4 className={styles.validationTitle}>{title}</h4>
      <p className={styles.validationIntro}>
        Run these commands to verify your milestone is working correctly.
      </p>

      <div className={styles.validationSteps}>
        {steps.map((step, index) => {
          const isExpanded = expandedSteps.has(step.id);

          return (
            <div key={step.id} className={styles.validationStep}>
              <div className={styles.stepHeader}>
                <span className={styles.stepNumber}>{index + 1}</span>
                <button
                  className={styles.stepToggle}
                  onClick={() => toggleStep(step.id)}
                  aria-expanded={isExpanded}
                >
                  <span className={styles.stepDescription}>
                    {step.description}
                  </span>
                  <span className={styles.stepIcon}>
                    {isExpanded ? '−' : '+'}
                  </span>
                </button>
              </div>

              {isExpanded && (
                <div className={styles.stepContent}>
                  <div className={styles.commandBlock}>
                    <div className={styles.commandHeader}>
                      <span>Command</span>
                      <button
                        className={styles.copyButton}
                        onClick={() => copyToClipboard(step.command)}
                        title="Copy command"
                      >
                        Copy
                      </button>
                    </div>
                    <pre className={styles.commandCode}>
                      <code>{step.command}</code>
                    </pre>
                  </div>

                  {step.expectedOutput && (
                    <div className={styles.outputBlock}>
                      <span className={styles.outputLabel}>Expected Output</span>
                      <pre className={styles.outputCode}>
                        <code>{step.expectedOutput}</code>
                      </pre>
                    </div>
                  )}

                  <div className={styles.criteriaBlock}>
                    <span className={styles.criteriaLabel}>Success Criteria</span>
                    <p className={styles.criteriaText}>{step.successCriteria}</p>
                  </div>
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}

export default ValidationScript;
