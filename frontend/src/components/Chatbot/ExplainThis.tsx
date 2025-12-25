/**
 * ExplainThis component - context menu for explaining selected text.
 */

import React, { useState, useEffect, useCallback } from 'react';
import { chatbotService } from '../../services/chatbot';
import styles from './ExplainThis.module.css';

interface ExplainThisProps {
  chapterId?: string;
}

interface Position {
  x: number;
  y: number;
}

interface ExplanationState {
  isLoading: boolean;
  explanation: string | null;
  error: string | null;
}

export default function ExplainThis({
  chapterId,
}: ExplainThisProps): JSX.Element | null {
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [position, setPosition] = useState<Position | null>(null);
  const [showPopup, setShowPopup] = useState(false);
  const [explanation, setExplanation] = useState<ExplanationState>({
    isLoading: false,
    explanation: null,
    error: null,
  });

  // Handle text selection
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10 && text.length < 500) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectedText(text);
          setPosition({
            x: rect.left + rect.width / 2,
            y: rect.bottom + window.scrollY,
          });
          setShowPopup(true);
          setExplanation({
            isLoading: false,
            explanation: null,
            error: null,
          });
        }
      }
    };

    const handleMouseDown = (e: MouseEvent) => {
      // Hide popup if clicking outside
      const target = e.target as HTMLElement;
      if (!target.closest(`.${styles.popup}`) && !target.closest(`.${styles.button}`)) {
        setShowPopup(false);
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
    };
  }, []);

  // Handle keyboard shortcut (Ctrl+Shift+E)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.ctrlKey && e.shiftKey && e.key === 'E') {
        if (selectedText) {
          handleExplain();
        }
      }
      if (e.key === 'Escape') {
        setShowPopup(false);
        setSelectedText(null);
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [selectedText]);

  const handleExplain = useCallback(async () => {
    if (!selectedText) return;

    setExplanation({
      isLoading: true,
      explanation: null,
      error: null,
    });

    try {
      const result = await chatbotService.explainText(selectedText, chapterId);
      setExplanation({
        isLoading: false,
        explanation: result.explanation,
        error: null,
      });
    } catch (error) {
      console.error('Failed to explain text:', error);
      setExplanation({
        isLoading: false,
        explanation: null,
        error: 'Failed to get explanation. Please try again.',
      });
    }
  }, [selectedText, chapterId]);

  const handleClose = () => {
    setShowPopup(false);
    setSelectedText(null);
    setExplanation({
      isLoading: false,
      explanation: null,
      error: null,
    });
  };

  if (!showPopup || !position) return null;

  return (
    <div
      className={styles.popup}
      style={{
        left: `${position.x}px`,
        top: `${position.y + 8}px`,
        transform: 'translateX(-50%)',
      }}
    >
      {explanation.explanation ? (
        <div className={styles.explanationContent}>
          <div className={styles.explanationHeader}>
            <span className={styles.explanationTitle}>Explanation</span>
            <button
              className={styles.closeButton}
              onClick={handleClose}
              aria-label="Close"
            >
              ×
            </button>
          </div>
          <div className={styles.explanationText}>{explanation.explanation}</div>
        </div>
      ) : explanation.error ? (
        <div className={styles.errorContent}>
          <span>{explanation.error}</span>
          <button className={styles.retryButton} onClick={handleExplain}>
            Retry
          </button>
        </div>
      ) : explanation.isLoading ? (
        <div className={styles.loadingContent}>
          <span className={styles.loadingSpinner} />
          <span>Getting explanation...</span>
        </div>
      ) : (
        <button className={styles.button} onClick={handleExplain}>
          <ExplainIcon />
          <span>Explain this</span>
        </button>
      )}
    </div>
  );
}

function ExplainIcon(): JSX.Element {
  return (
    <svg
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <circle cx="12" cy="12" r="10" />
      <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3" />
      <line x1="12" y1="17" x2="12.01" y2="17" />
    </svg>
  );
}
