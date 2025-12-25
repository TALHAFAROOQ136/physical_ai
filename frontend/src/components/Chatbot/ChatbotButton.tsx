/**
 * ChatbotButton component - floating action button to open the chatbot.
 */

import React from 'react';
import styles from './ChatbotButton.module.css';

interface ChatbotButtonProps {
  isOpen: boolean;
  onClick: () => void;
}

export default function ChatbotButton({
  isOpen,
  onClick,
}: ChatbotButtonProps): JSX.Element {
  return (
    <button
      className={`${styles.button} ${isOpen ? styles.hidden : ''}`}
      onClick={onClick}
      aria-label="Open chat assistant"
      title="Ask AI Assistant"
    >
      <ChatIcon />
      <span className={styles.label}>Ask AI</span>
    </button>
  );
}

function ChatIcon(): JSX.Element {
  return (
    <svg
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
    </svg>
  );
}
