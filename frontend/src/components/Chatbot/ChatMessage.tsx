/**
 * ChatMessage component - displays a single chat message.
 */

import React from 'react';
import type { Message, Citation } from '../../services/chatbot';
import styles from './ChatMessage.module.css';

interface ChatMessageProps {
  message: Message;
  onFeedback?: (messageId: string, rating: number) => void;
}

export default function ChatMessage({
  message,
  onFeedback,
}: ChatMessageProps): JSX.Element {
  const isUser = message.role === 'user';
  const [showFeedback, setShowFeedback] = React.useState(false);

  return (
    <div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}
    >
      <div className={styles.avatar}>
        {isUser ? (
          <span className={styles.userAvatar}>U</span>
        ) : (
          <span className={styles.assistantAvatar}>AI</span>
        )}
      </div>
      <div className={styles.content}>
        <div className={styles.messageBody}>
          <MessageContent content={message.content} />
        </div>
        {message.citations && message.citations.length > 0 && (
          <CitationList citations={message.citations} />
        )}
        {!isUser && onFeedback && (
          <div className={styles.feedbackSection}>
            {showFeedback ? (
              <FeedbackButtons
                onFeedback={(rating) => {
                  onFeedback(message.id, rating);
                  setShowFeedback(false);
                }}
              />
            ) : (
              <button
                className={styles.feedbackToggle}
                onClick={() => setShowFeedback(true)}
                aria-label="Rate this response"
              >
                Rate response
              </button>
            )}
          </div>
        )}
      </div>
    </div>
  );
}

function MessageContent({ content }: { content: string }): JSX.Element {
  // Simple markdown-like rendering for code blocks
  const parts = content.split(/(```[\s\S]*?```)/g);

  return (
    <>
      {parts.map((part, index) => {
        if (part.startsWith('```')) {
          const match = part.match(/```(\w+)?\n?([\s\S]*?)```/);
          if (match) {
            const language = match[1] || 'text';
            const code = match[2].trim();
            return (
              <pre key={index} className={styles.codeBlock}>
                <code className={`language-${language}`}>{code}</code>
              </pre>
            );
          }
        }
        // Handle inline code
        const inlineCodeParts = part.split(/(`[^`]+`)/g);
        return (
          <span key={index}>
            {inlineCodeParts.map((inlinePart, i) => {
              if (inlinePart.startsWith('`') && inlinePart.endsWith('`')) {
                return (
                  <code key={i} className={styles.inlineCode}>
                    {inlinePart.slice(1, -1)}
                  </code>
                );
              }
              return <span key={i}>{inlinePart}</span>;
            })}
          </span>
        );
      })}
    </>
  );
}

function CitationList({ citations }: { citations: Citation[] }): JSX.Element {
  return (
    <div className={styles.citations}>
      <span className={styles.citationsLabel}>Sources:</span>
      <ul className={styles.citationsList}>
        {citations.map((citation, index) => (
          <li key={index} className={styles.citation}>
            <a
              href={`/docs/${citation.chapter_id}`}
              className={styles.citationLink}
            >
              {citation.title}
            </a>
            <span className={styles.relevanceScore}>
              {Math.round(citation.relevance_score * 100)}% match
            </span>
          </li>
        ))}
      </ul>
    </div>
  );
}

function FeedbackButtons({
  onFeedback,
}: {
  onFeedback: (rating: number) => void;
}): JSX.Element {
  return (
    <div className={styles.feedbackButtons}>
      <span className={styles.feedbackLabel}>Was this helpful?</span>
      <button
        className={styles.feedbackButton}
        onClick={() => onFeedback(5)}
        aria-label="Very helpful"
        title="Very helpful"
      >
        +
      </button>
      <button
        className={styles.feedbackButton}
        onClick={() => onFeedback(1)}
        aria-label="Not helpful"
        title="Not helpful"
      >
        -
      </button>
    </div>
  );
}
