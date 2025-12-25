/**
 * ChatbotWidget - combines the floating button and chat panel.
 * This is the main export for integrating the chatbot into the theme.
 */

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import Chatbot from './Chatbot';
import ChatbotButton from './ChatbotButton';

interface ChatbotWidgetProps {
  defaultOpen?: boolean;
}

export default function ChatbotWidget({
  defaultOpen = false,
}: ChatbotWidgetProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  const location = useLocation();

  // Extract chapter context from URL
  const chapterContext = extractChapterFromPath(location.pathname);

  // Close chatbot on ESC key
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isOpen]);

  return (
    <>
      <ChatbotButton isOpen={isOpen} onClick={() => setIsOpen(true)} />
      <Chatbot
        isOpen={isOpen}
        onClose={() => setIsOpen(false)}
        chapterContext={chapterContext}
      />
    </>
  );
}

/**
 * Extract chapter ID from the URL path.
 * Example: /docs/module-1-ros2/intro -> module-1-ros2/intro
 */
function extractChapterFromPath(pathname: string): string | undefined {
  const match = pathname.match(/^\/docs\/(.+)/);
  if (match) {
    return match[1].replace(/\/$/, '');
  }
  return undefined;
}
