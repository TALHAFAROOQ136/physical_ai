/**
 * Chatbot container component - main chat widget with RAG support.
 */

import React, { useReducer, useEffect, useRef, useCallback } from 'react';
import {
  chatbotService,
  chatbotReducer,
  initialChatbotState,
  type Message,
} from '../../services/chatbot';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import styles from './Chatbot.module.css';

interface ChatbotProps {
  chapterContext?: string;
  isOpen: boolean;
  onClose: () => void;
}

export default function Chatbot({
  chapterContext,
  isOpen,
  onClose,
}: ChatbotProps): JSX.Element | null {
  const [state, dispatch] = useReducer(chatbotReducer, initialChatbotState);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const initializedRef = useRef(false);

  // Initialize conversation on mount
  useEffect(() => {
    if (isOpen && !state.conversationId && !initializedRef.current) {
      initializedRef.current = true;
      initConversation();
    }
  }, [isOpen, state.conversationId]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [state.messages]);

  const initConversation = async () => {
    try {
      dispatch({ type: 'SET_LOADING', payload: true });
      const conversation = await chatbotService.createConversation({
        chapterContext,
      });
      dispatch({ type: 'SET_CONVERSATION_ID', payload: conversation.id });
      dispatch({ type: 'SET_ERROR', payload: null });
    } catch (error) {
      console.error('Failed to initialize conversation:', error);
      dispatch({
        type: 'SET_ERROR',
        payload: 'Failed to connect to chatbot. Please try again.',
      });
    } finally {
      dispatch({ type: 'SET_LOADING', payload: false });
    }
  };

  const handleSendMessage = useCallback(
    async (content: string) => {
      if (!state.conversationId || state.isLoading) return;

      // Add user message immediately
      const userMessage: Message = {
        id: `temp-${Date.now()}`,
        role: 'user',
        content,
        citations: [],
        created_at: new Date().toISOString(),
      };
      dispatch({ type: 'ADD_MESSAGE', payload: userMessage });
      dispatch({ type: 'SET_LOADING', payload: true });
      dispatch({ type: 'SET_ERROR', payload: null });

      // Add placeholder for assistant response
      const assistantMessage: Message = {
        id: `temp-assistant-${Date.now()}`,
        role: 'assistant',
        content: '',
        citations: [],
        created_at: new Date().toISOString(),
      };
      dispatch({ type: 'ADD_MESSAGE', payload: assistantMessage });

      try {
        // Stream the response
        for await (const event of chatbotService.sendMessageStream(
          state.conversationId,
          content,
          chapterContext
        )) {
          if (event.type === 'citations') {
            dispatch({ type: 'SET_CITATIONS', payload: event.data as any });
          } else if (event.type === 'chunk') {
            dispatch({
              type: 'UPDATE_LAST_MESSAGE',
              payload: event.data as string,
            });
          } else if (event.type === 'done') {
            // Streaming complete
          }
        }
      } catch (error) {
        console.error('Failed to send message:', error);
        dispatch({
          type: 'SET_ERROR',
          payload: 'Failed to send message. Please try again.',
        });
        // Remove the empty assistant message on error
        dispatch({
          type: 'SET_MESSAGES',
          payload: state.messages.slice(0, -1),
        });
      } finally {
        dispatch({ type: 'SET_LOADING', payload: false });
      }
    },
    [state.conversationId, state.isLoading, state.messages, chapterContext]
  );

  const handleFeedback = useCallback(
    async (messageId: string, rating: number) => {
      try {
        await chatbotService.submitFeedback(messageId, rating);
      } catch (error) {
        console.error('Failed to submit feedback:', error);
      }
    },
    []
  );

  const handleNewConversation = useCallback(() => {
    dispatch({ type: 'SET_CONVERSATION_ID', payload: null as any });
    dispatch({ type: 'CLEAR_MESSAGES' });
    initializedRef.current = false;
    initConversation();
  }, []);

  if (!isOpen) return null;

  return (
    <div className={styles.chatbot}>
      <div className={styles.header}>
        <div className={styles.headerContent}>
          <h3 className={styles.title}>AI Teaching Assistant</h3>
          <p className={styles.subtitle}>Ask questions about the textbook</p>
        </div>
        <div className={styles.headerActions}>
          <button
            className={styles.newChatButton}
            onClick={handleNewConversation}
            title="Start new conversation"
            aria-label="Start new conversation"
          >
            <NewChatIcon />
          </button>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
          >
            <CloseIcon />
          </button>
        </div>
      </div>

      <div className={styles.messages}>
        {state.messages.length === 0 ? (
          <WelcomeMessage chapterContext={chapterContext} />
        ) : (
          <>
            {state.messages.map((message, index) => (
              <ChatMessage
                key={message.id || index}
                message={message}
                onFeedback={
                  message.role === 'assistant' && !message.id.startsWith('temp')
                    ? handleFeedback
                    : undefined
                }
              />
            ))}
          </>
        )}
        {state.error && <ErrorMessage message={state.error} />}
        <div ref={messagesEndRef} />
      </div>

      <ChatInput onSend={handleSendMessage} isLoading={state.isLoading} />
    </div>
  );
}

function WelcomeMessage({
  chapterContext,
}: {
  chapterContext?: string;
}): JSX.Element {
  return (
    <div className={styles.welcome}>
      <div className={styles.welcomeIcon}>AI</div>
      <h4 className={styles.welcomeTitle}>
        Welcome to the AI Teaching Assistant!
      </h4>
      <p className={styles.welcomeText}>
        I can help you understand concepts from the Physical AI & Humanoid
        Robotics textbook. Ask me about:
      </p>
      <ul className={styles.welcomeList}>
        <li>ROS 2 concepts and implementation</li>
        <li>Gazebo and Unity simulation setup</li>
        <li>NVIDIA Isaac Sim and perception</li>
        <li>Vision-Language-Action (VLA) models</li>
        <li>Humanoid robot development</li>
      </ul>
      {chapterContext && (
        <p className={styles.contextNote}>
          Currently viewing: <strong>{chapterContext}</strong>
        </p>
      )}
    </div>
  );
}

function ErrorMessage({ message }: { message: string }): JSX.Element {
  return (
    <div className={styles.error}>
      <span className={styles.errorIcon}>!</span>
      {message}
    </div>
  );
}

function CloseIcon(): JSX.Element {
  return (
    <svg
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <line x1="18" y1="6" x2="6" y2="18" />
      <line x1="6" y1="6" x2="18" y2="18" />
    </svg>
  );
}

function NewChatIcon(): JSX.Element {
  return (
    <svg
      width="18"
      height="18"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <line x1="12" y1="5" x2="12" y2="19" />
      <line x1="5" y1="12" x2="19" y2="12" />
    </svg>
  );
}
