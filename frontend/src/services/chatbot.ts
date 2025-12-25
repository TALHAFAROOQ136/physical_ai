/**
 * Chatbot API client service.
 * Handles communication with the RAG-powered chatbot backend.
 */

// Browser-compatible API URL configuration
// In production, this should be set via Docusaurus customFields
const getApiBaseUrl = (): string => {
  // Check if we're in a browser environment
  if (typeof window !== 'undefined') {
    // Try to get from window config (set by Docusaurus)
    const windowConfig = (window as { __DOCUSAURUS__?: { siteConfig?: { customFields?: { apiUrl?: string } } } }).__DOCUSAURUS__;
    if (windowConfig?.siteConfig?.customFields?.apiUrl) {
      return windowConfig.siteConfig.customFields.apiUrl;
    }
  }
  // Default fallback for development
  return 'http://localhost:8000/api/v1';
};

const API_BASE_URL = getApiBaseUrl();

// Types
export interface Conversation {
  id: string;
  chapter_context: string | null;
  message_count: number;
  created_at: string;
  updated_at: string;
}

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations: Citation[];
  created_at: string;
}

export interface Citation {
  chapter_id: string;
  title: string;
  relevance_score: number;
}

export interface ExplanationResponse {
  explanation: string;
  citations: Citation[];
}

export interface StreamEvent {
  type: 'citations' | 'chunk' | 'done';
  data: Citation[] | string | null;
}

// API Client class
export class ChatbotService {
  private baseUrl: string;

  constructor(baseUrl: string = API_BASE_URL) {
    this.baseUrl = baseUrl;
  }

  /**
   * Create a new conversation.
   */
  async createConversation(options?: {
    userId?: string;
    chapterContext?: string;
    context?: Record<string, unknown>;
  }): Promise<Conversation> {
    const response = await fetch(`${this.baseUrl}/chatbot/conversations`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        user_id: options?.userId,
        chapter_context: options?.chapterContext,
        context: options?.context,
      }),
    });

    if (!response.ok) {
      throw new Error(`Failed to create conversation: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Get conversation by ID.
   */
  async getConversation(conversationId: string): Promise<Conversation> {
    const response = await fetch(
      `${this.baseUrl}/chatbot/conversations/${conversationId}`
    );

    if (!response.ok) {
      throw new Error(`Failed to get conversation: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Get messages from a conversation.
   */
  async getMessages(conversationId: string, limit = 50): Promise<Message[]> {
    const response = await fetch(
      `${this.baseUrl}/chatbot/conversations/${conversationId}/messages?limit=${limit}`
    );

    if (!response.ok) {
      throw new Error(`Failed to get messages: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Send a message and get a non-streaming response.
   */
  async sendMessage(
    conversationId: string,
    message: string,
    chapterContext?: string
  ): Promise<{ message: Message; citations: Citation[] }> {
    const response = await fetch(
      `${this.baseUrl}/chatbot/conversations/${conversationId}/messages`,
      {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          chapter_context: chapterContext,
          stream: false,
        }),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to send message: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Send a message and stream the response using SSE.
   */
  async *sendMessageStream(
    conversationId: string,
    message: string,
    chapterContext?: string
  ): AsyncGenerator<StreamEvent, void, unknown> {
    const response = await fetch(
      `${this.baseUrl}/chatbot/conversations/${conversationId}/messages`,
      {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          chapter_context: chapterContext,
          stream: true,
        }),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to send message: ${response.statusText}`);
    }

    const reader = response.body?.getReader();
    if (!reader) {
      throw new Error('No response body');
    }

    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      let currentEvent = '';
      for (const line of lines) {
        if (line.startsWith('event: ')) {
          currentEvent = line.slice(7);
        } else if (line.startsWith('data: ')) {
          const data = line.slice(6);
          if (currentEvent === 'citations') {
            yield { type: 'citations', data: JSON.parse(data) };
          } else if (currentEvent === 'chunk') {
            yield { type: 'chunk', data };
          } else if (currentEvent === 'done') {
            yield { type: 'done', data: null };
          }
        }
      }
    }
  }

  /**
   * Explain selected text.
   */
  async explainText(
    text: string,
    chapterId?: string
  ): Promise<ExplanationResponse> {
    const response = await fetch(`${this.baseUrl}/chatbot/explain`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        text,
        chapter_id: chapterId,
      }),
    });

    if (!response.ok) {
      throw new Error(`Failed to explain text: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Submit feedback on a message.
   */
  async submitFeedback(
    messageId: string,
    rating: number,
    comment?: string
  ): Promise<{ success: boolean; message: string }> {
    const response = await fetch(`${this.baseUrl}/chatbot/feedback`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message_id: messageId,
        rating,
        comment,
      }),
    });

    if (!response.ok) {
      throw new Error(`Failed to submit feedback: ${response.statusText}`);
    }

    return response.json();
  }
}

// Default instance
export const chatbotService = new ChatbotService();

// React hook for chatbot state management
export interface ChatbotState {
  conversationId: string | null;
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  currentCitations: Citation[];
}

export const initialChatbotState: ChatbotState = {
  conversationId: null,
  messages: [],
  isLoading: false,
  error: null,
  currentCitations: [],
};

export type ChatbotAction =
  | { type: 'SET_CONVERSATION_ID'; payload: string }
  | { type: 'ADD_MESSAGE'; payload: Message }
  | { type: 'UPDATE_LAST_MESSAGE'; payload: string }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'SET_CITATIONS'; payload: Citation[] }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'SET_MESSAGES'; payload: Message[] };

export function chatbotReducer(
  state: ChatbotState,
  action: ChatbotAction
): ChatbotState {
  switch (action.type) {
    case 'SET_CONVERSATION_ID':
      return { ...state, conversationId: action.payload };
    case 'ADD_MESSAGE':
      return { ...state, messages: [...state.messages, action.payload] };
    case 'UPDATE_LAST_MESSAGE':
      const messages = [...state.messages];
      if (messages.length > 0) {
        const lastMessage = messages[messages.length - 1];
        messages[messages.length - 1] = {
          ...lastMessage,
          content: lastMessage.content + action.payload,
        };
      }
      return { ...state, messages };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload };
    case 'SET_CITATIONS':
      return { ...state, currentCitations: action.payload };
    case 'CLEAR_MESSAGES':
      return { ...state, messages: [], currentCitations: [] };
    case 'SET_MESSAGES':
      return { ...state, messages: action.payload };
    default:
      return state;
  }
}
