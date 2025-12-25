/**
 * API client service for backend communication.
 *
 * DISABLED MODE: When backend is not available, all requests will fail gracefully.
 * Services should handle errors and provide mock data when needed.
 */

// Browser-compatible API URL configuration
const getApiBaseUrl = (): string => {
  // Check if we're in a browser environment with Docusaurus config
  if (typeof window !== 'undefined') {
    const windowConfig = (window as { __DOCUSAURUS__?: { siteConfig?: { customFields?: { apiUrl?: string } } } }).__DOCUSAURUS__;
    if (windowConfig?.siteConfig?.customFields?.apiUrl) {
      return windowConfig.siteConfig.customFields.apiUrl;
    }
  }
  // Default fallback for development
  return 'http://localhost:8000/v1';
};

const API_BASE_URL = getApiBaseUrl();

// Flag to track if backend is available
let backendAvailable: boolean | null = null;

interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

class ApiClient {
  private baseUrl: string;

  constructor(baseUrl: string = API_BASE_URL) {
    this.baseUrl = baseUrl;
  }

  /**
   * Check if backend is available (cached result).
   */
  async isBackendAvailable(): Promise<boolean> {
    if (backendAvailable !== null) {
      return backendAvailable;
    }

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 2000);

      await fetch(`${this.baseUrl}/health`, {
        method: 'GET',
        signal: controller.signal,
      });

      clearTimeout(timeoutId);
      backendAvailable = true;
    } catch {
      backendAvailable = false;
    }

    return backendAvailable;
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.baseUrl}${endpoint}`;

    const defaultHeaders: HeadersInit = {
      'Content-Type': 'application/json',
    };

    const config: RequestInit = {
      ...options,
      headers: {
        ...defaultHeaders,
        ...options.headers,
      },
      credentials: 'include', // Include cookies for session auth
    };

    const response = await fetch(url, config);

    if (!response.ok) {
      const error: ApiError = await response.json().catch(() => ({
        code: 'UNKNOWN_ERROR',
        message: `HTTP error ${response.status}`,
      }));
      throw new ApiClientError(error.code, error.message, response.status, error.details);
    }

    // Handle empty responses (204 No Content)
    if (response.status === 204) {
      return undefined as T;
    }

    return response.json();
  }

  async get<T>(endpoint: string, options?: RequestInit): Promise<T> {
    return this.request<T>(endpoint, { ...options, method: 'GET' });
  }

  async post<T>(endpoint: string, data?: unknown, options?: RequestInit): Promise<T> {
    return this.request<T>(endpoint, {
      ...options,
      method: 'POST',
      body: data ? JSON.stringify(data) : undefined,
    });
  }

  async put<T>(endpoint: string, data?: unknown, options?: RequestInit): Promise<T> {
    return this.request<T>(endpoint, {
      ...options,
      method: 'PUT',
      body: data ? JSON.stringify(data) : undefined,
    });
  }

  async patch<T>(endpoint: string, data?: unknown, options?: RequestInit): Promise<T> {
    return this.request<T>(endpoint, {
      ...options,
      method: 'PATCH',
      body: data ? JSON.stringify(data) : undefined,
    });
  }

  async delete<T>(endpoint: string, options?: RequestInit): Promise<T> {
    return this.request<T>(endpoint, { ...options, method: 'DELETE' });
  }

  /**
   * Create a streaming request for SSE (Server-Sent Events).
   */
  async stream(
    endpoint: string,
    data?: unknown,
    onMessage?: (message: string) => void
  ): Promise<void> {
    const url = `${this.baseUrl}${endpoint}`;

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'text/event-stream',
      },
      body: data ? JSON.stringify(data) : undefined,
      credentials: 'include',
    });

    if (!response.ok) {
      throw new ApiClientError('STREAM_ERROR', 'Failed to start stream', response.status);
    }

    const reader = response.body?.getReader();
    if (!reader) {
      throw new ApiClientError('STREAM_ERROR', 'No response body', 500);
    }

    const decoder = new TextDecoder();

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      const text = decoder.decode(value, { stream: true });
      const lines = text.split('\n');

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const data = line.slice(6);
          if (data !== '[DONE]') {
            onMessage?.(data);
          }
        }
      }
    }
  }
}

class ApiClientError extends Error {
  code: string;
  statusCode: number;
  details?: Record<string, unknown>;

  constructor(
    code: string,
    message: string,
    statusCode: number,
    details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ApiClientError';
    this.code = code;
    this.statusCode = statusCode;
    this.details = details;
  }
}

// Export singleton instance
export const api = new ApiClient();

// Export types
export type { ApiError };
export { ApiClient, ApiClientError };
