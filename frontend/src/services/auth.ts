/**
 * Authentication service for user management.
 *
 * DISABLED MODE: Returns mock data when backend is unavailable.
 * Authentication is not available without a backend.
 */

import { api, ApiClientError } from './api';

// Types
export interface User {
  id: string;
  email: string;
  displayName: string | null;
  emailVerified: boolean;
  createdAt: string;
}

export interface Session {
  id: string;
  expiresAt: string;
}

export interface AuthResponse {
  user: User;
  session: Session;
}

export interface SessionInfo {
  user: User;
  sessionId: string;
  expiresAt: string;
}

export interface BackgroundAssessment {
  pythonLevel?: 'beginner' | 'intermediate' | 'advanced';
  rosExperience?: 'none' | 'basic' | 'experienced';
  hardwareAccess?: 'simulation_only' | 'jetson' | 'full_kit';
  learningGoals?: string[];
  completedAssessment?: boolean;
}

export interface UserProfile {
  id: string;
  email: string;
  displayName: string | null;
  emailVerified: boolean;
  background: BackgroundAssessment;
  languagePreference: string;
  createdAt: string;
  updatedAt: string;
}

export interface UserPreferences {
  languagePreference?: 'en' | 'ur';
  emailNotifications?: boolean;
  darkMode?: boolean | null;
}

// Auth state management
type AuthStateListener = (user: User | null) => void;

// Error for disabled auth
class AuthDisabledError extends Error {
  constructor() {
    super('Authentication is disabled. Backend server is not available.');
    this.name = 'AuthDisabledError';
  }
}

class AuthService {
  private user: User | null = null;
  private listeners: Set<AuthStateListener> = new Set();
  private initialized = false;
  private backendDisabled = false;

  /**
   * Check if backend is available.
   */
  private async checkBackend(): Promise<boolean> {
    if (this.backendDisabled) return false;

    try {
      const available = await api.isBackendAvailable();
      if (!available) {
        this.backendDisabled = true;
        console.info('Authentication disabled: backend not available');
      }
      return available;
    } catch {
      this.backendDisabled = true;
      return false;
    }
  }

  /**
   * Subscribe to auth state changes.
   */
  subscribe(listener: AuthStateListener): () => void {
    this.listeners.add(listener);
    // Immediately call with current state
    listener(this.user);
    return () => this.listeners.delete(listener);
  }

  private notifyListeners(): void {
    this.listeners.forEach((listener) => listener(this.user));
  }

  /**
   * Get current user (from memory).
   */
  getCurrentUser(): User | null {
    return this.user;
  }

  /**
   * Check if user is authenticated.
   */
  isAuthenticated(): boolean {
    return this.user !== null;
  }

  /**
   * Initialize auth state by fetching current session.
   */
  async initialize(): Promise<User | null> {
    if (this.initialized) {
      return this.user;
    }

    // Check if backend is available
    if (!(await this.checkBackend())) {
      this.user = null;
      this.initialized = true;
      this.notifyListeners();
      return null;
    }

    try {
      const session = await this.getSession();
      this.user = session?.user || null;
    } catch {
      // Not authenticated
      this.user = null;
    }

    this.initialized = true;
    this.notifyListeners();
    return this.user;
  }

  /**
   * Sign up a new user.
   */
  async signup(
    email: string,
    password: string,
    displayName?: string
  ): Promise<AuthResponse> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    const response = await api.post<AuthResponse>('/auth/signup', {
      email,
      password,
      display_name: displayName,
    });

    this.user = response.user;
    this.notifyListeners();
    return response;
  }

  /**
   * Sign in with email and password.
   */
  async signin(email: string, password: string): Promise<AuthResponse> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    const response = await api.post<AuthResponse>('/auth/signin', {
      email,
      password,
    });

    this.user = response.user;
    this.notifyListeners();
    return response;
  }

  /**
   * Sign out current session.
   */
  async signout(): Promise<void> {
    if (!(await this.checkBackend())) {
      this.user = null;
      this.notifyListeners();
      return;
    }

    try {
      await api.post('/auth/signout');
    } finally {
      this.user = null;
      this.notifyListeners();
    }
  }

  /**
   * Sign out from all devices.
   */
  async signoutAll(): Promise<void> {
    if (!(await this.checkBackend())) {
      this.user = null;
      this.notifyListeners();
      return;
    }

    try {
      await api.post('/auth/signout-all');
    } finally {
      this.user = null;
      this.notifyListeners();
    }
  }

  /**
   * Request password reset email.
   */
  async forgotPassword(email: string): Promise<void> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    await api.post('/auth/forgot-password', { email });
  }

  /**
   * Reset password with token.
   */
  async resetPassword(token: string, newPassword: string): Promise<void> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    await api.post('/auth/reset-password', {
      token,
      new_password: newPassword,
    });
  }

  /**
   * Get current session info.
   */
  async getSession(): Promise<SessionInfo | null> {
    if (!(await this.checkBackend())) {
      return null;
    }

    try {
      return await api.get<SessionInfo>('/auth/session');
    } catch (error) {
      if (error instanceof ApiClientError && error.statusCode === 401) {
        return null;
      }
      throw error;
    }
  }

  /**
   * Get current user profile.
   */
  async getProfile(): Promise<UserProfile> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    return api.get<UserProfile>('/users/me');
  }

  /**
   * Update user profile.
   */
  async updateProfile(displayName?: string | null): Promise<UserProfile> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    const response = await api.patch<UserProfile>('/users/me', {
      display_name: displayName,
    });

    // Update cached user
    if (this.user) {
      this.user.displayName = response.displayName;
      this.notifyListeners();
    }

    return response;
  }

  /**
   * Delete user account.
   */
  async deleteAccount(confirmPassword: string): Promise<void> {
    if (!(await this.checkBackend())) {
      throw new AuthDisabledError();
    }

    await api.delete('/users/me', {
      body: JSON.stringify({ confirm_password: confirmPassword }),
    });
    this.user = null;
    this.notifyListeners();
  }

  /**
   * Get background assessment.
   */
  async getBackground(): Promise<BackgroundAssessment> {
    if (!(await this.checkBackend())) {
      return {};
    }

    try {
      return await api.get<BackgroundAssessment>('/users/me/background');
    } catch {
      return {};
    }
  }

  /**
   * Update background assessment.
   */
  async updateBackground(
    background: BackgroundAssessment
  ): Promise<BackgroundAssessment> {
    if (!(await this.checkBackend())) {
      return background;
    }

    return api.put<BackgroundAssessment>('/users/me/background', {
      python_level: background.pythonLevel,
      ros_experience: background.rosExperience,
      hardware_access: background.hardwareAccess,
      learning_goals: background.learningGoals,
      completed_assessment: background.completedAssessment,
    });
  }

  /**
   * Update user preferences.
   */
  async updatePreferences(
    preferences: UserPreferences
  ): Promise<UserPreferences> {
    if (!(await this.checkBackend())) {
      return preferences;
    }

    return api.patch<UserPreferences>('/users/me/preferences', {
      language_preference: preferences.languagePreference,
      email_notifications: preferences.emailNotifications,
      dark_mode: preferences.darkMode,
    });
  }
}

// Export singleton instance
export const authService = new AuthService();

// Export class for testing
export { AuthService, AuthDisabledError };
