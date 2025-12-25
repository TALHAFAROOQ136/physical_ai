/**
 * Authentication context provider for React components.
 *
 * DISABLED: This is a mock implementation that doesn't require a backend.
 * All auth operations return mock data or no-op.
 * To re-enable real auth, restore the original implementation from git history.
 */

import React, {
  createContext,
  useContext,
  ReactNode,
} from 'react';

// Types (kept for API compatibility)
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

interface AuthContextType {
  // State - always null/false in disabled mode
  user: User | null;
  loading: boolean;
  initialized: boolean;

  // Auth methods - all no-op in disabled mode
  signup: (
    email: string,
    password: string,
    displayName?: string
  ) => Promise<AuthResponse>;
  signin: (email: string, password: string) => Promise<AuthResponse>;
  signout: () => Promise<void>;
  signoutAll: () => Promise<void>;
  forgotPassword: (email: string) => Promise<void>;
  resetPassword: (token: string, newPassword: string) => Promise<void>;

  // Profile methods
  getProfile: () => Promise<UserProfile>;
  updateProfile: (displayName?: string | null) => Promise<UserProfile>;
  deleteAccount: (confirmPassword: string) => Promise<void>;

  // Background methods
  getBackground: () => Promise<BackgroundAssessment>;
  updateBackground: (
    background: BackgroundAssessment
  ) => Promise<BackgroundAssessment>;

  // Preferences methods
  updatePreferences: (preferences: UserPreferences) => Promise<UserPreferences>;

  // Utility
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

// Mock user for disabled auth
const mockUser: User = {
  id: 'guest',
  email: 'guest@example.com',
  displayName: 'Guest User',
  emailVerified: false,
  createdAt: new Date().toISOString(),
};

const mockProfile: UserProfile = {
  id: 'guest',
  email: 'guest@example.com',
  displayName: 'Guest User',
  emailVerified: false,
  background: {},
  languagePreference: 'en',
  createdAt: new Date().toISOString(),
  updatedAt: new Date().toISOString(),
};

// No-op async function
const noop = async () => {};

// Disabled auth error
const authDisabledError = () => {
  console.warn('Auth is disabled. Backend not configured.');
  throw new Error('Authentication is currently disabled.');
};

export function AuthProvider({ children }: AuthProviderProps): JSX.Element {
  // Disabled mode: no user, not loading, initialized
  const value: AuthContextType = {
    user: null,
    loading: false,
    initialized: true,

    // All auth methods throw or no-op
    signup: authDisabledError,
    signin: authDisabledError,
    signout: noop,
    signoutAll: noop,
    forgotPassword: noop,
    resetPassword: noop,

    // Profile methods return mock data
    getProfile: async () => mockProfile,
    updateProfile: async () => mockProfile,
    deleteAccount: noop,

    // Background methods return empty
    getBackground: async () => ({}),
    updateBackground: async (bg) => bg,

    // Preferences
    updatePreferences: async (prefs) => prefs,

    // Utility
    refreshSession: noop,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * Hook to access auth context.
 */
export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

/**
 * Hook to require authentication.
 * In disabled mode, always returns null and doesn't redirect.
 */
export function useRequireAuth(): User | null {
  const { user } = useAuth();
  // In disabled mode, just return null without redirecting
  return user;
}

export { AuthContext };
