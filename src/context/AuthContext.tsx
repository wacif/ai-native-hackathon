/**
 * Authentication Context for React
 * Provides auth state management similar to better-auth's useSession hook
 */
import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  ReactNode,
} from 'react';

// API base URL - hardcoded for now, can be configured via docusaurus.config.ts customFields
const API_BASE_URL = 'http://localhost:8000';

// Types
export interface User {
  user_id: string;
  username: string;
  email: string;
  software_background: string | null;
  hardware_background: string | null;
  personalization_preferences: Record<string, any> | null;
  selected_language: string;
  created_at: string;
  updated_at: string;
}

export interface AuthTokens {
  access_token: string;
  refresh_token: string;
  expires_in: number;
}

interface SignupData {
  username: string;
  email: string;
  password: string;
  software_background?: string;
  hardware_background?: string;
}

interface SigninData {
  email: string;
  password: string;
}

interface UpdateProfileData {
  username?: string;
  software_background?: string;
  hardware_background?: string;
  selected_language?: string;
  personalization_preferences?: Record<string, any>;
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
  signUp: (data: SignupData) => Promise<{ success: boolean; error?: string }>;
  signIn: (data: SigninData) => Promise<{ success: boolean; error?: string }>;
  signOut: () => void;
  updateProfile: (data: UpdateProfileData) => Promise<{ success: boolean; error?: string }>;
  refreshSession: () => Promise<boolean>;
  clearError: () => void;
}

// Create context
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Storage keys
const TOKEN_KEY = 'auth_tokens';
const USER_KEY = 'auth_user';

// Helper functions for token storage
const getStoredTokens = (): AuthTokens | null => {
  if (typeof window === 'undefined') return null;
  const stored = localStorage.getItem(TOKEN_KEY);
  return stored ? JSON.parse(stored) : null;
};

const setStoredTokens = (tokens: AuthTokens | null) => {
  if (typeof window === 'undefined') return;
  if (tokens) {
    localStorage.setItem(TOKEN_KEY, JSON.stringify(tokens));
  } else {
    localStorage.removeItem(TOKEN_KEY);
  }
};

const getStoredUser = (): User | null => {
  if (typeof window === 'undefined') return null;
  const stored = localStorage.getItem(USER_KEY);
  return stored ? JSON.parse(stored) : null;
};

const setStoredUser = (user: User | null) => {
  if (typeof window === 'undefined') return;
  if (user) {
    localStorage.setItem(USER_KEY, JSON.stringify(user));
  } else {
    localStorage.removeItem(USER_KEY);
  }
};

// Provider component
export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const isAuthenticated = !!user;

  // Clear error
  const clearError = useCallback(() => setError(null), []);

  // Sign out
  const signOut = useCallback(() => {
    setStoredTokens(null);
    setStoredUser(null);
    setUser(null);
    setError(null);
  }, []);

  // Fetch user profile
  const fetchProfile = useCallback(async (accessToken: string): Promise<User | null> => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/me`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (!response.ok) {
        if (response.status === 401) {
          return null; // Token expired
        }
        throw new Error('Failed to fetch profile');
      }

      return await response.json();
    } catch (err) {
      console.error('Error fetching profile:', err);
      return null;
    }
  }, []);

  // Refresh session
  const refreshSession = useCallback(async (): Promise<boolean> => {
    const tokens = getStoredTokens();
    if (!tokens?.refresh_token) {
      signOut();
      return false;
    }

    try {
      const response = await fetch(`${API_BASE_URL}/auth/refresh`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ refresh_token: tokens.refresh_token }),
      });

      if (!response.ok) {
        signOut();
        return false;
      }

      const data = await response.json();
      const newTokens: AuthTokens = {
        ...tokens,
        access_token: data.access_token,
        expires_in: data.expires_in,
      };
      setStoredTokens(newTokens);

      const profile = await fetchProfile(data.access_token);
      if (profile) {
        setUser(profile);
        setStoredUser(profile);
        return true;
      }

      signOut();
      return false;
    } catch (err) {
      console.error('Error refreshing session:', err);
      signOut();
      return false;
    }
  }, [fetchProfile, signOut]);

  // Initialize auth state on mount
  useEffect(() => {
    const initAuth = async () => {
      const tokens = getStoredTokens();
      const storedUser = getStoredUser();

      if (tokens?.access_token) {
        // Try to fetch fresh profile
        const profile = await fetchProfile(tokens.access_token);
        if (profile) {
          setUser(profile);
          setStoredUser(profile);
        } else {
          // Token might be expired, try refresh
          const refreshed = await refreshSession();
          if (!refreshed && storedUser) {
            // Use stored user as fallback during offline/error
            setUser(storedUser);
          }
        }
      }

      setIsLoading(false);
    };

    initAuth();
  }, [fetchProfile, refreshSession]);

  // Sign up
  const signUp = useCallback(async (data: SignupData) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data),
      });

      const result = await response.json();

      if (!response.ok) {
        const errorMsg = result.detail || 'Signup failed';
        setError(errorMsg);
        setIsLoading(false);
        return { success: false, error: errorMsg };
      }

      setIsLoading(false);
      return { success: true };
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Signup failed';
      setError(errorMsg);
      setIsLoading(false);
      return { success: false, error: errorMsg };
    }
  }, []);

  // Sign in
  const signIn = useCallback(async (data: SigninData) => {
    setIsLoading(true);
    setError(null);

    try {
      // OAuth2 password flow uses form data
      const formData = new URLSearchParams();
      formData.append('username', data.email); // OAuth2 uses 'username' field
      formData.append('password', data.password);

      const response = await fetch(`${API_BASE_URL}/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: formData,
      });

      const result = await response.json();

      if (!response.ok) {
        const errorMsg = result.detail || 'Sign in failed';
        setError(errorMsg);
        setIsLoading(false);
        return { success: false, error: errorMsg };
      }

      // Store tokens
      const tokens: AuthTokens = {
        access_token: result.access_token,
        refresh_token: result.refresh_token,
        expires_in: result.expires_in,
      };
      setStoredTokens(tokens);

      // Fetch user profile
      const profile = await fetchProfile(result.access_token);
      if (profile) {
        setUser(profile);
        setStoredUser(profile);
        setIsLoading(false);
        return { success: true };
      }

      setError('Failed to fetch profile');
      setIsLoading(false);
      return { success: false, error: 'Failed to fetch profile' };
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Sign in failed';
      setError(errorMsg);
      setIsLoading(false);
      return { success: false, error: errorMsg };
    }
  }, [fetchProfile]);

  // Update profile
  const updateProfile = useCallback(async (data: UpdateProfileData) => {
    const tokens = getStoredTokens();
    if (!tokens?.access_token) {
      return { success: false, error: 'Not authenticated' };
    }

    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/auth/me`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${tokens.access_token}`,
        },
        body: JSON.stringify(data),
      });

      const result = await response.json();

      if (!response.ok) {
        const errorMsg = result.detail || 'Update failed';
        setError(errorMsg);
        return { success: false, error: errorMsg };
      }

      // Update stored user
      if (result.user) {
        setUser(result.user);
        setStoredUser(result.user);
      }

      return { success: true };
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Update failed';
      setError(errorMsg);
      return { success: false, error: errorMsg };
    }
  }, []);

  const value: AuthContextType = {
    user,
    isAuthenticated,
    isLoading,
    error,
    signUp,
    signIn,
    signOut,
    updateProfile,
    refreshSession,
    clearError,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

// Hook to use auth context
export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

// Convenience hook similar to better-auth's useSession
export function useSession() {
  const { user, isLoading, isAuthenticated, error, refreshSession } = useAuth();
  return {
    data: user ? { user } : null,
    isPending: isLoading,
    isAuthenticated,
    error,
    refetch: refreshSession,
  };
}

export default AuthContext;

