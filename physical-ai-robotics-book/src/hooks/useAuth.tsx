import { useState, useEffect, createContext, useContext, ReactNode } from 'react';
import API_CONFIG from '../config/api';

interface User {
  id: number;
  email: string;
  full_name?: string;
  software_background_level?: string;
  hardware_background_level?: string;
  preferred_languages?: string;
  learning_goals?: string;
  is_active: boolean;
  is_admin: boolean;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => void;
  register: (userData: {
    email: string;
    password: string;
    full_name?: string;
    software_background_level?: string;
    hardware_background_level?: string;
    preferred_languages?: string;
    learning_goals?: string;
  }) => Promise<void>;
  isAuthenticated: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check if user is already logged in on initial load
    const token = localStorage.getItem('access_token');
    if (token) {
      fetchUser();
    } else {
      setLoading(false);
    }
  }, []);

  const fetchUser = async () => {
    try {
      const token = localStorage.getItem('access_token');
      if (!token) {
        setLoading(false);
        return;
      }

      const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/me`, {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });

      if (response.ok) {
        const userData = await response.json();
        setUser(userData);
      } else {
        console.error(`Failed to fetch user: ${response.status} ${response.statusText}`);
        // Token might be expired, clear it
        localStorage.removeItem('access_token');
        localStorage.removeItem('refresh_token');
      }
    } catch (error) {
      console.error('Error fetching user:', error);
      localStorage.removeItem('access_token');
      localStorage.removeItem('refresh_token');
    } finally {
      setLoading(false);
    }
  };

  const login = async (email: string, password: string) => {
    setLoading(true);
    try {
      const body = new URLSearchParams();
      body.append('username', email);
      body.append('password', password);

      const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: body
      });

      if (!response.ok) {
        let errorData;
        try {
          errorData = await response.json();
        } catch (e) {
          // If response is not JSON, create a generic error
          errorData = { detail: `HTTP Error: ${response.status} - ${response.statusText}` };
        }
        throw new Error(errorData.detail || `Login failed: ${response.status}`);
      }

      const data = await response.json();

      // Store tokens
      localStorage.setItem('access_token', data.access_token);
      localStorage.setItem('refresh_token', data.refresh_token);

      // Set user
      setUser({
        id: data.user_id,
        email: data.email,
        full_name: data.full_name,
        software_background_level: data.software_background_level,
        hardware_background_level: data.hardware_background_level,
        preferred_languages: data.preferred_languages,
        learning_goals: data.learning_goals,
        is_active: data.is_active,
        is_admin: data.is_admin
      });
    } catch (error) {
      setLoading(false);
      if (error instanceof Error) {
        throw error;
      } else if (typeof error === 'string') {
        throw new Error(error);
      } else {
        throw new Error('An unknown error occurred');
      }
    }
  };

  const register = async (userData: {
    email: string;
    password: string;
    full_name?: string;
    software_background_level?: string;
    hardware_background_level?: string;
    preferred_languages?: string;
    learning_goals?: string;
  }) => {
    setLoading(true);
    try {
      // Build form data object, only including non-empty optional fields
      const formDataObj: Record<string, string> = {
        email: userData.email,
        password: userData.password
      };

      // Add optional fields only if they have values
      if (userData.full_name) formDataObj.full_name = userData.full_name;
      if (userData.software_background_level) formDataObj.software_background_level = userData.software_background_level;
      if (userData.hardware_background_level) formDataObj.hardware_background_level = userData.hardware_background_level;
      if (userData.preferred_languages) formDataObj.preferred_languages = userData.preferred_languages;
      if (userData.learning_goals) formDataObj.learning_goals = userData.learning_goals;

      const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams(formDataObj)
      });

      if (!response.ok) {
        let errorData;
        try {
          errorData = await response.json();
        } catch (e) {
          // If response is not JSON, create a generic error
          errorData = { detail: `HTTP Error: ${response.status} - ${response.statusText}` };
        }
        throw new Error(errorData.detail || `Registration failed: ${response.status}`);
      }

      const data = await response.json();

      // Store tokens
      localStorage.setItem('access_token', data.access_token);
      localStorage.setItem('refresh_token', data.refresh_token);

      // Set user
      setUser({
        id: data.user_id,
        email: data.email,
        full_name: data.full_name,
        software_background_level: data.software_background_level,
        hardware_background_level: data.hardware_background_level,
        preferred_languages: data.preferred_languages,
        learning_goals: data.learning_goals,
        is_active: data.is_active,
        is_admin: data.is_admin
      });
    } catch (error) {
      setLoading(false);
      if (error instanceof Error) {
        throw error;
      } else if (typeof error === 'string') {
        throw new Error(error);
      } else {
        throw new Error('An unknown error occurred');
      }
    }
  };

  const logout = () => {
    localStorage.removeItem('access_token');
    localStorage.removeItem('refresh_token');
    setUser(null);
  };

  const value: AuthContextType = {
    user: user,
    loading: loading,
    login: login,
    logout: logout,
    register: register,
    isAuthenticated: !!user
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType | null => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    // In development, warn about the missing provider
    if (process.env.NODE_ENV !== 'production') {
      console.warn('useAuth used outside of AuthProvider - returning null');
    }
    return null;
  }
  return context;
};