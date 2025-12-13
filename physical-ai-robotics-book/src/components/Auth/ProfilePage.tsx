import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import API_CONFIG from '../../config/api';

const ProfilePage: React.FC = () => {
  const auth = useAuth();
  const [formData, setFormData] = useState({
    full_name: '',
    software_background_level: 'beginner',
    hardware_background_level: 'beginner',
    preferred_languages: '',
    learning_goals: ''
  });
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  if (!auth?.isAuthenticated) {
    // Redirect programmatically since Docusaurus doesn't use React Router
    if (typeof window !== 'undefined') {
      window.location.href = '/signin';
    }
    return <div>Redirecting...</div>;
  }

  useEffect(() => {
    if (auth?.user) {
      setFormData({
        full_name: auth.user.full_name || '',
        software_background_level: auth.user.software_background_level || 'beginner',
        hardware_background_level: auth.user.hardware_background_level || 'beginner',
        preferred_languages: auth.user.preferred_languages || '',
        learning_goals: auth.user.learning_goals || ''
      });
      setLoading(false);
    }
  }, [auth?.user]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setSaving(true);
    setMessage(null);

    try {
      // Update user profile via API
      const token = localStorage.getItem('access_token');
      const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          full_name: formData.full_name,
          software_background_level: formData.software_background_level,
          hardware_background_level: formData.hardware_background_level,
          preferred_languages: formData.preferred_languages,
          learning_goals: formData.learning_goals
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Failed to update profile' }));
        throw new Error(errorData.detail || 'Failed to update profile');
      }

      // Update the user in auth context
      const updatedUser = {
        ...auth.user!,
        full_name: formData.full_name,
        software_background_level: formData.software_background_level,
        hardware_background_level: formData.hardware_background_level,
        preferred_languages: formData.preferred_languages,
        learning_goals: formData.learning_goals
      };

      // Update auth context with new user data
      auth.user = updatedUser;

      setMessage('Profile updated successfully!');
    } catch (err: any) {
      // Handle error properly to avoid [object Object] display
      if (err instanceof Error) {
        setError(err.message);
      } else if (typeof err === 'string') {
        setError(err);
      } else if (typeof err === 'object' && err !== null && 'message' in err) {
        setError((err as any).message || 'Failed to update profile');
      } else {
        setError('Failed to update profile');
      }
    } finally {
      setSaving(false);
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  if (!auth) {
    return <div>Loading...</div>;
  }

  return (
    <div style={{ maxWidth: '600px', margin: '2rem auto', padding: '2rem' }}>
      <h1>User Profile</h1>

      {message && (
        <div style={{ color: 'green', marginBottom: '1rem' }}>
          {message}
        </div>
      )}

      {error && (
        <div style={{ color: 'red', marginBottom: '1rem' }}>
          {typeof error === 'string' ? error : JSON.stringify(error, null, 2)}
        </div>
      )}

      <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
        <div>
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            value={auth.user?.email || ''}
            disabled
            style={{ width: '100%', padding: '0.5rem', backgroundColor: '#f5f5f5' }}
          />
        </div>

        <div>
          <label htmlFor="full_name">Full Name:</label>
          <input
            type="text"
            id="full_name"
            name="full_name"
            value={formData.full_name}
            onChange={handleChange}
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>

        <div>
          <label htmlFor="software_background_level">Software Background Level:</label>
          <select
            id="software_background_level"
            name="software_background_level"
            value={formData.software_background_level}
            onChange={handleChange}
            style={{ width: '100%', padding: '0.5rem' }}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div>
          <label htmlFor="hardware_background_level">Hardware Background Level:</label>
          <select
            id="hardware_background_level"
            name="hardware_background_level"
            value={formData.hardware_background_level}
            onChange={handleChange}
            style={{ width: '100%', padding: '0.5rem' }}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div>
          <label htmlFor="preferred_languages">Preferred Programming Languages (comma separated):</label>
          <input
            type="text"
            id="preferred_languages"
            name="preferred_languages"
            value={formData.preferred_languages}
            onChange={handleChange}
            placeholder="e.g., Python, JavaScript, C++"
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>

        <div>
          <label htmlFor="learning_goals">Learning Goals:</label>
          <textarea
            id="learning_goals"
            name="learning_goals"
            value={formData.learning_goals}
            onChange={handleChange}
            placeholder="What do you hope to learn from this book?"
            rows={3}
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>

        <button
          type="submit"
          disabled={saving}
          style={{ padding: '0.75rem', backgroundColor: '#007cba', color: 'white', border: 'none', cursor: saving ? 'not-allowed' : 'pointer' }}
        >
          {saving ? 'Saving...' : 'Update Profile'}
        </button>
      </form>

      <button
        onClick={() => {
          auth.logout();
        }}
        style={{
          marginTop: '1rem',
          padding: '0.75rem',
          backgroundColor: '#dc3545',
          color: 'white',
          border: 'none',
          cursor: 'pointer'
        }}
      >
        Sign Out
      </button>
    </div>
  );
};

export default ProfilePage;