import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import Link from '@docusaurus/Link';

const SignupPage: React.FC = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    full_name: '',
    software_background_level: 'beginner',
    hardware_background_level: 'beginner',
    preferred_languages: '',
    learning_goals: ''
  });
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const auth = useAuth();

  if (auth?.isAuthenticated) {
    // Redirect programmatically since Docusaurus doesn't use React Router
    if (typeof window !== 'undefined') {
      window.location.href = '/';
    }
    return <div>Redirecting...</div>;
  }

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
    setLoading(true);

    try {
      await auth?.register(formData);
    } catch (err: any) {
      // Handle error properly to avoid [object Object] display
      if (err instanceof Error) {
        setError(err.message);
      } else if (typeof err === 'object' && err !== null && 'message' in err) {
        setError(err.message as string);
      } else if (typeof err === 'string') {
        setError(err);
      } else {
        setError('Registration failed');
      }
    } finally {
      setLoading(false);
    }
  };

  if (!auth) {
    return <div>Loading...</div>;
  }

  return (
    <div style={{ maxWidth: '600px', margin: '2rem auto', padding: '2rem' }}>
      <h1>Create Account</h1>

      {error && (
        <div style={{ color: 'red', marginBottom: '1rem' }}>
          {typeof error === 'string' ? error :
           error instanceof Error ? error.message :
           JSON.stringify(error, null, 2)}
        </div>
      )}

      <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
        <div>
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>

        <div>
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            style={{ width: '100%', padding: '0.5rem' }}
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
          disabled={loading}
          style={{ padding: '0.75rem', backgroundColor: '#007cba', color: 'white', border: 'none', cursor: loading ? 'not-allowed' : 'pointer' }}
        >
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>

      <p style={{ marginTop: '1rem' }}>
        Already have an account? <Link to="/signin">Sign in here</Link>
      </p>
    </div>
  );
};

export default SignupPage;