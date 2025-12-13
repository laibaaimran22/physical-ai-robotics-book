import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import Link from '@docusaurus/Link';

const SigninPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
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

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      await auth?.login(email, password);
    } catch (err: any) {
      // Handle error properly to avoid [object Object] display
      if (err instanceof Error) {
        setError(err.message);
      } else if (typeof err === 'object' && err !== null && 'message' in err) {
        setError(err.message as string);
      } else if (typeof err === 'string') {
        setError(err);
      } else {
        setError('Login failed');
      }
    } finally {
      setLoading(false);
    }
  };

  if (!auth) {
    return <div>Loading...</div>;
  }

  return (
    <div style={{ maxWidth: '400px', margin: '2rem auto', padding: '2rem' }}>
      <h1>Sign In</h1>

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
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>

        <div>
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>

        <button
          type="submit"
          disabled={loading}
          style={{ padding: '0.75rem', backgroundColor: '#007cba', color: 'white', border: 'none', cursor: loading ? 'not-allowed' : 'pointer' }}
        >
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>

      <p style={{ marginTop: '1rem' }}>
        Don't have an account? <Link to="/signup">Sign up here</Link>
      </p>
    </div>
  );
};

export default SigninPage;