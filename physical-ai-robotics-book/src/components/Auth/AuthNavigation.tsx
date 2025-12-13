import React from 'react';
import { useAuth } from '../../hooks/useAuth'; // Updated import path
import Link from '@docusaurus/Link';

const AuthNavigation: React.FC = () => {
  const auth = useAuth();

  if (!auth) {
    return null; // or a loading state
  }

  return (
    <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
      {auth.isAuthenticated ? (
        <>
          <span>Welcome, {auth.user?.full_name || auth.user?.email}!</span>
          <Link to="/profile">Profile</Link>
          <button
            onClick={() => auth.logout()}
            style={{
              background: 'none',
              border: 'none',
              color: '#007cba',
              cursor: 'pointer',
              textDecoration: 'underline'
            }}
          >
            Sign Out
          </button>
        </>
      ) : (
        <>
          <Link to="/signin">Sign In</Link>
          <Link to="/signup">Sign Up</Link>
        </>
      )}
    </div>
  );
};

export default AuthNavigation;