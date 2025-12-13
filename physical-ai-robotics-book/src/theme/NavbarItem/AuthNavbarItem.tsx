import React from 'react';
import { useAuth } from '../../../hooks/useAuth';
import Link from '@docusaurus/Link';
import { translate } from '@docusaurus/Translate';

const AuthNavbarItem: React.FC = () => {
  const auth = useAuth();

  if (!auth) {
    return null;
  }

  return (
    <div className="navbar__item">
      {auth.isAuthenticated ? (
        <div className="dropdown dropdown--none navbar__item navbar__item--icons">
          <span className="navbar__link">
            <span>Welcome, {auth.user?.full_name || auth.user?.email}!</span>
          </span>
          <Link to="/profile" className="navbar__link">
            {translate({ id: 'theme.navbar.profile', message: 'Profile' })}
          </Link>
          <button
            onClick={() => auth.logout()}
            className="navbar__link"
            style={{ background: 'none', border: 'none', cursor: 'pointer' }}
          >
            {translate({ id: 'theme.navbar.signout', message: 'Sign Out' })}
          </button>
        </div>
      ) : (
        <div className="dropdown dropdown--none navbar__item navbar__item--icons">
          <Link to="/signin" className="navbar__link">
            {translate({ id: 'theme.navbar.signin', message: 'Sign In' })}
          </Link>
          <Link to="/signup" className="navbar__link">
            {translate({ id: 'theme.navbar.signup', message: 'Sign Up' })}
          </Link>
        </div>
      )}
    </div>
  );
};

export default AuthNavbarItem;