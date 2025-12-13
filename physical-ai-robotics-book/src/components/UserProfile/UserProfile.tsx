import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import clsx from 'clsx';
import styles from './UserProfile.module.css';

const UserProfile: React.FC = () => {
  const { user, loading, isAuthenticated, logout } = useAuth();

  if (loading) {
    return <div className={styles.loading}>Loading...</div>;
  }

  if (!isAuthenticated || !user) {
    return (
      <div className={styles.profileContainer}>
        <p>You are not logged in. <a href="/signin">Sign in</a> or <a href="/signup">sign up</a> to access personalized content.</p>
      </div>
    );
  }

  return (
    <div className={styles.profileContainer}>
      <div className={styles.profileCard}>
        <h2 className={styles.profileTitle}>Your Profile</h2>

        <div className={styles.profileSection}>
          <h3>Account Information</h3>
          <div className={styles.profileField}>
            <strong>Email:</strong> {user.email}
          </div>
          {user.full_name && (
            <div className={styles.profileField}>
              <strong>Name:</strong> {user.full_name}
            </div>
          )}
        </div>

        <div className={styles.profileSection}>
          <h3>Background Information</h3>
          <div className={styles.profileField}>
            <strong>Software Background:</strong> {user.software_background_level || 'Not specified'}
          </div>
          <div className={styles.profileField}>
            <strong>Hardware Background:</strong> {user.hardware_background_level || 'Not specified'}
          </div>
          {user.preferred_languages && (
            <div className={styles.profileField}>
              <strong>Preferred Languages:</strong> {user.preferred_languages}
            </div>
          )}
          {user.learning_goals && (
            <div className={styles.profileField}>
              <strong>Learning Goals:</strong> {user.learning_goals}
            </div>
          )}
        </div>

        <div className={styles.profileActions}>
          <button
            onClick={logout}
            className={styles.logoutButton}
          >
            Sign Out
          </button>
          <a href="/update-profile" className={styles.updateProfileLink}>
            Update Profile
          </a>
        </div>
      </div>
    </div>
  );
};

export default UserProfile;