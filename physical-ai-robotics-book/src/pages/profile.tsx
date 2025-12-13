import React from 'react';
import Layout from '@theme/Layout';
import ProfilePage from '../components/Auth/ProfilePage';

export default function Profile(): JSX.Element {
  return (
    <Layout title="My Profile" description="Manage your profile and background information">
      <main style={{ padding: '2rem 0' }}>
        <div className="container">
          <ProfilePage />
        </div>
      </main>
    </Layout>
  );
}