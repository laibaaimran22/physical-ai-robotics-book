import React from 'react';
import Layout from '@theme/Layout';
import SigninPage from '../components/Auth/SigninPage';

export default function Signin(): JSX.Element {
  return (
    <Layout title="Sign In" description="Sign in to access personalized content">
      <main style={{ padding: '2rem 0' }}>
        <div className="container">
          <SigninPage />
        </div>
      </main>
    </Layout>
  );
}