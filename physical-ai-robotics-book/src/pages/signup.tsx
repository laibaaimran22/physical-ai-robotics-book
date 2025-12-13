import React from 'react';
import Layout from '@theme/Layout';
import SignupPage from '../components/Auth/SignupPage';

export default function Signup(): JSX.Element {
  return (
    <Layout title="Sign Up" description="Create an account for personalized content">
      <main style={{ padding: '2rem 0' }}>
        <div className="container">
          <SignupPage />
        </div>
      </main>
    </Layout>
  );
}