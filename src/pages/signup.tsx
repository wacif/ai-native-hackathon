import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Auth/SignupForm';

export default function Signup() {
  return (
    <Layout title="Sign Up">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        padding: '2rem 0',
        minHeight: 'calc(100vh - var(--ifm-navbar-height) - var(--ifm-footer-height))',
      }}>
        <SignupForm />
      </main>
    </Layout>
  );
}
