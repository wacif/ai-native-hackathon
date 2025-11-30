import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { LoginForm, SignupForm } from '@site/src/components/Auth';
import { useAuth } from '@site/src/context/AuthContext';

export default function LoginPage(): React.ReactElement {
  const [mode, setMode] = useState<'login' | 'signup'>('login');
  const { isAuthenticated, user } = useAuth();

  // If already logged in, show profile
  if (isAuthenticated && user) {
    return (
      <Layout title="Profile" description="Your profile">
        <main style={{ 
          display: 'flex', 
          justifyContent: 'center', 
          alignItems: 'center', 
          minHeight: '70vh',
          padding: '2rem'
        }}>
          <div style={{
            background: 'var(--ifm-background-surface-color)',
            borderRadius: '16px',
            padding: '2.5rem',
            maxWidth: '420px',
            width: '100%',
            boxShadow: '0 4px 6px -1px rgba(0, 0, 0, 0.1)',
            textAlign: 'center'
          }}>
            <div style={{
              width: '80px',
              height: '80px',
              borderRadius: '50%',
              background: 'linear-gradient(135deg, var(--ifm-color-primary) 0%, var(--ifm-color-primary-dark) 100%)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              margin: '0 auto 1.5rem',
              fontSize: '2rem',
              color: 'white',
              fontWeight: 'bold'
            }}>
              {user.username.charAt(0).toUpperCase()}
            </div>
            <h2 style={{ margin: '0 0 0.5rem' }}>Welcome, {user.username}!</h2>
            <p style={{ color: 'var(--ifm-color-emphasis-600)', margin: '0 0 1.5rem' }}>
              {user.email}
            </p>
            <div style={{
              background: 'var(--ifm-color-emphasis-100)',
              borderRadius: '8px',
              padding: '1rem',
              textAlign: 'left',
              marginBottom: '1.5rem'
            }}>
              <p style={{ margin: '0 0 0.5rem', fontSize: '0.875rem' }}>
                <strong>Software Background:</strong> {user.software_background || 'Not set'}
              </p>
              <p style={{ margin: '0 0 0.5rem', fontSize: '0.875rem' }}>
                <strong>Hardware Background:</strong> {user.hardware_background || 'Not set'}
              </p>
              <p style={{ margin: '0', fontSize: '0.875rem' }}>
                <strong>Language:</strong> {user.selected_language === 'ur' ? 'Urdu' : 'English'}
              </p>
            </div>
            <a 
              href="/ai-native-hackathon/docs/physical-ai/intro"
              style={{
                display: 'inline-block',
                padding: '0.75rem 1.5rem',
                background: 'linear-gradient(135deg, var(--ifm-color-primary) 0%, var(--ifm-color-primary-dark) 100%)',
                color: 'white',
                borderRadius: '8px',
                textDecoration: 'none',
                fontWeight: '600'
              }}
            >
              Continue Learning →
            </a>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout
      title={mode === 'login' ? 'Sign In' : 'Sign Up'}
      description={mode === 'login' ? 'Sign in to your account' : 'Create a new account'}
    >
      <main style={{ 
        display: 'flex', 
        justifyContent: 'center', 
        alignItems: 'center', 
        minHeight: '70vh' 
      }}>
        {mode === 'login' ? (
          <LoginForm
            onSuccess={() => window.location.reload()}
            onSwitchToSignup={() => setMode('signup')}
          />
        ) : (
          <SignupForm
            onSuccess={() => setMode('login')}
            onSwitchToLogin={() => setMode('login')}
          />
        )}
      </main>
    </Layout>
  );
}

