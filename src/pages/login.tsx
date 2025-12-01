import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { LoginForm, SignupForm } from '@site/src/components/Auth';
import { useAuth } from '@site/src/context/AuthContext';

export default function LoginPage(): React.ReactElement {
  const [mode, setMode] = useState<'login' | 'signup'>('login');
  const { isAuthenticated, isLoading } = useAuth();
  const [mounted, setMounted] = useState(false);

  // Handle client-side mounting
  useEffect(() => {
    setMounted(true);
  }, []);

  // Redirect to dashboard if already authenticated
  useEffect(() => {
    if (mounted && !isLoading && isAuthenticated) {
      window.location.href = '/ai-native-hackathon/dashboard';
    }
  }, [mounted, isLoading, isAuthenticated]);

  // Show nothing during redirect
  if (mounted && !isLoading && isAuthenticated) {
    return (
      <Layout title="Redirecting..." description="Redirecting to dashboard">
        <main style={{ 
          display: 'flex', 
          justifyContent: 'center', 
          alignItems: 'center', 
          minHeight: '70vh' 
        }}>
          <p>Redirecting to dashboard...</p>
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
            onSuccess={() => {
              window.location.href = '/ai-native-hackathon/dashboard';
            }}
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

