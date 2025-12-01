/**
 * Dashboard Page
 * Displays user info, learning progress, and preferences in a futuristic card layout
 */
import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './dashboard.module.css';

// Hardcoded modules from docs structure
const MODULES = [
  { id: 1, title: 'Introduction', path: '/docs/physical-ai/intro' },
  { id: 2, title: 'Hardware Setup', path: '/docs/physical-ai/hardware-setup' },
  { id: 3, title: 'ROS 2 Basics', path: '/docs/physical-ai/module1-ros2' },
  { id: 4, title: 'Simulation', path: '/docs/physical-ai/module2-simulation' },
  { id: 5, title: 'NVIDIA Isaac', path: '/docs/physical-ai/module3-isaac' },
  { id: 6, title: 'VLA Models', path: '/docs/physical-ai/module4-vla' },
];

function DashboardContent() {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();
  const [mounted, setMounted] = useState(false);

  // Handle client-side mounting to prevent SSR hydration issues
  useEffect(() => {
    setMounted(true);
  }, []);

  // Redirect to login if not authenticated
  useEffect(() => {
    if (mounted && !isLoading && !isAuthenticated) {
      window.location.href = '/ai-native-hackathon/login';
    }
  }, [mounted, isLoading, isAuthenticated]);

  // Handle sign out
  const handleSignOut = () => {
    signOut();
    window.location.href = '/ai-native-hackathon/login';
  };

  // Handle chatbot open (dispatch custom event for chatbot component)
  const handleOpenChatbot = () => {
    // Try to dispatch a custom event that the chatbot might listen to
    window.dispatchEvent(new CustomEvent('openChatbot'));
    // Fallback: try clicking the chatbot button if it exists
    const chatbotButton = document.querySelector('[data-chatbot-toggle]') as HTMLElement;
    if (chatbotButton) {
      chatbotButton.click();
    }
  };

  // Show nothing during SSR
  if (!mounted) {
    return null;
  }

  // Show loading state while auth is being checked
  if (isLoading) {
    return (
      <div className={styles.loading}>
        <div className={styles.loadingSpinner}></div>
        Loading...
      </div>
    );
  }

  // If not authenticated, return null (redirect will happen via useEffect)
  if (!isAuthenticated || !user) {
    return (
      <div className={styles.loading}>
        Redirecting to login...
      </div>
    );
  }

  // Check if profile is incomplete
  const isProfileIncomplete = !user.software_background || !user.hardware_background;

  return (
    <div className={styles.container}>
      <h1 className={styles.title}>Welcome back, {user.username}!</h1>
      
      <div className={styles.grid}>
        {/* User Info Card */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardIcon}>👤</span>
            <h2>Profile</h2>
          </div>
          <div className={styles.avatar}>
            {user.username.charAt(0).toUpperCase()}
          </div>
          <p className={styles.username}>{user.username}</p>
          <p className={styles.email}>{user.email}</p>
        </div>

        {/* Learning Progress Card */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardIcon}>📚</span>
            <h2>Learning Progress</h2>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>{MODULES.length}</span>
            <span className={styles.statLabel}>modules available</span>
          </div>
          <a 
            href={`/ai-native-hackathon${MODULES[0].path}`} 
            className={styles.moduleLink}
          >
            Start Module 1: {MODULES[0].title} →
          </a>
        </div>

        {/* Preferences Card */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardIcon}>⚙️</span>
            <h2>Preferences</h2>
          </div>
          <div className={styles.prefItem}>
            <span className={styles.prefLabel}>Software Level</span>
            <span className={styles.prefValue}>
              {user.software_background || 'Not set'}
            </span>
          </div>
          <div className={styles.prefItem}>
            <span className={styles.prefLabel}>Hardware Level</span>
            <span className={styles.prefValue}>
              {user.hardware_background || 'Not set'}
            </span>
          </div>
          <div className={styles.prefItem}>
            <span className={styles.prefLabel}>Operating System</span>
            <span className={styles.prefValue}>
              {user.operating_system || 'Not set'}
            </span>
          </div>
          <div className={styles.prefItem}>
            <span className={styles.prefLabel}>Learning Style</span>
            <span className={styles.prefValue}>
              {user.preferred_explanation_style || 'Not set'}
            </span>
          </div>
          {isProfileIncomplete && (
            <p className={styles.hint}>
              ⚠️ Complete your profile for personalized content
            </p>
          )}
        </div>
      </div>

      {/* Action Buttons */}
      <div className={styles.actions}>
        <a 
          href="/ai-native-hackathon/docs/physical-ai/intro" 
          className={styles.primaryButton}
        >
          Continue Learning →
        </a>
        <a 
          href="/ai-native-hackathon/profile" 
          className={styles.secondaryButton}
        >
          Edit Profile
        </a>
        <button 
          onClick={handleSignOut} 
          className={styles.secondaryButton}
        >
          Sign Out
        </button>
        <button 
          onClick={handleOpenChatbot} 
          className={styles.secondaryButton}
        >
          💬 Ask AI Tutor
        </button>
      </div>
    </div>
  );
}

export default function DashboardPage() {
  return (
    <Layout title="Dashboard" description="Your learning dashboard">
      <DashboardContent />
    </Layout>
  );
}
