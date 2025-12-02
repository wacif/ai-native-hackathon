/**
 * Dashboard Page
 * Professional futuristic dashboard with hero section and data visualization
 */
import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useAuth } from '@site/src/context/AuthContext';
import ParticleBackground from '@site/src/components/Landing/ParticleBackground';
import styles from './dashboard.module.css';

// Course modules from docs structure
const MODULES = [
  { id: 1, title: 'Introduction to Physical AI', path: '/docs/physical-ai/intro', status: 'available' },
  { id: 2, title: 'Hardware Setup Guide', path: '/docs/physical-ai/hardware-setup', status: 'available' },
  { id: 3, title: 'ROS 2 Fundamentals', path: '/docs/physical-ai/module1-ros2', status: 'available' },
  { id: 4, title: 'Simulation Environments', path: '/docs/physical-ai/module2-simulation', status: 'available' },
  { id: 5, title: 'NVIDIA Isaac Platform', path: '/docs/physical-ai/module3-isaac', status: 'available' },
  { id: 6, title: 'Vision-Language-Action Models', path: '/docs/physical-ai/module4-vla', status: 'available' },
];

// Get time-based greeting
function getGreeting(): string {
  const hour = new Date().getHours();
  if (hour < 12) return 'Good morning';
  if (hour < 17) return 'Good afternoon';
  return 'Good evening';
}

// Format preference value for display
function formatPrefValue(value: string | undefined): string {
  if (!value) return 'Not configured';
  return value.charAt(0).toUpperCase() + value.slice(1).replace(/_/g, ' ');
}

function DashboardContent() {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  // Redirect to login if not authenticated
  useEffect(() => {
    if (mounted && !isLoading && !isAuthenticated) {
      window.location.href = '/ai-native-hackathon/login';
    }
  }, [mounted, isLoading, isAuthenticated]);

  const handleSignOut = () => {
    signOut();
    window.location.href = '/ai-native-hackathon/';
  };

  const handleOpenChatbot = () => {
    window.dispatchEvent(new CustomEvent('openChatbot'));
    const chatbotButton = document.querySelector('[data-chatbot-toggle]') as HTMLElement;
    if (chatbotButton) chatbotButton.click();
  };

  // SSR guard
  if (!mounted) return null;

  // Loading state
  if (isLoading) {
    return (
      <div className={styles.loading}>
        <div className={styles.loadingSpinner} />
        <span className={styles.loadingText}>Loading your dashboard...</span>
      </div>
    );
  }

  // Auth guard
  if (!isAuthenticated || !user) {
    return (
      <div className={styles.loading}>
        <div className={styles.loadingSpinner} />
        <span className={styles.loadingText}>Redirecting to login...</span>
      </div>
    );
  }

  const isProfileIncomplete = !user.software_background || !user.hardware_background;
  const completedModules = 0; // Placeholder - would come from user progress tracking
  const progressPercent = Math.round((completedModules / MODULES.length) * 100);

  return (
    <div className={styles.dashboard}>
      <ParticleBackground />
      
      {/* Hero Section */}
      <section className={styles.hero}>
        <p className={styles.greeting}>{getGreeting()}</p>
        <h1 className={styles.userName}>{user.username}</h1>
        <p className={styles.userEmail}>{user.email}</p>
        
        {/* Quick Stats */}
        <div className={styles.quickStats}>
          <div className={styles.quickStat}>
            <span className={styles.quickStatValue}>{MODULES.length}</span>
            <span className={styles.quickStatLabel}>Modules</span>
          </div>
          <div className={styles.quickStat}>
            <span className={styles.quickStatValue}>{completedModules}</span>
            <span className={styles.quickStatLabel}>Completed</span>
          </div>
          <div className={styles.quickStat}>
            <span className={styles.quickStatValue}>{progressPercent}%</span>
            <span className={styles.quickStatLabel}>Progress</span>
          </div>
          <div className={styles.quickStat}>
            <span className={styles.quickStatValue}>12</span>
            <span className={styles.quickStatLabel}>Weeks</span>
          </div>
        </div>
      </section>

      {/* Main Content */}
      <div className={styles.content}>
        {/* Main Grid: Progress + Profile */}
        <div className={styles.mainGrid}>
          {/* Learning Progress Card */}
          <div className={styles.progressCard}>
            <h2 className={styles.cardTitle}>
              <span className={styles.cardIcon}>📚</span>
              Learning Path
            </h2>
            <div className={styles.modulesList}>
              {MODULES.map((module) => (
                <Link
                  key={module.id}
                  to={`/ai-native-hackathon${module.path}`}
                  className={styles.moduleItem}
                >
                  <div className={styles.moduleNumber}>{module.id}</div>
                  <div className={styles.moduleInfo}>
                    <h3 className={styles.moduleTitle}>{module.title}</h3>
                    <span className={styles.moduleStatus}>
                      {module.status === 'available' ? 'Ready to start' : 'Coming soon'}
                    </span>
                  </div>
                  <span className={styles.moduleArrow}>→</span>
                </Link>
              ))}
            </div>
          </div>

          {/* Profile Card */}
          <div className={styles.profileCard}>
            <div className={styles.avatarWrapper}>
              <div className={styles.avatarGlow} />
              <div className={styles.avatar}>
                {user.username.charAt(0).toUpperCase()}
              </div>
            </div>
            <h3 className={styles.profileName}>{user.username}</h3>
            <p className={styles.profileEmail}>{user.email}</p>
            <div className={styles.profileStats}>
              <div className={styles.profileStat}>
                <span className={styles.profileStatValue}>{completedModules}/{MODULES.length}</span>
                <span className={styles.profileStatLabel}>Modules</span>
              </div>
              <div className={styles.profileStat}>
                <span className={styles.profileStatValue}>{progressPercent}%</span>
                <span className={styles.profileStatLabel}>Complete</span>
              </div>
            </div>
          </div>
        </div>

        {/* Secondary Grid: Preferences + Activity */}
        <div className={styles.secondaryGrid}>
          {/* Preferences Card */}
          <div className={styles.preferencesCard}>
            <h2 className={styles.cardTitle}>
              <span className={styles.cardIcon}>⚙️</span>
              Learning Preferences
            </h2>
            <div className={styles.prefGrid}>
              <div className={styles.prefItem}>
                <span className={styles.prefLabel}>
                  <span className={styles.prefLabelIcon}>💻</span>
                  Software Level
                </span>
                <span className={`${styles.prefValue} ${!user.software_background ? styles.notSet : ''}`}>
                  {formatPrefValue(user.software_background)}
                </span>
              </div>
              <div className={styles.prefItem}>
                <span className={styles.prefLabel}>
                  <span className={styles.prefLabelIcon}>🔧</span>
                  Hardware Level
                </span>
                <span className={`${styles.prefValue} ${!user.hardware_background ? styles.notSet : ''}`}>
                  {formatPrefValue(user.hardware_background)}
                </span>
              </div>
              <div className={styles.prefItem}>
                <span className={styles.prefLabel}>
                  <span className={styles.prefLabelIcon}>🖥️</span>
                  Operating System
                </span>
                <span className={`${styles.prefValue} ${!user.operating_system ? styles.notSet : ''}`}>
                  {formatPrefValue(user.operating_system)}
                </span>
              </div>
              <div className={styles.prefItem}>
                <span className={styles.prefLabel}>
                  <span className={styles.prefLabelIcon}>📖</span>
                  Learning Style
                </span>
                <span className={`${styles.prefValue} ${!user.preferred_explanation_style ? styles.notSet : ''}`}>
                  {formatPrefValue(user.preferred_explanation_style)}
                </span>
              </div>
            </div>
            {isProfileIncomplete && (
              <div className={styles.profileHint}>
                ⚠️ Complete your profile for personalized content recommendations
              </div>
            )}
          </div>

          {/* Recent Activity Card */}
          <div className={styles.activityCard}>
            <h2 className={styles.cardTitle}>
              <span className={styles.cardIcon}>📊</span>
              Recent Activity
            </h2>
            <div className={styles.activityList}>
              <div className={styles.activityItem}>
                <div className={styles.activityIcon}>🎉</div>
                <div className={styles.activityContent}>
                  <p className={styles.activityText}>Welcome to Physical AI & Robotics!</p>
                  <span className={styles.activityTime}>Account created</span>
                </div>
              </div>
              <div className={styles.activityItem}>
                <div className={styles.activityIcon}>🚀</div>
                <div className={styles.activityContent}>
                  <p className={styles.activityText}>Ready to start Module 1</p>
                  <span className={styles.activityTime}>Begin your journey</span>
                </div>
              </div>
              {user.software_background && (
                <div className={styles.activityItem}>
                  <div className={styles.activityIcon}>✅</div>
                  <div className={styles.activityContent}>
                    <p className={styles.activityText}>Profile preferences saved</p>
                    <span className={styles.activityTime}>Content personalized</span>
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>

        {/* Action Buttons */}
        <div className={styles.actions}>
          <Link to="/ai-native-hackathon/docs/physical-ai/intro" className={styles.primaryButton}>
            <span>Continue Learning</span>
            <span className={styles.buttonGlow} aria-hidden="true" />
          </Link>
          <Link to="/ai-native-hackathon/profile" className={styles.secondaryButton}>
            ✏️ Edit Profile
          </Link>
          <button onClick={handleOpenChatbot} className={styles.secondaryButton}>
            💬 Ask AI Tutor
          </button>
          <button onClick={handleSignOut} className={styles.dangerButton}>
            🚪 Sign Out
          </button>
        </div>
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
