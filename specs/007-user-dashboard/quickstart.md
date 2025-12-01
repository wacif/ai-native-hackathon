# Quickstart: User Dashboard

**Feature**: 007-user-dashboard  
**Branch**: `007-user-dashboard`

## Overview

Create a futuristic-themed dashboard page at `/dashboard` that displays user info, learning progress, and preferences in a card layout. Redirect authenticated users from `/login` to `/dashboard`.

## Prerequisites

- Existing AuthContext with user data and signOut method
- Existing CSS variables in custom.css for theme
- Docusaurus 3.9.2 page routing

## Files to Create/Modify

### New Files
1. `src/pages/dashboard.tsx` - Dashboard page component
2. `src/pages/dashboard.module.css` - Dashboard styles

### Files to Modify
1. `src/pages/login.tsx` - Add redirect to dashboard when authenticated

## Implementation Steps

### Step 1: Create Dashboard Page (`src/pages/dashboard.tsx`)

```tsx
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

export default function DashboardPage() {
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

  if (!mounted || isLoading) {
    return (
      <Layout title="Dashboard">
        <div className={styles.loading}>Loading...</div>
      </Layout>
    );
  }

  if (!isAuthenticated || !user) {
    return null; // Will redirect
  }

  return (
    <Layout title="Dashboard">
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
            <p className={styles.stat}>
              <span className={styles.statNumber}>{MODULES.length}</span>
              <span className={styles.statLabel}>modules available</span>
            </p>
            <a href={`/ai-native-hackathon${MODULES[0].path}`} className={styles.moduleLink}>
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
              <span className={styles.prefLabel}>Software:</span>
              <span className={styles.prefValue}>{user.software_background || 'Not set'}</span>
            </div>
            <div className={styles.prefItem}>
              <span className={styles.prefLabel}>Hardware:</span>
              <span className={styles.prefValue}>{user.hardware_background || 'Not set'}</span>
            </div>
            <div className={styles.prefItem}>
              <span className={styles.prefLabel}>OS:</span>
              <span className={styles.prefValue}>{user.operating_system || 'Not set'}</span>
            </div>
            {(!user.software_background || !user.hardware_background) && (
              <p className={styles.hint}>Complete your profile for personalized content</p>
            )}
          </div>
        </div>

        {/* Action Buttons */}
        <div className={styles.actions}>
          <a href="/ai-native-hackathon/docs/physical-ai/intro" className={styles.primaryButton}>
            Continue Learning →
          </a>
          <a href="/ai-native-hackathon/profile" className={styles.secondaryButton}>
            Edit Profile
          </a>
          <button onClick={signOut} className={styles.secondaryButton}>
            Sign Out
          </button>
          <button onClick={() => {/* TODO: Open chatbot */}} className={styles.secondaryButton}>
            💬 Open Chatbot
          </button>
        </div>
      </div>
    </Layout>
  );
}
```

### Step 2: Create Dashboard Styles (`src/pages/dashboard.module.css`)

Key styles to implement:
- Glass-morphism cards with backdrop blur
- Cyan accent glow effects
- Responsive grid layout
- Button hover animations

### Step 3: Update Login Page (`src/pages/login.tsx`)

Add redirect when authenticated:
```tsx
useEffect(() => {
  if (!isLoading && isAuthenticated) {
    window.location.href = '/ai-native-hackathon/dashboard';
  }
}, [isLoading, isAuthenticated]);
```

Remove the inline dashboard UI that currently shows when authenticated.

## Testing Checklist

- [ ] Unauthenticated user visiting `/dashboard` → redirected to `/login`
- [ ] Authenticated user visiting `/login` → redirected to `/dashboard`
- [ ] Dashboard shows user info card with avatar, username, email
- [ ] Dashboard shows learning progress card with module count
- [ ] Dashboard shows preferences card with user settings
- [ ] "Continue Learning" navigates to docs
- [ ] "Edit Profile" navigates to `/profile`
- [ ] "Sign Out" logs out and redirects to `/login`
- [ ] "Open Chatbot" opens chatbot (if implemented)
- [ ] Responsive: cards stack on mobile (< 768px)
- [ ] Theme matches landing page (dark bg, cyan accents)

## Dependencies

No new dependencies required. Uses existing:
- AuthContext for user data
- Docusaurus Layout component
- CSS variables from custom.css
