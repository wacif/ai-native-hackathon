/**
 * User Button Component
 * Shows user avatar/name when logged in, or login button when logged out
 */
import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './Auth.module.css';

interface UserButtonProps {
  onLoginClick?: () => void;
  onSignupClick?: () => void;
}

export default function UserButton({ onLoginClick, onSignupClick }: UserButtonProps) {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  if (isLoading) {
    return (
      <button className={styles.userButton} disabled>
        <span>Loading...</span>
      </button>
    );
  }

  if (!isAuthenticated || !user) {
    return (
      <div style={{ display: 'flex', gap: '0.5rem' }}>
        <button className={styles.userButton} onClick={onLoginClick}>
          Sign In
        </button>
      </div>
    );
  }

  const initials = user.username
    .split(' ')
    .map(n => n[0])
    .join('')
    .toUpperCase()
    .slice(0, 2);

  return (
    <div ref={dropdownRef} style={{ position: 'relative' }}>
      <button 
        className={styles.userButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <div className={styles.userAvatar}>{initials}</div>
        <span>{user.username}</span>
        <svg 
          width="12" 
          height="12" 
          viewBox="0 0 12 12" 
          fill="currentColor"
          style={{ 
            transform: isOpen ? 'rotate(180deg)' : 'rotate(0deg)',
            transition: 'transform 0.2s'
          }}
        >
          <path d="M2 4L6 8L10 4" stroke="currentColor" strokeWidth="1.5" fill="none"/>
        </svg>
      </button>

      {isOpen && (
        <div className={styles.userDropdown}>
          <div className={styles.dropdownHeader}>
            <strong>{user.username}</strong>
            <span>{user.email}</span>
          </div>
          <a 
            href="/ai-native-hackathon/profile"
            className={styles.dropdownItem}
            onClick={() => setIsOpen(false)}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"/>
              <circle cx="12" cy="7" r="4"/>
            </svg>
            Profile Settings
          </a>
          <button 
            className={`${styles.dropdownItem} ${styles.danger}`}
            onClick={() => {
              signOut();
              setIsOpen(false);
            }}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"/>
              <polyline points="16,17 21,12 16,7"/>
              <line x1="21" y1="12" x2="9" y2="12"/>
            </svg>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}

