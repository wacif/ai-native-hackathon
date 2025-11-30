import React, { useState, useEffect, useCallback } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { AuthProvider, useAuth } from '@site/src/context/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Content.module.css';

type Props = WrapperProps<typeof ContentType>;

function PersonalizeControls(): React.ReactElement | null {
  const { isAuthenticated, user, token } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [chapterId, setChapterId] = useState<string>('');
  const [mounted, setMounted] = useState(false);

  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';

  useEffect(() => {
    setMounted(true);
    const path = window.location.pathname;
    // Match various doc paths - capture the last segment as chapterId
    const matches = path.match(/\/docs\/(?:physical-ai\/)?([^\/]+)\/?$/);
    if (matches) {
      setChapterId(matches[1]);
    }
  }, []);

  const hasPreferences = user && (
    (user.programming_languages && user.programming_languages.length > 0) ||
    user.operating_system ||
    (user.learning_goals && user.learning_goals.length > 0) ||
    user.preferred_explanation_style ||
    (user.prior_knowledge && user.prior_knowledge.length > 0) ||
    user.industry
  );

  const handlePersonalize = useCallback(async () => {
    if (!token || !chapterId) return;

    setIsLoading(true);
    setError(null);

    try {
      const articleElement = document.querySelector('article.markdown');
      if (!articleElement) {
        throw new Error('Could not find article content');
      }

      const originalContent = articleElement.innerHTML;

      const response = await fetch(`${API_URL}/personalize-chapter`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_content: articleElement.textContent || '',
          force_refresh: false,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      sessionStorage.setItem(`original_${chapterId}`, originalContent);
      
      const container = document.createElement('div');
      container.innerHTML = data.personalized_content
        .replace(/\n/g, '<br>')
        .replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre><code>$2</code></pre>')
        .replace(/`([^`]+)`/g, '<code>$1</code>')
        .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
        .replace(/\*([^*]+)\*/g, '<em>$1</em>')
        .replace(/^# (.*$)/gm, '<h1>$1</h1>')
        .replace(/^## (.*$)/gm, '<h2>$1</h2>')
        .replace(/^### (.*$)/gm, '<h3>$1</h3>');
      
      articleElement.innerHTML = container.innerHTML;
      setIsPersonalized(true);
    } catch (err) {
      console.error('Error personalizing content:', err);
      setError('Failed to personalize content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  }, [API_URL, chapterId, token]);

  const handleRevert = useCallback(() => {
    const original = sessionStorage.getItem(`original_${chapterId}`);
    if (original) {
      const articleElement = document.querySelector('article.markdown');
      if (articleElement) {
        articleElement.innerHTML = original;
        setIsPersonalized(false);
        sessionStorage.removeItem(`original_${chapterId}`);
      }
    } else {
      window.location.reload();
    }
  }, [chapterId]);

  // Don't render on server
  if (!mounted) {
    return null;
  }

  // Don't show for non-chapter pages
  if (!chapterId) {
    return null;
  }

  // Show sign-in message if not authenticated
  if (!isAuthenticated) {
    return (
      <div className={styles.personalizeContainer}>
        <div className={styles.noPreferences}>
          <span className={styles.icon}>🔒</span>
          <span>Sign in to personalize this content</span>
        </div>
      </div>
    );
  }

  // Suggest profile setup if no preferences
  if (!hasPreferences) {
    return (
      <div className={styles.personalizeContainer}>
        <div className={styles.noPreferences}>
          <span className={styles.icon}>⚙️</span>
          <span>Update your profile to enable personalized content</span>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.personalizeContainer}>
      {error && <div className={styles.error}>{error}</div>}
      
      {isPersonalized ? (
        <div className={styles.personalizedBadge}>
          <span className={styles.checkmark}>✨</span>
          <span>Content personalized for you</span>
          <button
            className={styles.revertButton}
            onClick={handleRevert}
          >
            View Original
          </button>
        </div>
      ) : (
        <button
          className={styles.personalizeButton}
          onClick={handlePersonalize}
          disabled={isLoading}
        >
          {isLoading ? (
            <>
              <span className={styles.spinner}></span>
              Personalizing...
            </>
          ) : (
            <>
              <span className={styles.icon}>✨</span>
              Personalize for me
            </>
          )}
        </button>
      )}
      
      {user && (
        <div className={styles.profileHint}>
          {user.operating_system && (
            <span className={styles.tag}>{user.operating_system.toUpperCase()}</span>
          )}
          {user.programming_languages && user.programming_languages.slice(0, 2).map(lang => (
            <span key={lang} className={styles.tag}>{lang}</span>
          ))}
          {user.preferred_explanation_style && (
            <span className={styles.tag}>{user.preferred_explanation_style}</span>
          )}
        </div>
      )}
    </div>
  );
}

export default function ContentWrapper(props: Props): React.ReactElement {
  return (
    <>
      <AuthProvider>
        <PersonalizeControls />
      </AuthProvider>
      <Content {...props} />
    </>
  );
}
