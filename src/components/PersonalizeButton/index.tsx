import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  chapterContent: string;
  onPersonalize: (personalizedContent: string) => void;
}

export default function PersonalizeButton({
  chapterId,
  chapterContent,
  onPersonalize,
}: PersonalizeButtonProps) {
  const { isAuthenticated, user, token } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isPersonalized, setIsPersonalized] = useState(false);

  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';

  // Check if user has any personalization preferences
  const hasPreferences = user && (
    (user.programming_languages && user.programming_languages.length > 0) ||
    user.operating_system ||
    (user.learning_goals && user.learning_goals.length > 0) ||
    user.preferred_explanation_style ||
    (user.prior_knowledge && user.prior_knowledge.length > 0) ||
    user.industry
  );

  const handlePersonalize = async () => {
    if (!token) return;

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/personalize-chapter`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_content: chapterContent,
          force_refresh: false,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      onPersonalize(data.personalized_content);
      setIsPersonalized(true);
    } catch (err) {
      console.error('Error personalizing content:', err);
      setError('Failed to personalize content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleRevert = () => {
    // Reload the page to get original content
    window.location.reload();
  };

  // Don't show if not authenticated
  if (!isAuthenticated) {
    return null;
  }

  // Don't show if user has no preferences set
  if (!hasPreferences) {
    return (
      <div className={styles.personalizeContainer}>
        <div className={styles.noPreferences}>
          <span className={styles.icon}>⚙️</span>
          <span>Set up your profile to enable personalized content</span>
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
            title="View original content"
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
