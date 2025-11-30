import React, { useState, useEffect, useCallback } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { AuthProvider, useAuth } from '@site/src/context/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import styles from './Content.module.css';

type Props = WrapperProps<typeof ContentType>;

function PersonalizeControls(): React.ReactElement | null {
  const { isAuthenticated, user, token } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const location = useLocation();
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [isUrduMode, setIsUrduMode] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [chapterId, setChapterId] = useState<string>('');
  const [mounted, setMounted] = useState(false);

  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';
  const BASE_URL = siteConfig.baseUrl || '/ai-native-hackathon/';

  useEffect(() => {
    setMounted(true);
    const path = window.location.pathname;
    
    // Match doc paths - capture the last segment as chapterId
    // Path format: /ai-native-hackathon/docs/physical-ai/intro
    const matches = path.match(/\/physical-ai\/([^\/]+)\/?$/);
    if (matches) {
      setChapterId(matches[1]);
    } else {
      setChapterId('');
    }
    
    // Reset Urdu mode when navigating to a new chapter
    setIsUrduMode(false);
  }, [location.pathname]);

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

  const handleLanguageSwitch = useCallback(async () => {
    // Try multiple selectors to find the article content
    const articleElement = document.querySelector('.markdown') || 
                          document.querySelector('article') ||
                          document.querySelector('[class*="docItemCol"]') ||
                          document.querySelector('.theme-doc-markdown');
    
    if (!articleElement || !chapterId) {
      console.log('Early return - articleElement:', !!articleElement, 'chapterId:', chapterId);
      return;
    }

    if (isUrduMode) {
      // Switch back to English - restore original content
      const original = sessionStorage.getItem(`english_${chapterId}`);
      if (original) {
        articleElement.innerHTML = original;
        // Reset to LTR for English
        articleElement.setAttribute('dir', 'ltr');
        (articleElement as HTMLElement).style.textAlign = 'left';
      } else {
        window.location.reload();
      }
      setIsUrduMode(false);
      return;
    }

    // Switch to Urdu - fetch the Urdu markdown file from static folder
    setIsTranslating(true);
    setError(null);

    try {
      // Save English content first
      sessionStorage.setItem(`english_${chapterId}`, articleElement.innerHTML);

      // Fetch the Urdu markdown file from static translations folder
      const urduUrl = `${BASE_URL}translations/ur/${chapterId}.md`;
      const response = await fetch(urduUrl);
      
      if (!response.ok) {
        throw new Error('Urdu translation not available for this chapter');
      }

      const markdown = await response.text();
      
      // Convert markdown to HTML
      const htmlContent = convertMarkdownToHtml(markdown);
      
      articleElement.innerHTML = htmlContent;
      // Add RTL styling for Urdu content
      articleElement.setAttribute('dir', 'rtl');
      (articleElement as HTMLElement).style.textAlign = 'right';

      setIsUrduMode(true);
    } catch (err) {
      console.error('Error loading Urdu content:', err);
      setError('اردو ترجمہ دستیاب نہیں ہے (Urdu translation not available)');
      // Restore English content
      const original = sessionStorage.getItem(`english_${chapterId}`);
      if (original) {
        articleElement.innerHTML = original;
      }
    } finally {
      setIsTranslating(false);
    }
  }, [chapterId, isUrduMode, BASE_URL]);

  // Simple markdown to HTML converter
  const convertMarkdownToHtml = (markdown: string): string => {
    // Remove frontmatter (---...---)
    let content = markdown.replace(/^---[\s\S]*?---\n*/m, '');
    
    // Convert code blocks first (to protect them from other transformations)
    const codeBlocks: string[] = [];
    content = content.replace(/```(\w+)?\n([\s\S]*?)```/g, (_, lang, code) => {
      const index = codeBlocks.length;
      codeBlocks.push(`<pre><code class="language-${lang || ''}">${escapeHtml(code.trim())}</code></pre>`);
      return `__CODE_BLOCK_${index}__`;
    });
    
    // Convert inline code
    content = content.replace(/`([^`]+)`/g, '<code>$1</code>');
    
    // Convert headers
    content = content.replace(/^### (.*$)/gm, '<h3>$1</h3>');
    content = content.replace(/^## (.*$)/gm, '<h2>$1</h2>');
    content = content.replace(/^# (.*$)/gm, '<h1>$1</h1>');
    
    // Convert bold and italic
    content = content.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
    content = content.replace(/\*([^*]+)\*/g, '<em>$1</em>');
    
    // Convert lists
    content = content.replace(/^\s*[-*]\s+(.*)$/gm, '<li>$1</li>');
    content = content.replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>');
    
    // Convert numbered lists
    content = content.replace(/^\s*\d+\.\s+(.*)$/gm, '<li>$1</li>');
    
    // Convert paragraphs (double newlines)
    content = content.replace(/\n\n+/g, '</p><p>');
    content = '<p>' + content + '</p>';
    
    // Clean up empty paragraphs
    content = content.replace(/<p>\s*<\/p>/g, '');
    content = content.replace(/<p>\s*(<h[1-6]>)/g, '$1');
    content = content.replace(/(<\/h[1-6]>)\s*<\/p>/g, '$1');
    content = content.replace(/<p>\s*(<ul>)/g, '$1');
    content = content.replace(/(<\/ul>)\s*<\/p>/g, '$1');
    content = content.replace(/<p>\s*(<pre>)/g, '$1');
    content = content.replace(/(<\/pre>)\s*<\/p>/g, '$1');
    
    // Restore code blocks
    codeBlocks.forEach((block, index) => {
      content = content.replace(`__CODE_BLOCK_${index}__`, block);
    });
    
    return content;
  };

  const escapeHtml = (text: string): string => {
    return text
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  };

  // Don't render on server
  if (!mounted) {
    return null;
  }

  // Don't show for non-chapter pages
  if (!chapterId) {
    return null;
  }

  return (
    <div className={styles.personalizeContainer}>
      {error && <div className={styles.error}>{error}</div>}
      
      {/* Language Switcher - only for authenticated users */}
      {isAuthenticated && (
        <button
          className={`${styles.translateButton} ${isUrduMode ? styles.active : ''}`}
          onClick={handleLanguageSwitch}
          disabled={isTranslating}
          title={isUrduMode ? 'Switch to English' : 'اردو میں پڑھیں (Read in Urdu)'}
        >
          {isTranslating ? (
            <>
              <span className={styles.spinner}></span>
              Loading...
            </>
          ) : (
            <>
              <span className={styles.icon}>🌐</span>
              {isUrduMode ? 'English' : 'اردو'}
            </>
          )}
        </button>
      )}

      {/* Personalization Controls - only for authenticated users with preferences */}
      {isAuthenticated && hasPreferences && (
        <>
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
              title="Personalize content based on your profile"
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
        </>
      )}

      {/* Show sign-in hint for non-authenticated users */}
      {!isAuthenticated && (
        <div className={styles.noPreferences}>
          <span className={styles.icon}>🔒</span>
          <span>Sign in to access translation & personalization</span>
        </div>
      )}

      {/* Show profile hint for logged-in users without preferences */}
      {isAuthenticated && !hasPreferences && (
        <div className={styles.noPreferences}>
          <span className={styles.icon}>⚙️</span>
          <span>Set preferences to personalize</span>
        </div>
      )}
      
      {user && hasPreferences && (
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
