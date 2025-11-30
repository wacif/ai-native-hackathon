import React, { useEffect, useState } from 'react';
import Chatbot from '@site/src/components/Chatbot';
import { AuthProvider } from '@site/src/context/AuthContext';

/**
 * Root component wrapper for Docusaurus
 * Provides AuthProvider context and Chatbot component to all pages
 */
export default function Root({ children }) {
  const [pageUrl, setPageUrl] = useState('');
  const [chapterId, setChapterId] = useState('');

  useEffect(() => {
    // Get current page URL
    setPageUrl(window.location.pathname);

    // Extract chapter ID from URL if available
    // Example: /docs/module1/chapter1 -> chapter1
    const pathParts = window.location.pathname.split('/');
    const lastPart = pathParts[pathParts.length - 1] || pathParts[pathParts.length - 2];
    setChapterId(lastPart);

    // Update on navigation
    const handleNavigation = () => {
      setPageUrl(window.location.pathname);
      const pathParts = window.location.pathname.split('/');
      const lastPart = pathParts[pathParts.length - 1] || pathParts[pathParts.length - 2];
      setChapterId(lastPart);
    };

    window.addEventListener('popstate', handleNavigation);

    // Listen for Docusaurus navigation
    const originalPushState = window.history.pushState;
    window.history.pushState = function(...args) {
      originalPushState.apply(window.history, args);
      handleNavigation();
    };

    return () => {
      window.removeEventListener('popstate', handleNavigation);
      window.history.pushState = originalPushState;
    };
  }, []);

  return (
    <AuthProvider>
      {children}
      <Chatbot pageUrl={pageUrl} chapterId={chapterId} />
    </AuthProvider>
  );
}

