import React, {type ReactNode, useEffect, useRef} from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type {WrapperProps} from '@docusaurus/types';
import UserButton from '@site/src/components/Auth/UserButton';
import {useHistory} from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {createRoot} from 'react-dom/client';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): ReactNode {
  const history = useHistory();
  const {siteConfig} = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const findAndReplaceSignIn = () => {
      const navbar = document.querySelector('nav.navbar');
      if (!navbar) return;

      // Find the Sign In link
      const allLinks = Array.from(navbar.querySelectorAll('a'));
      const signInLink = allLinks.find(
        (link) => link.textContent?.trim() === 'Sign In' || link.getAttribute('href')?.includes('login')
      );

      if (signInLink) {
        // Hide the original link
        signInLink.style.display = 'none';
        
        // Find or create container for UserButton
        let container = signInLink.parentElement?.querySelector('.navbar__auth-button-container') as HTMLElement;
        
        if (!container) {
          container = document.createElement('div');
          container.className = 'navbar__item navbar__auth-button-container';
          container.style.display = 'inline-flex';
          container.style.alignItems = 'center';
          
          // Insert after the hidden link's parent
          if (signInLink.parentElement) {
            signInLink.parentElement.insertAdjacentElement('afterend', container);
          } else {
            signInLink.insertAdjacentElement('afterend', container);
          }
          
          // Render UserButton
          const root = createRoot(container);
          root.render(
            <UserButton
              onLoginClick={() => {
                const loginPath = `${baseUrl}login`.replace(/\/+/g, '/');
                history.push(loginPath);
              }}
              onSignupClick={() => {
                const signupPath = `${baseUrl}signup`.replace(/\/+/g, '/');
                history.push(signupPath);
              }}
            />
          );
        }
      }
    };

    // Try multiple times to catch the navbar when it's ready
    const timers = [
      setTimeout(findAndReplaceSignIn, 50),
      setTimeout(findAndReplaceSignIn, 200),
      setTimeout(findAndReplaceSignIn, 500),
    ];
    
    // Watch for navigation changes
    window.addEventListener('popstate', findAndReplaceSignIn);
    
    return () => {
      timers.forEach(clearTimeout);
      window.removeEventListener('popstate', findAndReplaceSignIn);
    };
  }, [history, baseUrl]);

  return <Navbar {...props} />;
}
