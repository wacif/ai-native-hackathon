import React, {type ReactNode, useEffect, useRef} from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type {WrapperProps} from '@docusaurus/types';
import UserButton from '@site/src/components/Auth/UserButton';
import {AuthProvider} from '@site/src/context/AuthContext';
import {useHistory} from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {createRoot, Root} from 'react-dom/client';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): ReactNode {
  const history = useHistory();
  const {siteConfig} = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;
  const rootRef = useRef<Root | null>(null);

  useEffect(() => {
    const renderAuthButton = () => {
      // Find the right side of navbar
      const navbarItems = document.querySelector('.navbar__items--right');
      if (!navbarItems) return;

      // Check if we already added our container
      let container = document.getElementById('navbar-auth-container');
      if (!container) {
        container = document.createElement('div');
        container.id = 'navbar-auth-container';
        container.className = 'navbar__item';
        container.style.display = 'flex';
        container.style.alignItems = 'center';
        navbarItems.appendChild(container);
      }

      // Only create root once
      if (!rootRef.current) {
        rootRef.current = createRoot(container);
      }

      // Wrap UserButton with AuthProvider since it's a separate React tree
      rootRef.current.render(
        <AuthProvider>
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
        </AuthProvider>
      );
    };

    // Try multiple times to catch the container when it's ready
    const timers = [
      setTimeout(renderAuthButton, 50),
      setTimeout(renderAuthButton, 200),
      setTimeout(renderAuthButton, 500),
    ];

    return () => {
      timers.forEach(clearTimeout);
    };
  }, [history, baseUrl]);

  return <Navbar {...props} />;
}
