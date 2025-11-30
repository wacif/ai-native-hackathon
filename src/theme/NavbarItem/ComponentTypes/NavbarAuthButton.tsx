import React from 'react';
import UserButton from '@site/src/components/Auth/UserButton';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * Custom Navbar Item Component for Authentication
 * This component is registered in docusaurus.config.ts as a custom navbar item
 * Location: src/theme/NavbarItem/ComponentTypes/NavbarAuthButton.tsx
 * 
 * According to Docusaurus documentation, custom navbar items should be placed
 * in the ComponentTypes directory and registered with type: 'custom' in the config.
 */
export default function NavbarAuthButton() {
  const history = useHistory();
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  const handleLoginClick = () => {
    // Use baseUrl-aware routing for proper navigation
    const loginPath = `${baseUrl}login`.replace(/\/+/g, '/'); // Remove duplicate slashes
    history.push(loginPath);
  };

  const handleSignupClick = () => {
    // Use baseUrl-aware routing for proper navigation
    const signupPath = `${baseUrl}signup`.replace(/\/+/g, '/'); // Remove duplicate slashes
    history.push(signupPath);
  };

  return (
    <UserButton
      onLoginClick={handleLoginClick}
      onSignupClick={handleSignupClick}
    />
  );
}

