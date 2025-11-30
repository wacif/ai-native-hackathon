/**
 * Custom Navbar Item Components
 * This file must export a default object that Docusaurus merges with its built-in component types
 */
import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import NavbarAuthButton from './NavbarAuthButton';

export default {
  ...ComponentTypes,
  'custom-authButton': NavbarAuthButton,
};
