import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useColorMode } from '@docusaurus/theme-common';
import CustomSearchBar from '../SearchBar/SearchBar'; // Import CustomSearchBar

import styles from './Navbar.module.css'; // Assuming a CSS module for styling

function Navbar(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const { colorMode, setColorMode } = useColorMode();

  return (
    <nav className={clsx('navbar', styles.navbar)}>
      <div className={styles.navbarLeft}>
        <Link to="/" className={styles.navbarBrand}>
          <img src={siteConfig.themeConfig.navbar.logo.src} alt={siteConfig.themeConfig.navbar.logo.alt} className={styles.navbarLogo} />
          <span className={styles.navbarTitle}>{siteConfig.themeConfig.navbar.title}</span>
        </Link>
      </div>

      <div className={styles.navbarCenter}>
        {/* Navigation Links will go here */}
        <Link to="/" className={styles.navItem}>Home</Link>
        <Link to="/docs" className={styles.navItem}>Modules</Link>
        <Link to="/docs" className={styles.navItem}>Chapters</Link> {/* Temporarily links to docs root */}
        <Link to="/blog" className={styles.navItem}>Blog</Link>
      </div>

      <div className={styles.navbarRight}>
        {/* Search Bar */}
        <CustomSearchBar />
        {/* GitHub Icon Button */}
        <Link
          href={siteConfig.themeConfig.navbar.items.find(item => item.label === 'GitHub')?.href}
          className={styles.githubButton}
          aria-label="GitHub repository"
        >
          {/* Placeholder for GitHub icon */}
          <svg viewBox="0 0 24 24" className={styles.githubIcon}>
            <path d="M12 .297c-6.63 0-12 5.373-12 12 0 5.303 3.438 9.8 8.205 11.385.6.113.82-.258.82-.577 0-.285-.01-1.04-.015-2.04-3.338.724-4.042-1.61-4.042-1.61-.546-1.387-1.333-1.756-1.333-1.756-1.09-.745.08-.729.08-.729 1.205.084 1.838 1.237 1.838 1.237 1.07 1.835 2.809 1.305 3.492.997.108-.775.418-1.305.762-1.605-2.665-.3-5.466-1.33-5.466-5.93 0-1.31.465-2.38 1.235-3.22-.125-.3-1.425-3.035.11-3.185 0 0 1.005-.322 3.3.123 1.04-.322 2.145-.482 3.25-.482 1.105 0 2.21.16 3.25.482 2.28-1.445 3.295-.123 3.295-.123 1.535.15 0 2.885.11 3.185.77.84 1.235 1.91 1.235 3.22 0 4.61-2.805 5.625-5.475 5.92.43.37.81 1.096.81 2.22 0 1.606-.015 2.895-.015 3.28 0 .315.21.69.825.57C20.565 22.092 24 17.592 24 12.297c0-6.627-5.373-12-12-12z" />
          </svg>
        </Link>

        {/* Light/Dark Mode Toggle */}
        <button
          className={styles.colorModeToggle}
          onClick={() => setColorMode(colorMode === 'dark' ? 'light' : 'dark')}
          aria-label={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
        >
          {colorMode === 'dark' ? '☀️' : '🌙'} {/* Placeholder icons */}
        </button>
      </div>
    </nav>
  );
}

export default Navbar;