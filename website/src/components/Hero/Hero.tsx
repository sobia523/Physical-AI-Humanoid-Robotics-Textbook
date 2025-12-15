import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './Hero.module.css'; // Assuming a CSS module for styling

function Hero(): JSX.Element {
  return (
    <header className={clsx('hero-section', styles.heroSection)}>
      <div className={clsx('container', styles.heroContainer)}>
        <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics — The Complete Guide</h1>
        <p className={styles.heroSubtitle}>
          A structured, modular, engineering-focused book for modern robotics development.
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--primary button--lg', styles.buttonPrimary)}
            to="/docs/tutorial-basics/create-a-document"> {/* Placeholder link */}
            Start Reading
          </Link>
          <Link
            className={clsx('button button--outline button--lg', styles.buttonOutline)}
            to="/docs"> {/* Placeholder link */}
            View Modules
          </Link>
        </div>
        <p className={styles.heroTag}>Trusted by learners & developers</p>
      </div>
    </header>
  );
}

export default Hero;
