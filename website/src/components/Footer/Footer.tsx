import React from 'react';
import clsx from 'clsx';
import styles from './Footer.module.css'; // Assuming a CSS module for styling

function Footer(): JSX.Element {
  return (
    <footer className={clsx('footer', styles.footer)}>
      <div className={clsx('container', styles.footerContainer)}>
        <div className={styles.footerLeft}>
          © 2025 Physical AI & Humanoid Robotics
        </div>
        <div className={styles.footerRight}>
          Built with Docusaurus
        </div>
      </div>
    </footer>
  );
}

export default Footer;
