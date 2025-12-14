import React from 'react';
import Layout from '@theme/Layout';
// Import custom components
// import Navbar from '../components/Navbar/Navbar'; // Navbar is configured in docusaurus.config.ts
import Hero from '../components/Hero/Hero';
import ModulesGrid from '../components/ModulesGrid/ModulesGrid';
// import SearchBar from '../components/SearchBar/SearchBar'; // SearchBar is integrated into Navbar/Hero
// import Footer from '../components/Footer/Footer'; // Footer is configured in docusaurus.config.ts

import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // For site config
import Head from '@docusaurus/Head'; // For meta tags

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description={siteConfig.tagline}>
      <Head>
        <html data-theme="light" /> {/* Default to light theme initially */}
      </Head>
      <main>
        {/* Custom components will be rendered here */}
        <Hero />
        <ModulesGrid />
      </main>
    </Layout>
  );
}