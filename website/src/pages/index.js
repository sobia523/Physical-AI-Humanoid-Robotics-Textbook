import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1-ros2-humanoid-control/chapter1">
            Start Learning - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

const FeatureList = [
  {
    title: 'Module 1: ROS 2 Humanoid Control',
    Svg: require('@site/static/img/undraw_robotics.svg').default,
    description: (
      <>
        Dive into the fundamentals of ROS 2 for controlling humanoid robots. Learn about topics, services,
        actions, and basic navigation.
      </>
    ),
    link: '/docs/module1-ros2-humanoid-control/chapter1',
  },
  {
    title: 'Module 2: Digital Twin',
    Svg: require('@site/static/img/undraw_digital_twin.svg').default,
    description: (
      <>
        Explore the creation and utilization of Digital Twins in robotics. Understand simulation
        environments like Gazebo and Unity for realistic robot behavior.
      </>
    ),
    link: '/docs/module2-digital-twin/index',
  },
  {
    title: 'Module 3: Isaac AI Robot Brain',
    Svg: require('@site/static/img/undraw_artificial_intelligence.svg').default,
    description: (
      <>
        Develop advanced AI capabilities for robots using NVIDIA Isaac Sim. Focus on perception,
        cognitive planning, and integrating AI models.
      </>
    ),
    link: '/docs/module3-isaac-ai-robot-brain/index',
  },
  {
    title: 'Module 4: VLA Robotics',
    Svg: require('@site/static/img/undraw_voice_assistant.svg').default,
    description: (
      <>
        Integrate Vision-Language Models (VLMs) with robotics for voice-commanded autonomous control.
        Learn to translate natural language into robot actions.
      </>
    ),
    link: '/docs/module4-vla-robotics/chapter1',
  },
];

function Feature({Svg, title, description, link}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3><Link to={link}>{title}</Link></h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageModules() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageModules />
      </main>
    </Layout>
  );
}
