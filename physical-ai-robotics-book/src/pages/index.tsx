import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import Chatbot from '@site/src/components/Chatbot/Chatbot';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              A comprehensive guide to Physical AI and Humanoid Robotics, exploring the intersection of artificial intelligence and embodied systems.
              Learn how to build intelligent robots that can perceive, reason, and interact with the physical world.
            </p>
          </div>
          <div className={styles.heroImage}>
            <div className={styles.robotIcon}>🤖</div>
          </div>
        </div>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro"
          >
            Start Learning
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/module1-ros2-nervous-system/lesson1-1-ros2-fundamentals"
          >
            Begin Your Journey
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics - exploring the intersection of artificial intelligence and embodied systems"
    >
      <HomepageHeader />
      <main>
        <section className={styles.introSection}>
          <div className="container">
            <div className="row">
              <div className="col col--12 text--center">
                <Heading as="h2" className={styles.sectionTitle}>
                  About This Book
                </Heading>
                <p className={styles.sectionDescription}>
                  This comprehensive guide covers the essential concepts of Physical AI and Humanoid Robotics,
                  from foundational principles to advanced implementations. You'll learn how to create intelligent
                  systems that can perceive, reason, and act in the physical world.
                </p>
              </div>
            </div>
          </div>
        </section>

        <HomepageFeatures />

        <section className={styles.featuresSection}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🤖 Physical AI</h3>
                  <p>Learn about embodied cognition, sensorimotor learning, and how AI systems can interact with the physical world.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🦾 Humanoid Systems</h3>
                  <p>Master the design, control, and intelligence of humanoid robots with advanced kinematics and dynamics.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🧠 AI Integration</h3>
                  <p>Discover how to integrate modern AI techniques with ROS2 frameworks for intelligent robotic systems.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>

      <Chatbot />
    </Layout>
  );
}
