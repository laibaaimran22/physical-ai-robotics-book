import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Fundamentals',
    Svg: require('@site/static/img/undraw_robotics.svg').default,
    description: (
      <>
        Explore the core principles of Physical AI - the integration of artificial intelligence with physical systems,
        embodied cognition, and real-world interaction. Learn how AI agents can understand and manipulate the physical world.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    Svg: require('@site/static/img/undraw_learning.svg').default,
    description: (
      <>
        Master the design, control, and intelligence of humanoid robots. From kinematics and dynamics to gait planning,
        sensor integration, and human-robot interaction for anthropomorphic systems.
      </>
    ),
  },
  {
    title: 'ROS2 & AI Integration',
    Svg: require('@site/static/img/undraw_ai_brain.svg').default,
    description: (
      <>
        Learn how to integrate modern AI techniques with ROS2 frameworks. Build intelligent robotic systems using
        Python agents, perception systems, and cognitive architectures.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className={clsx('text--center padding-horiz--md', styles['feature-container'])}>
        <Heading as="h3" className={styles['feature-title']}>{title}</Heading>
        <p className={styles['feature-description']}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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