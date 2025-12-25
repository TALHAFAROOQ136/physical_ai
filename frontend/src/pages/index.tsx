import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import Hero from '../components/Hero';
import ModuleCard from '../components/ModuleCard';

const modules = [
  {
    title: 'Module 1: ROS 2',
    subtitle: 'The Robotic Nervous System',
    description: 'Learn ROS 2 middleware architecture for robot control, including nodes, topics, services, and actions.',
    icon: '🔗',
    link: '/docs/module-1-ros2',
    color: '#2563eb',
    duration: '2 weeks',
  },
  {
    title: 'Module 2: Simulation',
    subtitle: 'The Digital Twin',
    description: 'Build physics simulations with Gazebo and Unity to test robot behaviors safely.',
    icon: '🎮',
    link: '/docs/module-2-simulation',
    color: '#7c3aed',
    duration: '2 weeks',
  },
  {
    title: 'Module 3: NVIDIA Isaac',
    subtitle: 'The AI-Robot Brain',
    description: 'Leverage NVIDIA Isaac for photorealistic simulation and GPU-accelerated perception.',
    icon: '🧠',
    link: '/docs/module-3-isaac',
    color: '#059669',
    duration: '2 weeks',
  },
  {
    title: 'Module 4: VLA',
    subtitle: 'Vision-Language-Action',
    description: 'Integrate voice commands and LLMs for intuitive robot control through natural language.',
    icon: '🗣️',
    link: '/docs/module-4-vla',
    color: '#dc2626',
    duration: '2 weeks',
  },
];

function ModuleSection(): JSX.Element {
  return (
    <section className={styles.moduleSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <p className={styles.sectionSubtitle}>
          Four comprehensive modules taking you from beginner to expert
        </p>
        <div className={styles.moduleGrid}>
          {modules.map((module, index) => (
            <ModuleCard key={index} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}

function CapstoneSection(): JSX.Element {
  return (
    <section className={styles.capstoneSection}>
      <div className="container">
        <div className={styles.capstoneContent}>
          <div className={styles.capstoneText}>
            <span className={styles.capstoneLabel}>Final Project</span>
            <h2>Capstone: Build Your Humanoid</h2>
            <p>
              Put everything together in a comprehensive capstone project. Design,
              build, and demonstrate a complete humanoid robot system using ROS 2,
              simulation, and AI.
            </p>
            <Link
              className={clsx('button button--primary button--lg', styles.capstoneButton)}
              to="/docs/capstone"
            >
              View Capstone Project
            </Link>
          </div>
          <div className={styles.capstoneStats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>5</span>
              <span className={styles.statLabel}>Milestones</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>4</span>
              <span className={styles.statLabel}>Weeks</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>1</span>
              <span className={styles.statLabel}>Robot</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function FeaturesSection(): JSX.Element {
  const features = [
    {
      title: 'Hands-On Learning',
      description: 'Every chapter includes practical exercises and real code examples.',
      icon: '💻',
    },
    {
      title: 'Industry Tools',
      description: 'Learn the same tools used by robotics companies worldwide.',
      icon: '🔧',
    },
    {
      title: 'AI Integration',
      description: 'Combine traditional robotics with modern AI and LLMs.',
      icon: '🤖',
    },
    {
      title: 'Bilingual Support',
      description: 'Available in English and Urdu for wider accessibility.',
      icon: '🌍',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <span className={styles.featureIcon}>{feature.icon}</span>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description="Learn to build intelligent robots with ROS 2, Gazebo, NVIDIA Isaac, and LLMs"
    >
      <Hero />
      <main>
        <FeaturesSection />
        <ModuleSection />
        <CapstoneSection />
      </main>
    </Layout>
  );
}
