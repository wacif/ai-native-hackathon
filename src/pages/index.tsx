import type { ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/Landing/HeroSection';
import FeaturesSection from '@site/src/components/Landing/FeaturesSection';
import CTASection from '@site/src/components/Landing/CTASection';

import styles from './index.module.css';

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Robotics Textbook"
      description="Master Physical AI and Robotics with ROS 2, NVIDIA Isaac, and Vision-Language-Action models. Your complete guide to building autonomous robots."
    >
      <main className={styles.main}>
        <HeroSection />
        <FeaturesSection />
        <CTASection />
      </main>
    </Layout>
  );
}
