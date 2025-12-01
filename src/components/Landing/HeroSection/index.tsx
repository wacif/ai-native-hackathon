import React, { type ReactElement } from 'react';
import Link from '@docusaurus/Link';
import ParticleBackground from '../ParticleBackground';
import AnimatedText from '../AnimatedText';
import styles from './styles.module.css';

export default function HeroSection(): ReactElement {
  return (
    <section className={styles.hero}>
      <ParticleBackground />
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>
          <AnimatedText 
            text="Master Physical AI & Robotics" 
            typingSpeed={60}
          />
        </h1>
        <p className={styles.heroSubtitle}>
          Build autonomous robots using ROS 2, NVIDIA Isaac, and Vision-Language-Action models. 
          Your journey from simulation to real-world deployment starts here.
        </p>
        <div className={styles.heroActions}>
          <Link className={styles.primaryButton} to="/docs/physical-ai/intro">
            <span className={styles.buttonText}>Start Learning</span>
            <span className={styles.buttonGlow} aria-hidden="true" />
          </Link>
          <Link className={styles.secondaryButton} to="/docs/physical-ai/weekly-schedule">
            <span className={styles.buttonText}>View Curriculum</span>
          </Link>
        </div>
        <div className={styles.heroStats}>
          <div className={styles.stat}>
            <span className={styles.statNumber}>6</span>
            <span className={styles.statLabel}>Modules</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>12</span>
            <span className={styles.statLabel}>Weeks</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>∞</span>
            <span className={styles.statLabel}>Projects</span>
          </div>
        </div>
      </div>
      <div className={styles.scrollIndicator} aria-hidden="true">
        <div className={styles.scrollMouse}>
          <div className={styles.scrollWheel} />
        </div>
        <span className={styles.scrollText}>Scroll to explore</span>
      </div>
    </section>
  );
}
