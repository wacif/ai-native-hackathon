import React, { type ReactElement } from 'react';
import Link from '@docusaurus/Link';
import { useScrollAnimation } from '../../../hooks/useScrollAnimation';
import styles from './styles.module.css';

export default function CTASection(): ReactElement {
  const [ref, isVisible] = useScrollAnimation<HTMLElement>({
    threshold: 0.3,
  });

  return (
    <section ref={ref} className={`${styles.cta} ${isVisible ? styles.visible : ''}`}>
      <div className={styles.container}>
        <h2 className={styles.title}>Ready to Build the Future?</h2>
        <p className={styles.subtitle}>
          Join the next generation of robotics engineers. Start your journey into Physical AI today.
        </p>
        <div className={styles.actions}>
          <Link className={styles.primaryButton} to="/docs/physical-ai/intro">
            <span className={styles.buttonText}>Get Started Free</span>
            <span className={styles.buttonGlow} aria-hidden="true" />
          </Link>
          <Link className={styles.secondaryButton} to="/docs/physical-ai/hardware-setup">
            <span className={styles.buttonText}>Hardware Guide</span>
          </Link>
        </div>
        <div className={styles.highlights}>
          <div className={styles.highlight}>
            <span className={styles.checkmark}>✓</span>
            <span>Open Source Curriculum</span>
          </div>
          <div className={styles.highlight}>
            <span className={styles.checkmark}>✓</span>
            <span>Hands-on Projects</span>
          </div>
          <div className={styles.highlight}>
            <span className={styles.checkmark}>✓</span>
            <span>Industry-Ready Skills</span>
          </div>
        </div>
      </div>
      <div className={styles.backgroundGlow} aria-hidden="true" />
    </section>
  );
}
