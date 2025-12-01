import React, { type ReactElement } from 'react';
import { useScrollAnimation } from '../../../hooks/useScrollAnimation';
import styles from './styles.module.css';

interface FeatureCardProps {
  icon: string;
  title: string;
  description: string;
  delay?: number;
}

export default function FeatureCard({
  icon,
  title,
  description,
  delay = 0,
}: FeatureCardProps): ReactElement {
  const [ref, isVisible] = useScrollAnimation<HTMLDivElement>({
    threshold: 0.2,
  });

  return (
    <div
      ref={ref}
      className={`${styles.featureCard} ${isVisible ? styles.visible : ''}`}
      style={{ animationDelay: `${delay}ms` }}
    >
      <div className={styles.iconWrapper}>
        <span className={styles.icon} aria-hidden="true">{icon}</span>
        <div className={styles.iconGlow} aria-hidden="true" />
      </div>
      <h3 className={styles.title}>{title}</h3>
      <p className={styles.description}>{description}</p>
      <div className={styles.cardBorder} aria-hidden="true" />
    </div>
  );
}
