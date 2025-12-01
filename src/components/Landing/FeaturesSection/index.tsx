import React, { type ReactElement } from 'react';
import FeatureCard from '../FeatureCard';
import { useScrollAnimation } from '../../../hooks/useScrollAnimation';
import styles from './styles.module.css';

const features = [
  {
    icon: '🤖',
    title: 'ROS 2 Fundamentals',
    description: 'Master the Robot Operating System 2 with hands-on projects. Learn nodes, topics, services, and actions.',
  },
  {
    icon: '🎮',
    title: 'Simulation & Digital Twins',
    description: 'Build and test robots in Gazebo and NVIDIA Isaac Sim before deploying to real hardware.',
  },
  {
    icon: '🧠',
    title: 'NVIDIA Isaac Platform',
    description: 'Leverage GPU-accelerated perception, navigation, and manipulation with Isaac ROS and Isaac Lab.',
  },
  {
    icon: '👁️',
    title: 'Vision-Language-Action',
    description: 'Implement cutting-edge VLA models for robots that understand and act on natural language commands.',
  },
  {
    icon: '🔧',
    title: 'Real Hardware Integration',
    description: 'Deploy your algorithms to actual robots. Work with sensors, actuators, and embedded systems.',
  },
  {
    icon: '🚀',
    title: 'Production Deployment',
    description: 'Learn best practices for deploying AI-powered robots in real-world applications and environments.',
  },
];

export default function FeaturesSection(): ReactElement {
  const [titleRef, titleVisible] = useScrollAnimation<HTMLHeadingElement>({
    threshold: 0.3,
  });

  return (
    <section className={styles.features}>
      <div className={styles.container}>
        <h2 
          ref={titleRef} 
          className={`${styles.sectionTitle} ${titleVisible ? styles.visible : ''}`}
        >
          What You'll Learn
        </h2>
        <p className={`${styles.sectionSubtitle} ${titleVisible ? styles.visible : ''}`}>
          A comprehensive curriculum designed to take you from beginner to expert in Physical AI and Robotics
        </p>
        <div className={styles.grid}>
          {features.map((feature, index) => (
            <FeatureCard
              key={feature.title}
              icon={feature.icon}
              title={feature.title}
              description={feature.description}
              delay={index * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
