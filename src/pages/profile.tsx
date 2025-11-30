/**
 * Profile Settings Page
 * Allows users to update their personalization preferences
 */
import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { AuthProvider, useAuth } from '../context/AuthContext';
import styles from './profile.module.css';

// Options for multi-select fields (same as SignupForm)
const PROGRAMMING_LANGUAGES = [
  'Python', 'C++', 'JavaScript', 'TypeScript', 'Rust', 'Go', 'Java', 'C#', 'MATLAB', 'Julia'
];

const LEARNING_GOALS = [
  'ROS 2 Basics', 'NVIDIA Isaac Sim', 'Robot Control', 'Computer Vision', 
  'Motion Planning', 'SLAM & Navigation', 'Reinforcement Learning', 
  'Manipulation', 'Human-Robot Interaction', 'Simulation'
];

const PRIOR_KNOWLEDGE = [
  'Python basics', 'Linux/Ubuntu', 'Docker', 'Git', 'Control Theory',
  'Linear Algebra', 'Machine Learning', 'Deep Learning', 'CAD/3D Modeling',
  'Electronics', 'Embedded Systems', 'ROS 1'
];

function ProfileContent() {
  const { user, token, isAuthenticated, isLoading: authLoading, updateProfile } = useAuth();
  const [mounted, setMounted] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [success, setSuccess] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const [formData, setFormData] = useState({
    username: '',
    email: '',
    software_background: '',
    hardware_background: '',
    programming_languages: [] as string[],
    operating_system: '',
    learning_goals: [] as string[],
    preferred_explanation_style: '',
    prior_knowledge: [] as string[],
    industry: '',
  });

  useEffect(() => {
    setMounted(true);
  }, []);

  // Populate form when user data loads
  useEffect(() => {
    if (user) {
      setFormData({
        username: user.username || '',
        email: user.email || '',
        software_background: user.software_background || '',
        hardware_background: user.hardware_background || '',
        programming_languages: user.programming_languages || [],
        operating_system: user.operating_system || '',
        learning_goals: user.learning_goals || [],
        preferred_explanation_style: user.preferred_explanation_style || '',
        prior_knowledge: user.prior_knowledge || [],
        industry: user.industry || '',
      });
    }
  }, [user]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    setError(null);
    setSuccess(null);
  };

  const handleMultiSelect = (field: 'programming_languages' | 'learning_goals' | 'prior_knowledge', value: string) => {
    setFormData(prev => {
      const current = prev[field];
      if (current.includes(value)) {
        return { ...prev, [field]: current.filter(v => v !== value) };
      } else {
        return { ...prev, [field]: [...current, value] };
      }
    });
    setError(null);
    setSuccess(null);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);
    setSuccess(null);

    try {
      const result = await updateProfile({
        software_background: formData.software_background || undefined,
        hardware_background: formData.hardware_background || undefined,
        programming_languages: formData.programming_languages.length > 0 ? formData.programming_languages : undefined,
        operating_system: formData.operating_system || undefined,
        learning_goals: formData.learning_goals.length > 0 ? formData.learning_goals : undefined,
        preferred_explanation_style: formData.preferred_explanation_style || undefined,
        prior_knowledge: formData.prior_knowledge.length > 0 ? formData.prior_knowledge : undefined,
        industry: formData.industry || undefined,
      });

      if (result.success) {
        setSuccess('Profile updated successfully! Your personalized content will reflect these changes.');
      } else {
        setError(result.error || 'Failed to update profile');
      }
    } catch (err) {
      setError('An error occurred while updating your profile');
    } finally {
      setIsLoading(false);
    }
  };

  if (!mounted) {
    return null;
  }

  if (authLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading...</div>
      </div>
    );
  }

  if (!isAuthenticated) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <h1>Profile Settings</h1>
          <p className={styles.message}>Please sign in to view and edit your profile.</p>
          <a href="/ai-native-hackathon/login" className={styles.primaryButton}>
            Sign In
          </a>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <h1>Profile Settings</h1>
        <p className={styles.subtitle}>
          Update your preferences to personalize your learning experience
        </p>

        {success && <div className={styles.successMessage}>{success}</div>}
        {error && <div className={styles.errorMessage}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.form}>
          {/* Account Info (read-only) */}
          <section className={styles.section}>
            <h2>Account Information</h2>
            <div className={styles.inputGroup}>
              <label htmlFor="username">Username</label>
              <input
                type="text"
                id="username"
                name="username"
                value={formData.username}
                disabled
                className={styles.disabledInput}
              />
            </div>
            <div className={styles.inputGroup}>
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                disabled
                className={styles.disabledInput}
              />
            </div>
          </section>

          {/* System Preferences */}
          <section className={styles.section}>
            <h2>System Preferences</h2>
            <div className={styles.inputGroup}>
              <label htmlFor="operating_system">Operating System</label>
              <select
                id="operating_system"
                name="operating_system"
                value={formData.operating_system}
                onChange={handleChange}
              >
                <option value="">Select your OS</option>
                <option value="linux">🐧 Linux (Ubuntu/Debian)</option>
                <option value="macos">🍎 macOS</option>
                <option value="windows">🪟 Windows</option>
              </select>
              <span className={styles.helpText}>
                Commands will be adapted for your operating system
              </span>
            </div>

            <div className={styles.inputGroup}>
              <label>Programming Languages</label>
              <div className={styles.chipContainer}>
                {PROGRAMMING_LANGUAGES.map(lang => (
                  <button
                    key={lang}
                    type="button"
                    className={`${styles.chip} ${formData.programming_languages.includes(lang) ? styles.selected : ''}`}
                    onClick={() => handleMultiSelect('programming_languages', lang)}
                  >
                    {lang}
                  </button>
                ))}
              </div>
              <span className={styles.helpText}>
                Code examples will prefer your selected languages
              </span>
            </div>
          </section>

          {/* Experience Level */}
          <section className={styles.section}>
            <h2>Experience Level</h2>
            <div className={styles.inputGroup}>
              <label htmlFor="software_background">Software Experience</label>
              <select
                id="software_background"
                name="software_background"
                value={formData.software_background}
                onChange={handleChange}
              >
                <option value="">Select your level</option>
                <option value="beginner">Beginner - New to programming</option>
                <option value="intermediate">Intermediate - Comfortable with coding</option>
                <option value="advanced">Advanced - Professional developer</option>
                <option value="expert">Expert - Deep systems knowledge</option>
              </select>
            </div>

            <div className={styles.inputGroup}>
              <label htmlFor="hardware_background">Hardware/Robotics Experience</label>
              <select
                id="hardware_background"
                name="hardware_background"
                value={formData.hardware_background}
                onChange={handleChange}
              >
                <option value="">Select your level</option>
                <option value="beginner">Beginner - No robotics experience</option>
                <option value="intermediate">Intermediate - Hobbyist / Arduino projects</option>
                <option value="advanced">Advanced - Worked with robots/drones</option>
                <option value="expert">Expert - Professional robotics engineer</option>
              </select>
            </div>

            <div className={styles.inputGroup}>
              <label htmlFor="industry">Your Role</label>
              <select
                id="industry"
                name="industry"
                value={formData.industry}
                onChange={handleChange}
              >
                <option value="">Select your role</option>
                <option value="student">🎓 Student</option>
                <option value="researcher">🔬 Researcher</option>
                <option value="industry">🏭 Industry Professional</option>
                <option value="hobbyist">🛠️ Hobbyist/Maker</option>
                <option value="educator">👨‍🏫 Educator</option>
              </select>
            </div>
          </section>

          {/* Learning Preferences */}
          <section className={styles.section}>
            <h2>Learning Preferences</h2>
            <div className={styles.inputGroup}>
              <label htmlFor="preferred_explanation_style">Preferred Explanation Style</label>
              <select
                id="preferred_explanation_style"
                name="preferred_explanation_style"
                value={formData.preferred_explanation_style}
                onChange={handleChange}
              >
                <option value="">Select your preference</option>
                <option value="conceptual">📚 Conceptual - Theory and concepts first</option>
                <option value="code-heavy">💻 Code-heavy - Learn by examples</option>
                <option value="visual">🎨 Visual - Diagrams and illustrations</option>
                <option value="step-by-step">📋 Step-by-step - Detailed tutorials</option>
              </select>
            </div>

            <div className={styles.inputGroup}>
              <label>Learning Goals</label>
              <div className={styles.chipContainer}>
                {LEARNING_GOALS.map(goal => (
                  <button
                    key={goal}
                    type="button"
                    className={`${styles.chip} ${formData.learning_goals.includes(goal) ? styles.selected : ''}`}
                    onClick={() => handleMultiSelect('learning_goals', goal)}
                  >
                    {goal}
                  </button>
                ))}
              </div>
              <span className={styles.helpText}>
                Content relevant to your goals will be emphasized
              </span>
            </div>

            <div className={styles.inputGroup}>
              <label>Prior Knowledge</label>
              <div className={styles.chipContainer}>
                {PRIOR_KNOWLEDGE.map(topic => (
                  <button
                    key={topic}
                    type="button"
                    className={`${styles.chip} ${formData.prior_knowledge.includes(topic) ? styles.selected : ''}`}
                    onClick={() => handleMultiSelect('prior_knowledge', topic)}
                  >
                    {topic}
                  </button>
                ))}
              </div>
              <span className={styles.helpText}>
                We'll skip explaining concepts you already know
              </span>
            </div>
          </section>

          <div className={styles.buttonContainer}>
            <button 
              type="submit" 
              className={styles.primaryButton}
              disabled={isLoading}
            >
              {isLoading ? 'Saving...' : 'Save Changes'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}

export default function ProfilePage() {
  return (
    <Layout title="Profile Settings" description="Update your personalization preferences">
      <AuthProvider>
        <ProfileContent />
      </AuthProvider>
    </Layout>
  );
}
