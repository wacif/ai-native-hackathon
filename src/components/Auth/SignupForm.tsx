/**
 * Signup Form Component
 * Multi-step form collecting user info and personalization preferences
 */
import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './Auth.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
  onSwitchToLogin?: () => void;
}

// Options for multi-select fields
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

export default function SignupForm({ onSuccess, onSwitchToLogin }: SignupFormProps) {
  const { signUp, isLoading, error, clearError } = useAuth();

  const [formData, setFormData] = useState({
    username: '',
    email: '',
    password: '',
    confirmPassword: '',
    software_background: '',
    hardware_background: '',
    programming_languages: [] as string[],
    operating_system: '',
    learning_goals: [] as string[],
    preferred_explanation_style: '',
    prior_knowledge: [] as string[],
    industry: '',
  });
  const [localError, setLocalError] = useState<string | null>(null);
  const [step, setStep] = useState<1 | 2 | 3>(1);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    setLocalError(null);
    clearError();
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
  };

  const handleStep1Submit = (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!formData.username.trim()) {
      setLocalError('Username is required');
      return;
    }
    if (!formData.email.trim()) {
      setLocalError('Email is required');
      return;
    }
    if (formData.password.length < 8) {
      setLocalError('Password must be at least 8 characters');
      return;
    }
    if (!/[A-Z]/.test(formData.password)) {
      setLocalError('Password must contain at least one uppercase letter');
      return;
    }
    if (!/[a-z]/.test(formData.password)) {
      setLocalError('Password must contain at least one lowercase letter');
      return;
    }
    if (!/[0-9]/.test(formData.password)) {
      setLocalError('Password must contain at least one digit');
      return;
    }
    if (formData.password !== formData.confirmPassword) {
      setLocalError('Passwords do not match');
      return;
    }

    setStep(2);
  };

  const handleStep2Submit = (e: React.FormEvent) => {
    e.preventDefault();
    setStep(3);
  };

  const handleFinalSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    const result = await signUp({
      username: formData.username,
      email: formData.email,
      password: formData.password,
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
      onSuccess?.();
    }
  };

  const displayError = localError || error;

  const renderStepIndicator = () => (
    <div className={styles.stepIndicator}>
      <div className={`${styles.stepDot} ${step >= 1 ? styles.active : ''}`}>1</div>
      <div className={`${styles.stepLine} ${step >= 2 ? styles.active : ''}`} />
      <div className={`${styles.stepDot} ${step >= 2 ? styles.active : ''}`}>2</div>
      <div className={`${styles.stepLine} ${step >= 3 ? styles.active : ''}`} />
      <div className={`${styles.stepDot} ${step >= 3 ? styles.active : ''}`}>3</div>
    </div>
  );

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        {renderStepIndicator()}
        
        <h2 className={styles.title}>
          {step === 1 && 'Create Account'}
          {step === 2 && 'Your Background'}
          {step === 3 && 'Learning Preferences'}
        </h2>
        <p className={styles.subtitle}>
          {step === 1 && 'Join the Physical AI & Robotics learning community'}
          {step === 2 && 'Help us understand your experience level'}
          {step === 3 && 'Customize your learning experience'}
        </p>

        {displayError && (
          <div className={styles.errorMessage}>{displayError}</div>
        )}

        {step === 1 && (
          <form onSubmit={handleStep1Submit} className={styles.form}>
            <div className={styles.inputGroup}>
              <label htmlFor="username">Username</label>
              <input
                type="text"
                id="username"
                name="username"
                value={formData.username}
                onChange={handleChange}
                placeholder="Choose a username"
                required
              />
            </div>

            <div className={styles.inputGroup}>
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                placeholder="your@email.com"
                required
              />
            </div>

            <div className={styles.inputGroup}>
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                placeholder="Min. 8 chars, upper, lower, digit"
                required
              />
            </div>

            <div className={styles.inputGroup}>
              <label htmlFor="confirmPassword">Confirm Password</label>
              <input
                type="password"
                id="confirmPassword"
                name="confirmPassword"
                value={formData.confirmPassword}
                onChange={handleChange}
                placeholder="Confirm your password"
                required
              />
            </div>

            <button type="submit" className={styles.submitButton}>
              Continue
            </button>
          </form>
        )}

        {step === 2 && (
          <form onSubmit={handleStep2Submit} className={styles.form}>
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
                We'll provide OS-specific terminal commands
              </span>
            </div>

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

            <div className={styles.inputGroup}>
              <label>Programming Languages You Know</label>
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
            </div>

            <div className={styles.buttonGroup}>
              <button 
                type="button" 
                className={styles.secondaryButton}
                onClick={() => setStep(1)}
              >
                Back
              </button>
              <button type="submit" className={styles.submitButton}>
                Continue
              </button>
            </div>
          </form>
        )}

        {step === 3 && (
          <form onSubmit={handleFinalSubmit} className={styles.form}>
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
              <label>What Do You Want to Learn?</label>
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
            </div>

            <div className={styles.inputGroup}>
              <label>Topics You Already Know</label>
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

            <div className={styles.buttonGroup}>
              <button 
                type="button" 
                className={styles.secondaryButton}
                onClick={() => setStep(2)}
              >
                Back
              </button>
              <button 
                type="submit" 
                className={styles.submitButton}
                disabled={isLoading}
              >
                {isLoading ? 'Creating Account...' : 'Create Account'}
              </button>
            </div>
          </form>
        )}

        <div className={styles.footer}>
          Already have an account?{' '}
          <button 
            type="button" 
            className={styles.linkButton}
            onClick={onSwitchToLogin}
          >
            Sign In
          </button>
        </div>
      </div>
    </div>
  );
}

