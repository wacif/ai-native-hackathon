/**
 * Signup Form Component
 * Collects user info including software/hardware background
 */
import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './Auth.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
  onSwitchToLogin?: () => void;
}

export default function SignupForm({ onSuccess, onSwitchToLogin }: SignupFormProps) {
  const { signUp, isLoading, error, clearError } = useAuth();

  const [formData, setFormData] = useState({
    username: '',
    email: '',
    password: '',
    confirmPassword: '',
    software_background: '',
    hardware_background: '',
  });
  const [localError, setLocalError] = useState<string | null>(null);
  const [step, setStep] = useState<1 | 2>(1);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    setLocalError(null);
    clearError();
  };

  const handleStep1Submit = (e: React.FormEvent) => {
    e.preventDefault();
    
    // Validate step 1
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
    if (formData.password !== formData.confirmPassword) {
      setLocalError('Passwords do not match');
      return;
    }

    setStep(2);
  };

  const handleFinalSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    const result = await signUp({
      username: formData.username,
      email: formData.email,
      password: formData.password,
      software_background: formData.software_background || undefined,
      hardware_background: formData.hardware_background || undefined,
    });

    if (result.success) {
      onSuccess?.();
    }
  };

  const displayError = localError || error;

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.title}>
          {step === 1 ? 'Create Account' : 'Your Background'}
        </h2>
        <p className={styles.subtitle}>
          {step === 1 
            ? 'Join the Physical AI & Robotics learning community'
            : 'Help us personalize your learning experience'}
        </p>

        {displayError && (
          <div className={styles.errorMessage}>{displayError}</div>
        )}

        {step === 1 ? (
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
                placeholder="Min. 8 characters"
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
        ) : (
          <form onSubmit={handleFinalSubmit} className={styles.form}>
            <div className={styles.inputGroup}>
              <label htmlFor="software_background">Software Background</label>
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
              <span className={styles.helpText}>
                This helps us tailor code examples and explanations
              </span>
            </div>

            <div className={styles.inputGroup}>
              <label htmlFor="hardware_background">Hardware Background</label>
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
              <span className={styles.helpText}>
                This helps us adjust technical depth for sensors, actuators, etc.
              </span>
            </div>

            <div className={styles.buttonGroup}>
              <button 
                type="button" 
                className={styles.secondaryButton}
                onClick={() => setStep(1)}
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

