import React, { useEffect, useState, type ReactElement } from 'react';
import styles from './styles.module.css';

interface AnimatedTextProps {
  text: string;
  typingSpeed?: number;
  className?: string;
  showCursor?: boolean;
}

export default function AnimatedText({
  text,
  typingSpeed = 80,
  className = '',
  showCursor = true,
}: AnimatedTextProps): ReactElement {
  const [displayedText, setDisplayedText] = useState('');
  const [isTypingComplete, setIsTypingComplete] = useState(false);

  useEffect(() => {
    // Check for reduced motion preference
    const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    
    if (prefersReducedMotion) {
      // Show full text immediately if user prefers reduced motion
      setDisplayedText(text);
      setIsTypingComplete(true);
      return;
    }

    let currentIndex = 0;
    setDisplayedText('');
    setIsTypingComplete(false);

    const typingInterval = setInterval(() => {
      if (currentIndex < text.length) {
        setDisplayedText(text.slice(0, currentIndex + 1));
        currentIndex++;
      } else {
        setIsTypingComplete(true);
        clearInterval(typingInterval);
      }
    }, typingSpeed);

    return () => clearInterval(typingInterval);
  }, [text, typingSpeed]);

  return (
    <span className={`${styles.animatedText} ${className}`}>
      {displayedText}
      {showCursor && (
        <span 
          className={`${styles.cursor} ${isTypingComplete ? styles.blinking : ''}`}
          aria-hidden="true"
        >
          |
        </span>
      )}
    </span>
  );
}
