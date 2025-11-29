import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import styles from './Chatbot.module.css';

interface Message {
  id: string;
  type: 'user' | 'bot';
  content: string;
  timestamp: Date;
  sources?: Array<{
    source: string;
    type: string;
    score: number;
    text_preview?: string;
  }>;
}

interface ChatbotProps {
  pageUrl?: string;
  chapterId?: string;
}

export default function Chatbot({ pageUrl, chapterId }: ChatbotProps) {
  const { siteConfig } = useDocusaurusContext();
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  
  // Get API URL from customFields or use localhost for development
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';

  // Auto-scroll to bottom when new messages arrive
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Listen for text selection events
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 0) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const sendMessage = async (messageText: string) => {
    if (!messageText.trim()) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      type: 'user',
      content: messageText,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Determine which endpoint to use based on whether we have selected text
      const endpoint = selectedText ? '/query-selection' : '/query';
      const requestBody = selectedText
        ? {
            question: messageText,
            selected_text: selectedText,
            page_url: pageUrl,
            chapter_id: chapterId,
          }
        : {
            question: messageText,
            page_url: pageUrl,
            chapter_id: chapterId,
          };

      const response = await fetch(`${API_URL}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        type: 'bot',
        content: data.answer,
        timestamp: new Date(),
        sources: data.sources,
      };

      setMessages(prev => [...prev, botMessage]);
      
      // Clear selected text after use
      if (selectedText) {
        setSelectedText('');
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        type: 'bot',
        content: 'Sorry, I encountered an error processing your question. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(input);
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.open : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>📚 Ask About the Content</h3>
            <p className={styles.subtitle}>AI-powered learning assistant</p>
          </div>

          {/* Selected Text Indicator */}
          {selectedText && (
            <div className={styles.selectionIndicator}>
              <span className={styles.selectionIcon}>📝</span>
              <span className={styles.selectionText}>
                Text selected: "{selectedText.substring(0, 50)}..."
              </span>
              <button
                className={styles.clearSelection}
                onClick={() => setSelectedText('')}
                aria-label="Clear selection"
              >
                ✕
              </button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.emptyState}>
                <p>👋 Hi! I'm your AI learning assistant.</p>
                <p>Ask me anything about the content on this page!</p>
                <ul className={styles.suggestions}>
                  <li>💡 "Explain this concept"</li>
                  <li>📖 "What's the main idea?"</li>
                  <li>🔍 Select text and ask specific questions</li>
                </ul>
              </div>
            ) : (
              messages.map(message => (
                <div
                  key={message.id}
                  className={`${styles.message} ${styles[message.type]}`}
                >
                  <div className={styles.messageContent}>
                    {message.type === 'bot' ? (
                      <ReactMarkdown 
                        remarkPlugins={[remarkGfm]}
                        components={{
                          // Customize markdown rendering for chatbot
                          p: ({ children }) => <p style={{ margin: '0.5em 0' }}>{children}</p>,
                          ul: ({ children }) => <ul style={{ margin: '0.5em 0', paddingLeft: '1.5em' }}>{children}</ul>,
                          ol: ({ children }) => <ol style={{ margin: '0.5em 0', paddingLeft: '1.5em' }}>{children}</ol>,
                          code: ({ node, inline, className, children, ...props }: any) => 
                            inline ? (
                              <code style={{ 
                                background: 'var(--ifm-code-background)', 
                                padding: '0.1em 0.3em',
                                borderRadius: '3px',
                                fontSize: '0.9em'
                              }} {...props}>{children}</code>
                            ) : (
                              <code style={{ 
                                display: 'block',
                                background: 'var(--ifm-code-background)',
                                padding: '0.5em',
                                borderRadius: '4px',
                                fontSize: '0.9em',
                                overflowX: 'auto'
                              }} {...props}>{children}</code>
                            )
                        }}
                      >
                        {message.content}
                      </ReactMarkdown>
                    ) : (
                      message.content
                    )}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <small>
                        Sources: {message.sources.map(s => s.type).join(', ')}
                      </small>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot} ${styles.loading}`}>
                <div className={styles.messageContent}>
                  <span className={styles.loadingDots}>Thinking</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form className={styles.inputForm} onSubmit={handleSubmit}>
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              placeholder={selectedText ? "Ask about selected text..." : "Ask a question..."}
              className={styles.input}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !input.trim()}
            >
              ↑
            </button>
          </form>
        </div>
      )}
    </>
  );
}

