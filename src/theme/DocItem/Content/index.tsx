import React, { useState, useEffect, useCallback } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { AuthProvider, useAuth } from '@site/src/context/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import styles from './Content.module.css';

type Props = WrapperProps<typeof ContentType>;

function PersonalizeControls(): React.ReactElement | null {
  const { isAuthenticated, user, token } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const location = useLocation();
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [isUrduMode, setIsUrduMode] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [chapterId, setChapterId] = useState<string>('');
  const [mounted, setMounted] = useState(false);

  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';
  const BASE_URL = siteConfig.baseUrl || '/ai-native-hackathon/';

  useEffect(() => {
    setMounted(true);
    const path = window.location.pathname;
    
    // Match doc paths - capture the last segment as chapterId
    // Path format: /ai-native-hackathon/docs/physical-ai/intro
    const matches = path.match(/\/physical-ai\/([^\/]+)\/?$/);
    if (matches) {
      setChapterId(matches[1]);
    } else {
      setChapterId('');
    }
    
    // Reset Urdu mode when navigating to a new chapter
    setIsUrduMode(false);
  }, [location.pathname]);

  const hasPreferences = user && (
    (user.programming_languages && user.programming_languages.length > 0) ||
    user.operating_system ||
    (user.learning_goals && user.learning_goals.length > 0) ||
    user.preferred_explanation_style ||
    (user.prior_knowledge && user.prior_knowledge.length > 0) ||
    user.industry
  );

  const handlePersonalize = useCallback(async () => {
    if (!token || !chapterId) return;

    setIsLoading(true);
    setError(null);

    try {
      // Try multiple selectors to find the article content
      const articleElement = document.querySelector('.markdown') || 
                            document.querySelector('article') ||
                            document.querySelector('[class*="docItemCol"]') ||
                            document.querySelector('.theme-doc-markdown');
      if (!articleElement) {
        throw new Error('Could not find article content');
      }

      const originalContent = articleElement.innerHTML;

      const response = await fetch(`${API_URL}/personalize-chapter`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_content: articleElement.textContent || '',
          force_refresh: false,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      sessionStorage.setItem(`original_${chapterId}`, originalContent);
      
      // Use the markdown converter for proper formatting
      const htmlContent = convertMarkdownToHtml(data.personalized_content);
      
      articleElement.innerHTML = htmlContent;
      setIsPersonalized(true);
    } catch (err) {
      console.error('Error personalizing content:', err);
      setError('Failed to personalize content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  }, [API_URL, chapterId, token]);

  const handleRevert = useCallback(() => {
    const original = sessionStorage.getItem(`original_${chapterId}`);
    if (original) {
      // Try multiple selectors to find the article content
      const articleElement = document.querySelector('.markdown') || 
                            document.querySelector('article') ||
                            document.querySelector('[class*="docItemCol"]') ||
                            document.querySelector('.theme-doc-markdown');
      if (articleElement) {
        articleElement.innerHTML = original;
        setIsPersonalized(false);
        sessionStorage.removeItem(`original_${chapterId}`);
      }
    } else {
      window.location.reload();
    }
  }, [chapterId]);

  const handleLanguageSwitch = useCallback(async () => {
    // Try multiple selectors to find the article content
    const articleElement = document.querySelector('.markdown') || 
                          document.querySelector('article') ||
                          document.querySelector('[class*="docItemCol"]') ||
                          document.querySelector('.theme-doc-markdown');
    
    if (!articleElement || !chapterId) {
      console.log('Early return - articleElement:', !!articleElement, 'chapterId:', chapterId);
      return;
    }

    if (isUrduMode) {
      // Switch back to English - restore original content
      const original = sessionStorage.getItem(`english_${chapterId}`);
      if (original) {
        articleElement.innerHTML = original;
        // Reset to LTR for English
        articleElement.setAttribute('dir', 'ltr');
        (articleElement as HTMLElement).style.textAlign = 'left';
      } else {
        window.location.reload();
      }
      setIsUrduMode(false);
      return;
    }

    // Switch to Urdu - fetch the Urdu markdown file from static folder
    setIsTranslating(true);
    setError(null);

    try {
      // Save English content first
      sessionStorage.setItem(`english_${chapterId}`, articleElement.innerHTML);

      // Fetch the Urdu markdown file from static translations folder
      const urduUrl = `${BASE_URL}translations/ur/${chapterId}.md`;
      const response = await fetch(urduUrl);
      
      if (!response.ok) {
        throw new Error('Urdu translation not available for this chapter');
      }

      const markdown = await response.text();
      
      // Convert markdown to HTML
      const htmlContent = convertMarkdownToHtml(markdown);
      
      articleElement.innerHTML = htmlContent;
      // Add RTL styling for Urdu content
      articleElement.setAttribute('dir', 'rtl');
      (articleElement as HTMLElement).style.textAlign = 'right';

      setIsUrduMode(true);
    } catch (err) {
      console.error('Error loading Urdu content:', err);
      setError('اردو ترجمہ دستیاب نہیں ہے (Urdu translation not available)');
      // Restore English content
      const original = sessionStorage.getItem(`english_${chapterId}`);
      if (original) {
        articleElement.innerHTML = original;
      }
    } finally {
      setIsTranslating(false);
    }
  }, [chapterId, isUrduMode, BASE_URL]);

  // Escape HTML entities
  const escapeHtml = (text: string): string => {
    return text
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  };

  // Syntax highlighter for code blocks
  const highlightSyntax = (code: string, lang: string): string => {
    const escaped = escapeHtml(code);
    const language = lang.toLowerCase();
    
    // Define token patterns and colors for VS Code Dark+ theme
    const highlight = (text: string, patterns: Array<{regex: RegExp, color: string}>): string => {
      // Create placeholders to avoid double-matching
      const placeholders: string[] = [];
      let result = text;
      
      patterns.forEach(({regex, color}) => {
        result = result.replace(regex, (match) => {
          const index = placeholders.length;
          placeholders.push(`<span style="color:${color}">${match}</span>`);
          return `__HIGHLIGHT_${index}__`;
        });
      });
      
      // Restore placeholders
      placeholders.forEach((placeholder, index) => {
        result = result.replace(`__HIGHLIGHT_${index}__`, placeholder);
      });
      
      return result;
    };
    
    // Python highlighting
    if (language === 'python' || language === 'py') {
      return highlight(escaped, [
        // Strings (single and double quotes, including multi-line)
        {regex: /(["'])(?:(?!\1)[^\\]|\\.)*\1|(&quot;|&#039;)(?:(?!\2)[^\\]|\\.)*\2/g, color: '#ce9178'},
        // Comments
        {regex: /#.*/g, color: '#6a9955'},
        // Keywords
        {regex: /\b(def|class|if|elif|else|for|while|try|except|finally|with|as|import|from|return|yield|raise|break|continue|pass|lambda|and|or|not|in|is|True|False|None|async|await|global|nonlocal)\b/g, color: '#569cd6'},
        // Built-in functions
        {regex: /\b(print|len|range|str|int|float|list|dict|set|tuple|bool|type|input|open|file|abs|all|any|bin|chr|dir|enumerate|eval|exec|filter|format|getattr|hasattr|hash|hex|id|isinstance|iter|map|max|min|next|oct|ord|pow|repr|reversed|round|setattr|slice|sorted|sum|super|vars|zip)\b(?=\s*\()/g, color: '#dcdcaa'},
        // Decorators
        {regex: /@\w+/g, color: '#dcdcaa'},
        // Numbers
        {regex: /\b(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?\b/g, color: '#b5cea8'},
        // Self
        {regex: /\bself\b/g, color: '#9cdcfe'},
        // Function/method calls
        {regex: /\b([a-zA-Z_]\w*)\s*(?=\()/g, color: '#dcdcaa'},
      ]);
    }
    
    // JavaScript/TypeScript highlighting
    if (language === 'javascript' || language === 'js' || language === 'typescript' || language === 'ts' || language === 'jsx' || language === 'tsx') {
      return highlight(escaped, [
        // Strings
        {regex: /(["'`])(?:(?!\1)[^\\]|\\.)*\1|(&quot;|&#039;)(?:(?!\2)[^\\]|\\.)*\2/g, color: '#ce9178'},
        // Comments
        {regex: /\/\/.*/g, color: '#6a9955'},
        // Keywords
        {regex: /\b(const|let|var|function|return|if|else|for|while|do|switch|case|break|continue|try|catch|finally|throw|new|this|class|extends|super|import|export|from|default|async|await|yield|typeof|instanceof|in|of|void|null|undefined|true|false)\b/g, color: '#569cd6'},
        // Types (TypeScript)
        {regex: /\b(string|number|boolean|any|void|never|unknown|object|Array|Promise|Record|Partial|Required|Pick|Omit)\b/g, color: '#4ec9b0'},
        // Numbers
        {regex: /\b(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?\b/g, color: '#b5cea8'},
        // Function calls
        {regex: /\b([a-zA-Z_$]\w*)\s*(?=\()/g, color: '#dcdcaa'},
        // Properties
        {regex: /\.([a-zA-Z_$]\w*)/g, color: '#9cdcfe'},
      ]);
    }
    
    // Bash/Shell highlighting
    if (language === 'bash' || language === 'sh' || language === 'shell' || language === 'zsh') {
      return highlight(escaped, [
        // Strings
        {regex: /(["'])(?:(?!\1)[^\\]|\\.)*\1|(&quot;|&#039;)(?:(?!\2)[^\\]|\\.)*\2/g, color: '#ce9178'},
        // Comments
        {regex: /#.*/g, color: '#6a9955'},
        // Keywords
        {regex: /\b(if|then|else|elif|fi|for|while|do|done|case|esac|in|function|return|exit|break|continue|export|source|alias|unalias|local|declare|readonly|shift|trap)\b/g, color: '#569cd6'},
        // Common commands
        {regex: /\b(echo|cd|ls|pwd|mkdir|rm|cp|mv|cat|grep|sed|awk|find|chmod|chown|sudo|apt|yum|pip|npm|git|docker|curl|wget|tar|zip|unzip|ssh|scp|kill|ps|top|df|du|head|tail|sort|uniq|wc|xargs|tee)\b/g, color: '#dcdcaa'},
        // Variables
        {regex: /\$\{?\w+\}?/g, color: '#9cdcfe'},
        // Flags
        {regex: /\s(-{1,2}[\w-]+)/g, color: '#ce9178'},
      ]);
    }
    
    // JSON highlighting
    if (language === 'json') {
      return highlight(escaped, [
        // Property names
        {regex: /(&quot;|")([^"\\]|\\.)*(&quot;|")(?=\s*:)/g, color: '#9cdcfe'},
        // String values
        {regex: /:\s*(&quot;|")([^"\\]|\\.)*(&quot;|")/g, color: '#ce9178'},
        // Numbers
        {regex: /:\s*(-?\d+\.?\d*)/g, color: '#b5cea8'},
        // Booleans and null
        {regex: /\b(true|false|null)\b/g, color: '#569cd6'},
      ]);
    }
    
    // YAML highlighting
    if (language === 'yaml' || language === 'yml') {
      return highlight(escaped, [
        // Comments
        {regex: /#.*/g, color: '#6a9955'},
        // Keys
        {regex: /^[\s-]*([a-zA-Z_][\w-]*)\s*:/gm, color: '#9cdcfe'},
        // Strings
        {regex: /(["'])(?:(?!\1)[^\\]|\\.)*\1/g, color: '#ce9178'},
        // Numbers
        {regex: /:\s*(-?\d+\.?\d*)\s*$/gm, color: '#b5cea8'},
        // Booleans
        {regex: /\b(true|false|yes|no|on|off)\b/gi, color: '#569cd6'},
      ]);
    }
    
    // HTML highlighting
    if (language === 'html' || language === 'xml') {
      return highlight(escaped, [
        // Comments
        {regex: /&lt;!--[\s\S]*?--&gt;/g, color: '#6a9955'},
        // Tags
        {regex: /&lt;\/?[\w-]+/g, color: '#569cd6'},
        {regex: /\/?&gt;/g, color: '#569cd6'},
        // Attributes
        {regex: /\s([\w-]+)(?==)/g, color: '#9cdcfe'},
        // Attribute values
        {regex: /=(&quot;|")([^"]*?)(&quot;|")/g, color: '#ce9178'},
      ]);
    }
    
    // CSS highlighting
    if (language === 'css' || language === 'scss' || language === 'sass') {
      return highlight(escaped, [
        // Comments
        {regex: /\/\*[\s\S]*?\*\//g, color: '#6a9955'},
        // Selectors
        {regex: /^[\s]*([.#]?[\w-]+)/gm, color: '#d7ba7d'},
        // Properties
        {regex: /([\w-]+)\s*:/g, color: '#9cdcfe'},
        // Values with units
        {regex: /:\s*([\d.]+)(px|em|rem|%|vh|vw|deg|s|ms)/g, color: '#b5cea8'},
        // Colors
        {regex: /#[a-fA-F0-9]{3,8}\b/g, color: '#ce9178'},
        // Important
        {regex: /!important/g, color: '#569cd6'},
      ]);
    }
    
    // SQL highlighting
    if (language === 'sql') {
      return highlight(escaped, [
        // Comments
        {regex: /--.*/g, color: '#6a9955'},
        // Strings
        {regex: /(')(?:(?!\1)[^\\]|\\.)*\1/g, color: '#ce9178'},
        // Keywords
        {regex: /\b(SELECT|FROM|WHERE|INSERT|INTO|UPDATE|DELETE|CREATE|DROP|ALTER|TABLE|INDEX|VIEW|JOIN|LEFT|RIGHT|INNER|OUTER|ON|AND|OR|NOT|IN|IS|NULL|AS|ORDER|BY|GROUP|HAVING|LIMIT|OFFSET|UNION|ALL|DISTINCT|COUNT|SUM|AVG|MAX|MIN|CASE|WHEN|THEN|ELSE|END|PRIMARY|KEY|FOREIGN|REFERENCES|CONSTRAINT|DEFAULT|VALUES|SET)\b/gi, color: '#569cd6'},
        // Numbers
        {regex: /\b(\d+\.?\d*)\b/g, color: '#b5cea8'},
      ]);
    }
    
    // C/C++ highlighting
    if (language === 'c' || language === 'cpp' || language === 'c++') {
      return highlight(escaped, [
        // Strings
        {regex: /(["'])(?:(?!\1)[^\\]|\\.)*\1/g, color: '#ce9178'},
        // Comments
        {regex: /\/\/.*/g, color: '#6a9955'},
        // Preprocessor
        {regex: /#\w+/g, color: '#c586c0'},
        // Keywords
        {regex: /\b(int|char|float|double|void|long|short|unsigned|signed|const|static|extern|auto|register|volatile|struct|union|enum|typedef|sizeof|return|if|else|for|while|do|switch|case|default|break|continue|goto|class|public|private|protected|virtual|override|new|delete|this|template|typename|namespace|using|try|catch|throw)\b/g, color: '#569cd6'},
        // Types
        {regex: /\b(size_t|int8_t|int16_t|int32_t|int64_t|uint8_t|uint16_t|uint32_t|uint64_t|bool|string|vector|map|set|list|array|shared_ptr|unique_ptr)\b/g, color: '#4ec9b0'},
        // Numbers
        {regex: /\b(\d+\.?\d*[fFlL]?|\d+[xX][0-9a-fA-F]+)\b/g, color: '#b5cea8'},
        // Function calls
        {regex: /\b([a-zA-Z_]\w*)\s*(?=\()/g, color: '#dcdcaa'},
      ]);
    }
    
    // Rust highlighting  
    if (language === 'rust' || language === 'rs') {
      return highlight(escaped, [
        // Strings
        {regex: /(["'])(?:(?!\1)[^\\]|\\.)*\1/g, color: '#ce9178'},
        // Comments
        {regex: /\/\/.*/g, color: '#6a9955'},
        // Keywords
        {regex: /\b(fn|let|mut|const|static|struct|enum|impl|trait|type|pub|mod|use|crate|self|super|where|for|loop|while|if|else|match|return|break|continue|move|ref|async|await|unsafe|extern|dyn)\b/g, color: '#569cd6'},
        // Types
        {regex: /\b(i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize|f32|f64|bool|char|str|String|Vec|Option|Result|Box|Rc|Arc|Cell|RefCell|HashMap|HashSet|BTreeMap|BTreeSet)\b/g, color: '#4ec9b0'},
        // Macros
        {regex: /\b\w+!/g, color: '#dcdcaa'},
        // Numbers
        {regex: /\b(\d+\.?\d*[fF]?|\d+[xX][0-9a-fA-F]+)\b/g, color: '#b5cea8'},
        // Lifetimes
        {regex: /'[a-zA-Z_]\w*/g, color: '#569cd6'},
      ]);
    }
    
    // Go highlighting
    if (language === 'go' || language === 'golang') {
      return highlight(escaped, [
        // Strings
        {regex: /(["'`])(?:(?!\1)[^\\]|\\.)*\1/g, color: '#ce9178'},
        // Comments
        {regex: /\/\/.*/g, color: '#6a9955'},
        // Keywords
        {regex: /\b(package|import|func|return|var|const|type|struct|interface|map|chan|go|select|case|default|if|else|for|range|switch|break|continue|fallthrough|goto|defer|panic|recover)\b/g, color: '#569cd6'},
        // Types
        {regex: /\b(int|int8|int16|int32|int64|uint|uint8|uint16|uint32|uint64|float32|float64|complex64|complex128|byte|rune|string|bool|error|any)\b/g, color: '#4ec9b0'},
        // Built-in functions
        {regex: /\b(append|cap|close|complex|copy|delete|imag|len|make|new|panic|print|println|real|recover)\b(?=\s*\()/g, color: '#dcdcaa'},
        // Numbers
        {regex: /\b(\d+\.?\d*|\d+[xX][0-9a-fA-F]+)\b/g, color: '#b5cea8'},
      ]);
    }
    
    // Default - no highlighting, just escaped
    return escaped;
  };

  // Simple markdown to HTML converter
  const convertMarkdownToHtml = (markdown: string): string => {
    // Remove frontmatter (---...---)
    let content = markdown.replace(/^---[\s\S]*?---\n*/m, '');
    
    // Convert code blocks first (to protect them from other transformations)
    const codeBlocks: string[] = [];
    content = content.replace(/```(\w+)?\n([\s\S]*?)```/g, (_, lang, code) => {
      const index = codeBlocks.length;
      // Apply syntax highlighting based on language
      const highlightedCode = highlightSyntax(code.trim(), lang || '');
      codeBlocks.push(
        `<pre style="background:#1e1e1e;color:#d4d4d4;padding:16px;border-radius:8px;overflow-x:auto;margin:1rem 0;font-family:Consolas,Monaco,'Courier New',monospace;font-size:14px;line-height:1.6;"><code>${highlightedCode}</code></pre>`
      );
      return `__CODE_BLOCK_${index}__`;
    });
    
    // Convert inline code
    content = content.replace(/`([^`]+)`/g, '<code style="background:#3c3c3c;color:#d4d4d4;padding:2px 6px;border-radius:4px;font-family:Consolas,Monaco,monospace;">$1</code>');
    
    // Convert headers
    content = content.replace(/^### (.*$)/gm, '<h3>$1</h3>');
    content = content.replace(/^## (.*$)/gm, '<h2>$1</h2>');
    content = content.replace(/^# (.*$)/gm, '<h1>$1</h1>');
    
    // Convert bold and italic
    content = content.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
    content = content.replace(/\*([^*]+)\*/g, '<em>$1</em>');
    
    // Convert lists
    content = content.replace(/^\s*[-*]\s+(.*)$/gm, '<li>$1</li>');
    content = content.replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>');
    
    // Convert numbered lists
    content = content.replace(/^\s*\d+\.\s+(.*)$/gm, '<li>$1</li>');
    
    // Convert paragraphs (double newlines)
    content = content.replace(/\n\n+/g, '</p><p>');
    content = '<p>' + content + '</p>';
    
    // Clean up empty paragraphs
    content = content.replace(/<p>\s*<\/p>/g, '');
    content = content.replace(/<p>\s*(<h[1-6]>)/g, '$1');
    content = content.replace(/(<\/h[1-6]>)\s*<\/p>/g, '$1');
    content = content.replace(/<p>\s*(<ul>)/g, '$1');
    content = content.replace(/(<\/ul>)\s*<\/p>/g, '$1');
    content = content.replace(/<p>\s*(<pre)/g, '$1');
    content = content.replace(/(<\/pre>)\s*<\/p>/g, '$1');
    
    // Restore code blocks
    codeBlocks.forEach((block, index) => {
      content = content.replace(`__CODE_BLOCK_${index}__`, block);
    });
    
    return content;
  };

  // Don't render on server
  if (!mounted) {
    return null;
  }

  // Don't show for non-chapter pages
  if (!chapterId) {
    return null;
  }

  return (
    <div className={styles.personalizeContainer}>
      {error && <div className={styles.error}>{error}</div>}
      
      {/* Language Switcher - only for authenticated users */}
      {isAuthenticated && (
        <button
          className={`${styles.translateButton} ${isUrduMode ? styles.active : ''}`}
          onClick={handleLanguageSwitch}
          disabled={isTranslating}
          title={isUrduMode ? 'Switch to English' : 'اردو میں پڑھیں (Read in Urdu)'}
        >
          {isTranslating ? (
            <>
              <span className={styles.spinner}></span>
              Loading...
            </>
          ) : (
            <>
              <span className={styles.icon}>🌐</span>
              {isUrduMode ? 'English' : 'اردو'}
            </>
          )}
        </button>
      )}

      {/* Personalization Controls - only for authenticated users with preferences */}
      {isAuthenticated && hasPreferences && (
        <>
          {isPersonalized ? (
            <div className={styles.personalizedBadge}>
              <span className={styles.checkmark}>✨</span>
              <span>Content personalized for you</span>
              <button
                className={styles.revertButton}
                onClick={handleRevert}
              >
                View Original
              </button>
            </div>
          ) : (
            <button
              className={styles.personalizeButton}
              onClick={handlePersonalize}
              disabled={isLoading}
              title="Personalize content based on your profile"
            >
              {isLoading ? (
                <>
                  <span className={styles.spinner}></span>
                  Personalizing...
                </>
              ) : (
                <>
                  <span className={styles.icon}>✨</span>
                  Personalize for me
                </>
              )}
            </button>
          )}
        </>
      )}

      {/* Show sign-in hint for non-authenticated users */}
      {!isAuthenticated && (
        <div className={styles.noPreferences}>
          <span className={styles.icon}>🔒</span>
          <span>Sign in to access translation & personalization</span>
        </div>
      )}

      {/* Show profile hint for logged-in users without preferences */}
      {isAuthenticated && !hasPreferences && (
        <div className={styles.noPreferences}>
          <span className={styles.icon}>⚙️</span>
          <span>Set preferences to personalize</span>
        </div>
      )}
      
      {user && hasPreferences && (
        <div className={styles.profileHint}>
          {user.operating_system && (
            <span className={styles.tag}>{user.operating_system.toUpperCase()}</span>
          )}
          {user.programming_languages && user.programming_languages.slice(0, 2).map(lang => (
            <span key={lang} className={styles.tag}>{lang}</span>
          ))}
          {user.preferred_explanation_style && (
            <span className={styles.tag}>{user.preferred_explanation_style}</span>
          )}
        </div>
      )}
    </div>
  );
}

export default function ContentWrapper(props: Props): React.ReactElement {
  return (
    <>
      <AuthProvider>
        <PersonalizeControls />
      </AuthProvider>
      <Content {...props} />
    </>
  );
}
