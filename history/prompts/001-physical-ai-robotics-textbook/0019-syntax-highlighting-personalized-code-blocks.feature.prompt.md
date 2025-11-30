```prompt
---
id: 0019
title: Syntax Highlighting for Personalized Code Blocks
stage: feature
date: 2025-11-30
surface: agent
model: claude-opus-4-5-20250929
feature: 001-physical-ai-robotics-textbook
branch: 001-physical-ai-robotics-textbook
user: user
command: implement feature
labels: ["syntax-highlighting", "code-blocks", "LLM", "personalization", "frontend", "backend", "feature"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/src/main.py
  - src/theme/DocItem/Content/index.tsx
tests:
  - Manual verification of syntax highlighting in personalized content
  - Manual verification of code block colors matching VS Code Dark+ theme
  - Manual verification of multiple language support (Python, JavaScript, Bash, etc.)
---

## Prompt

User reported that personalized content code blocks showed only one color (monochrome) while original Docusaurus pages showed proper syntax highlighting with different colors for keywords, strings, comments, etc. User requested updating the LLM prompt for proper code block formatting and implementing client-side syntax highlighting.

## Response snapshot

Implemented comprehensive syntax highlighting for personalized content:

1. **Backend**: Updated LLM prompt with strict markdown formatting rules, especially for code blocks. Changed model from `gemini-2.5-flash-lite` to `gemini-2.5-flash` for better formatting compliance. Added detailed code block format instructions.

2. **Frontend**: Implemented `highlightSyntax()` function with VS Code Dark+ theme colors supporting 12 programming languages. Code blocks now render with proper keyword, string, comment, and function highlighting.

## Changes Made

### Backend - LLM Prompt Enhancement
**`backend/src/main.py`**
- Updated `build_personalization_prompt()` with strict output format rules:
  - Explicit markdown heading, paragraph, list, and code block formatting rules
  - Examples of correct vs incorrect code block syntax
  - Requirement to ALWAYS specify language identifier after triple backticks
  - No HTML tags, no frontmatter, no meta-commentary rules
- Changed model from `gemini-2.5-flash-lite` to `gemini-2.5-flash` for better output quality
- Lowered temperature from 0.3 to 0.2 for more consistent formatting
- Updated health check endpoint to reflect new model name

### Frontend - Syntax Highlighting
**`src/theme/DocItem/Content/index.tsx`**
- Added `escapeHtml()` function to safely escape HTML entities in code
- Added `highlightSyntax()` function with VS Code Dark+ theme colors:
  - **Python**: keywords (blue), strings (orange), comments (green), built-ins (yellow), decorators, numbers, self
  - **JavaScript/TypeScript**: keywords, types (teal), strings, comments, function calls, properties
  - **Bash/Shell**: keywords, commands, variables, flags, comments
  - **JSON**: property names (light blue), string values (orange), numbers, booleans
  - **YAML**: keys, values, comments
  - **HTML/XML**: tags (blue), attributes, attribute values, comments
  - **CSS/SCSS**: selectors (gold), properties, values, colors
  - **SQL**: keywords (blue), strings, comments, numbers
  - **C/C++**: keywords, types, preprocessor (purple), strings, comments
  - **Rust**: keywords, types, macros, lifetimes
  - **Go**: keywords, types, built-in functions
- Updated `convertMarkdownToHtml()` to use `highlightSyntax()` for code blocks
- Code blocks styled with dark theme background (#1e1e1e) and Consolas font

## Technical Implementation

### Syntax Highlighting Approach
Implemented regex-based client-side highlighting (similar to Prism.js/highlight.js):

1. Code is HTML-escaped first to prevent XSS
2. Language-specific regex patterns identify tokens
3. Tokens wrapped in `<span style="color:...">` tags
4. Colors match VS Code Dark+ theme for familiarity

### VS Code Dark+ Theme Colors
```javascript
const colors = {
  keyword: '#569cd6',      // Blue - if, for, function, etc.
  string: '#ce9178',       // Orange - "text", 'text'
  comment: '#6a9955',      // Green - // comment, # comment
  function: '#dcdcaa',     // Yellow - function calls
  type: '#4ec9b0',         // Teal - TypeScript types
  number: '#b5cea8',       // Light green - 123, 3.14
  variable: '#9cdcfe',     // Light blue - variables
  preprocessor: '#c586c0', // Purple - #include, @decorator
};
```

### Language Detection
Languages supported via code block language identifier:
- `python`, `py`
- `javascript`, `js`, `typescript`, `ts`, `jsx`, `tsx`
- `bash`, `sh`, `shell`, `zsh`
- `json`
- `yaml`, `yml`
- `html`, `xml`
- `css`, `scss`, `sass`
- `sql`
- `c`, `cpp`, `c++`
- `rust`, `rs`
- `go`, `golang`

### LLM Prompt Format Rules
Added strict formatting instructions to ensure consistent markdown output:

```
=== STRICT OUTPUT FORMAT RULES ===
1. HEADINGS: Use # for h1, ## for h2, etc. Blank lines before/after.
2. PARAGRAPHS: Separate with blank lines.
3. CODE BLOCKS: ALWAYS use ```language format.
4. INLINE CODE: Use single backticks.
5. LISTS: Use - or * for unordered, 1. 2. 3. for ordered.
6. BOLD/ITALIC: Use **bold** and *italic*.
7. LINKS: Use [text](url) format.
8. BLOCKQUOTES: Use > at line start.
9. NO HTML TAGS.
10. NO FRONTMATTER.
```

## Outcome

- ✅ Impact: Personalized code blocks now have proper syntax highlighting matching VS Code Dark+ theme
- 🧪 Tests: Manual verification of Python, JavaScript, Bash code highlighting
- 📁 Files: 2 files modified
- 🔁 Next prompts: Add copy-to-clipboard button for code blocks, line numbers
- 🧠 Reflection: Client-side regex highlighting is lightweight and doesn't require additional libraries. The strict LLM prompt ensures consistent code block formatting that the highlighter can parse correctly.

## Evaluation notes (flywheel)

- Failure modes observed:
  - Initial attempt to make LLM return HTML instead of markdown broke page rendering (reverted)
  - Inline styles needed to be applied directly as dynamic content doesn't inherit Prism CSS
  - HTML entities (&lt;, &gt;) needed special handling in regex patterns
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Integrate Prism.highlightAll() after DOM insertion for more robust highlighting

```
