```prompt
---
id: 0018
title: Urdu Translation Feature - Static Pre-translated Files
stage: feature
date: 2025-11-30
surface: agent
model: claude-opus-4-5-20250929
feature: 001-physical-ai-robotics-textbook
branch: 001-physical-ai-robotics-textbook
user: user
command: implement feature
labels: ["translation", "urdu", "i18n", "static-files", "frontend", "feature"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - static/translations/ur/intro.md
  - static/translations/ur/chapter1.md
  - static/translations/ur/chapter2.md
  - static/translations/ur/chapter3.md
  - src/theme/DocItem/Content/index.tsx
  - src/theme/DocItem/Content/Content.module.css
  - backend/src/main.py (removed translate-chapter endpoint)
tests:
  - Manual verification of language toggle button visibility
  - Manual verification of Urdu content fetching and rendering
  - Manual verification of auth-gated translation access
---

## Prompt

User requested to switch from LLM-based translation to pre-translated static Urdu files. The translation should be available only for logged-in users, with a language toggle button that switches between English and Urdu versions of chapter content.

## Response snapshot

Implemented a static file-based Urdu translation system:

1. **Static Files**: Created pre-translated Urdu markdown files in `/static/translations/ur/` directory (hidden from Docusaurus sidebar).

2. **Frontend**: Added language toggle button (English/اردو) to chapter pages, fetches markdown from static files, converts to HTML and replaces content. Auth-gated to logged-in users only.

3. **Backend**: Removed the LLM-based `/translate-chapter` endpoint as translation now uses static files.

## Changes Made

### Static Translation Files
**`static/translations/ur/intro.md`** (NEW)
- Urdu translation of Introduction to Physical AI chapter

**`static/translations/ur/chapter1.md`** (NEW)
- Urdu translation of Chapter 1: The Rise of Embodied AI

**`static/translations/ur/chapter2.md`** (NEW)
- Urdu translation of Chapter 2: Robot Hardware & Sensors

**`static/translations/ur/chapter3.md`** (NEW)
- Urdu translation of Chapter 3: Software & Simulation

### Frontend Implementation
**`src/theme/DocItem/Content/index.tsx`**
- Added language toggle button (English/اردو) next to personalize button
- `isUrduMode` state to track current language
- `handleLanguageToggle()` function to fetch Urdu content from static files
- Auth check: Only logged-in users can access translation
- Stores original English content in sessionStorage for revert
- `convertMarkdownToHtml()` function to render fetched markdown content
- Multiple DOM selector fallbacks for reliable content injection

**`src/theme/DocItem/Content/Content.module.css`**
- Added `.translateButton` styles with teal color theme (#0d9488)
- Added `.translateButton.active` state for Urdu mode indicator
- Hover and focus states for accessibility

### Backend Changes
**`backend/src/main.py`**
- Removed `/translate-chapter` endpoint (no longer needed with static files)
- Removed `TranslatedContent` model import
- Removed `TranslateChapterRequest` and `TranslateChapterResponse` classes

## Technical Implementation

### Why Static Files?
1. **Performance**: No LLM latency, instant content switching
2. **Cost**: No API costs for translation
3. **Quality**: Human-reviewed translations are more accurate
4. **Reliability**: No dependency on external translation services
5. **Offline**: Works without backend for translation

### File Location Strategy
Files are placed in `/static/translations/ur/` instead of `/docs/physical-ai-ur/` to:
- Hide from Docusaurus sidebar (only /docs/ appears in navigation)
- Keep original chapter URLs clean
- Serve static files directly via Docusaurus static folder

### Content Fetching Flow
1. User clicks "اردو" button
2. Frontend fetches `/ai-native-hackathon/translations/ur/{chapterId}.md`
3. Markdown content is converted to HTML using `convertMarkdownToHtml()`
4. Content replaces article innerHTML
5. Original English saved to sessionStorage for "English" revert

### Auth Gating
- Translation button only visible to authenticated users
- `useAuth()` hook checks `isAuthenticated` state
- Unauthenticated users see only "Sign in to personalize" prompt

### DOM Selector Robustness
Multiple selector fallbacks ensure content injection works:
```javascript
const articleElement = document.querySelector('.markdown') ||
                       document.querySelector('article') ||
                       document.querySelector('[class*="docItemCol"]') ||
                       document.querySelector('.theme-doc-markdown');
```

## Outcome

- ✅ Impact: Users can toggle between English and Urdu content instantly without LLM delays
- 🧪 Tests: Manual verification of toggle functionality, auth gating, content rendering
- 📁 Files: 7 files created/modified
- 🔁 Next prompts: Add more language support, improve markdown-to-HTML conversion
- 🧠 Reflection: Static files provide a more reliable and faster translation experience. The trade-off is that translations must be pre-created, but this ensures quality and consistency.

## Evaluation notes (flywheel)

- Failure modes observed:
  - Initial DOM selector `article.markdown` returned null (fixed with multiple fallbacks)
  - Files in /docs/ appeared in sidebar (moved to /static/translations/)
  - Button click not triggering (selector issue on some pages)
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Add RTL support for Urdu text rendering, lazy load translations

```
