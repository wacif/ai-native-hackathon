---
id: 0017
title: Personalization Feature - User Profile & Content Adaptation
stage: feature
date: 2025-11-30
surface: agent
model: claude-opus-4-5-20250929
feature: 001-physical-ai-robotics-textbook
branch: 001-physical-ai-robotics-textbook
user: user
command: implement feature
labels: ["personalization", "user-profile", "chatbot", "LLM", "fullstack", "feature"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/migrations/versions/e7f97eb76c81_add_personalization_fields_and_content_.py
  - backend/src/models/user.py
  - backend/src/models/personalized_content.py
  - backend/src/models/__init__.py
  - backend/src/api/auth.py
  - backend/src/services/auth_service.py
  - backend/src/chatbot/agent.py
  - backend/src/chatbot/__init__.py
  - backend/src/main.py
  - src/context/AuthContext.tsx
  - src/components/Auth/SignupForm.tsx
  - src/components/Auth/Auth.module.css
  - src/theme/DocItem/Content/index.tsx
  - src/theme/DocItem/Content/Content.module.css
  - src/components/PersonalizeButton/index.tsx
  - src/components/PersonalizeButton/PersonalizeButton.module.css
tests:
  - Manual verification of signup flow with new fields
  - Manual verification of personalized chatbot responses
  - Manual verification of chapter personalization button
---

## Prompt

user can personalize website content and chatbot answer based on there profile... also we have to add the user programming language the system like windows mac linux so the bot and assistant provide commands accordingly. The user can also specify their learning goals, prior knowledge, preferred explanation style, and industry focus.

## Response snapshot

Implemented a comprehensive personalization system that allows users to customize their learning experience:

1. **Database & Models**: Added 6 new user profile fields (programming_languages, operating_system, learning_goals, preferred_explanation_style, prior_knowledge, industry) and a PersonalizedContent model for caching LLM-generated content.

2. **Backend API**: Updated signup/profile endpoints, modified chatbot to inject user profile into system prompts, created `/personalize-chapter` endpoint for chapter content personalization.

3. **Frontend**: Multi-step signup form with chip selection, personalization button on doc pages, auth token injection in chatbot requests.

## Changes Made

### Database Migration
**`backend/migrations/versions/e7f97eb76c81_add_personalization_fields_and_content_.py`**
- Added columns to `users` table: `programming_languages` (JSONB), `operating_system`, `learning_goals` (JSONB), `preferred_explanation_style`, `prior_knowledge` (JSONB), `industry`
- Created `personalized_content` table with `user_id`, `chapter_id`, `original_content_hash`, `personalized_content`, timestamps

### Backend Models
**`backend/src/models/user.py`**
- Added 6 new SQLAlchemy columns for personalization fields

**`backend/src/models/personalized_content.py`** (NEW)
- PersonalizedContent model for caching personalized chapter content per user
- Unique constraint on (user_id, chapter_id)

### Backend API
**`backend/src/api/auth.py`**
- Updated SignupRequest, UserProfileResponse, UpdateProfileRequest with new fields
- Signup and profile endpoints accept/return new fields

**`backend/src/services/auth_service.py`**
- Updated create_user and update_user to handle new fields

### Chatbot Agent
**`backend/src/chatbot/agent.py`**
- Added `UserProfile` dataclass
- Created `build_personalized_instructions()` function that generates OS-specific commands, language-specific code examples, style-adapted explanations
- Updated `query_agent()` to accept user_profile parameter
- Created `create_rag_agent()` factory for personalized agents

### Main API
**`backend/src/main.py`**
- Added `build_user_profile()` helper function
- Updated `/query` and `/query-selection` to accept auth token and personalize responses
- Added `/personalize-chapter` endpoint for chapter content personalization with caching
- Added `build_personalization_prompt()` for chapter personalization

### Frontend Auth Context
**`src/context/AuthContext.tsx`**
- Updated User interface with new personalization fields
- Added `token` to context for API authentication

### Signup Form
**`src/components/Auth/SignupForm.tsx`**
- Converted to 3-step multi-select form:
  - Step 1: Credentials (username, email, password)
  - Step 2: Background (OS selection, programming languages chips)
  - Step 3: Learning Preferences (explanation style, learning goals, prior knowledge chips)
- Added PROGRAMMING_LANGUAGES, LEARNING_GOALS, PRIOR_KNOWLEDGE arrays for options

**`src/components/Auth/Auth.module.css`**
- Added step indicator styles (.stepIndicator, .stepDot, .stepLine)
- Added chip selection styles (.chipContainer, .chip, .chip.selected)
- Dark mode support

### Doc Page Personalization
**`src/theme/DocItem/Content/index.tsx`** (NEW - Docusaurus swizzle)
- PersonalizeControls component that shows:
  - "Sign in to personalize" for unauthenticated users
  - "Update profile" prompt for users without preferences
  - "Personalize for me" button for users with preferences
- Handles personalization API calls and content replacement
- Session storage for reverting to original content

**`src/theme/DocItem/Content/Content.module.css`** (NEW)
- Styles for personalization container, button, badge, tags

## Personalization Behavior

### Chatbot Personalization
When authenticated users query the chatbot, their profile is injected into the system prompt:
- **Operating System**: Commands are adapted (PowerShell for Windows, bash for Linux/macOS)
- **Programming Languages**: Code examples prefer user's preferred language
- **Explanation Style**: Concise (bullet points), Detailed (comprehensive), Visual (diagrams), Example-driven
- **Prior Knowledge**: Skips basic explanations for known topics
- **Learning Goals**: Emphasizes relevant content
- **Industry**: Relates examples to user's field

### Chapter Personalization
Users can click "Personalize for me" on any chapter page:
1. Content is sent to Gemini with user profile context
2. LLM adapts content (commands, examples, explanations)
3. Result is cached in database (PersonalizedContent table)
4. User can revert to original with "View Original" button

## Outcome

- ✅ Impact: Users get personalized learning experience based on their OS, programming languages, learning style, and background
- 🧪 Tests: Manual verification of signup flow, chatbot personalization, chapter personalization
- 📁 Files: 15 files created/modified across backend and frontend
- 🔁 Next prompts: Add profile editing page, implement personalization analytics
- 🧠 Reflection: The implementation creates a foundation for adaptive learning where content and responses are tailored to each user's context. The caching system prevents redundant LLM calls for the same content.

## Evaluation notes (flywheel)

- Failure modes observed: 
  - Docusaurus theme swizzling required specific folder structure (DocItem/Content not DocItem)
  - JSX namespace errors required using React.ReactElement instead
  - search_book_content function needed to be defined before agent creation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Add real-time profile editing, personalization analytics dashboard
