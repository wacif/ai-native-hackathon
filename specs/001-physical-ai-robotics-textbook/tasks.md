# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-robotics-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The spec does not explicitly request test tasks, but implies them through "Independent Test" criteria. I will include foundational testing setup and specific acceptance test scenarios as tasks.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below assume this web app structure.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Initialize Docusaurus project in `frontend/` (Docusaurus 3.9.2 configured at root with proper title)
- [X] T002 Initialize FastAPI project in `backend/`
- [X] T003 Configure Docusaurus `docusaurus.config.ts` for textbook structure
- [X] T004 Configure FastAPI `backend/main.py` for basic server setup
- [X] T005 [P] Setup Git repository and initial commit (already exists, but for completeness)
- [X] T006 Install frontend dependencies (Docusaurus) in `frontend/package.json` (Docusaurus 3.9.2 + deps installed)
- [X] T006.1 Create and activate virtual environment in `backend/` (`.venv/`)
- [X] T007 Install backend dependencies (FastAPI, uvicorn, openai, qdrant-client, psycopg2-binary) in `backend/requirements.txt` (all installed)
- [X] T008 [P] Configure basic linting and formatting for frontend (`.prettierrc`, `.eslintrc.js`)
- [X] T009 [P] Configure basic linting and formatting for backend (`pyproject.toml`, `.flake8`)
- [X] T009.1 Verify external CLI tools (alembic, gh) are installed and accessible (alembic 1.17.2, gh 2.83.1)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T009.2 Verify foundational files (e.g., `backend/src/models/base.py`) exist and are correctly configured (fixed imports, created models/__init__.py)
- [X] T009.3 Ensure `DATABASE_URL` is set in `.env` and correctly loaded for backend tools/scripts (load_dotenv() in db.py, qdrant.py, env.py)
- [X] T010 Setup Neon Serverless Postgres database connection in `backend/src/config/db.py` (with error handling and dotenv)
- [X] T011 Setup Qdrant Cloud client connection in `backend/src/config/qdrant.py` (updated with dotenv and error handling)
- [X] T012 Create base User model in `backend/src/models/user.py` (id, username, email, password_hash, + extended fields)
- [X] T013 Create base Book Content model in `backend/src/models/book_content.py` (id, title, raw_text, + extended fields)
- [X] T014 Create Chatbot Interaction model in `backend/src/models/chatbot_interaction.py` (fixed foreign key reference)
- [X] T015 Implement initial database migration script for core models in `backend/migrations/` (migration 9b67d188e1c6 created successfully)
- [X] T016 Setup FastAPI router for API endpoints in `backend/src/api/` (created api/__init__.py with router structure)
- [X] T017 Configure error handling and logging infrastructure for backend in `backend/src/utils/` (created logger.py, errors.py, __init__.py)
- [X] T018 Setup environment configuration management (`.env` files) in `backend/.env.example`, `frontend/.env.example` (both created)
- [X] T019 Implement utility for embedding generation in `backend/src/chatbot/agent.py` (using FastEmbed, working implementation)
- [X] T020 Implement utility for Qdrant indexing and search in `backend/src/rag/ingestion.py` and `backend/src/chatbot/agent.py` (working implementation)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## ✅ **Phase 1 & 2 Completion Status**

**Date**: 2025-11-29  
**Status**: ✅ **COMPLETE - Foundation Ready**

### Achievements:
- ✅ All setup tasks complete (T001-T009.1)
- ✅ All foundational tasks complete (T009.2-T020)
- ✅ Backend organized into proper modules (chatbot/, rag/, api/, utils/)
- ✅ Database migrations created (9b67d188e1c6)
- ✅ RAG chatbot backend tested and working (T032-T034)
- ✅ GitHub Pages deployment configured
- ✅ Site deployed at: https://wacif.github.io/ai-native-hackathon/

### Strategic Decision:
**Proceeding with Phase 4 (User Story 2) before Phase 3 (User Story 1)**

**Rationale:**
1. Backend RAG chatbot already 60% complete (T032-T034 done)
2. Demonstrates AI-native capability (core hackathon focus)
3. Can test with existing Docusaurus tutorial content
4. Phase 3 content creation is time-intensive and can be done later
5. Both US1 and US2 are P1 priority - either order valid

---

## Phase 3: User Story 1 - Core Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Access comprehensive, AI-native textbook content on Physical AI & Humanoid Robotics, organized by modules and weekly breakdowns.

**Independent Test**: Navigate through the published book's chapters and modules, verifying content availability and readability.

### Tests for User Story 1
- [ ] T021 [US1] Frontend integration test: Verify chapter content loads in `frontend/tests/integration/test_chapter_load.spec.js`
- [ ] T022 [US1] Frontend integration test: Verify weekly breakdown navigation in `frontend/tests/integration/test_weekly_navigation.spec.js`

### Implementation for User Story 1
- [ ] T023 [US1] Create Docusaurus `docs/` structure for modules/chapters/weeks in `frontend/docs/`
- [ ] T024 [P] [US1] Add initial content for Module 1, Chapter 1 in `frontend/docs/module1/chapter1.mdx`
- [ ] T025 [P] [US1] Add initial content for Module 1, Week 1 breakdown in `frontend/docs/module1/week1.mdx`
- [ ] T026 [US1] Configure `sidebars.ts` for hierarchical navigation in `frontend/sidebars.ts`
- [ ] T027 [US1] Implement basic Docusaurus page layout components in `frontend/src/theme/`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Integrated RAG Chatbot (Priority: P1)

**Goal**: Integrated RAG chatbot to ask questions about the book's content, including selected text.

**Independent Test**: Ask general questions about the book content and specific questions by selecting text, verifying that the chatbot provides accurate and relevant answers.

### Tests for User Story 2
- [ ] T028 [US2] Backend unit test: Validate RAG logic for general questions in `backend/tests/unit/test_rag_service.py`
- [ ] T029 [US2] Backend unit test: Validate RAG logic for selected context questions in `backend/tests/unit/test_rag_service_context.py`
- [ ] T030 [US2] Frontend integration test: Verify chatbot UI appears and sends queries in `frontend/tests/integration/test_chatbot_ui.spec.js`
- [ ] T031 [US2] Frontend integration test: Verify chatbot response display for general queries in `frontend/tests/integration/test_chatbot_response.spec.js`

### Implementation for User Story 2
- [X] T032 [US2] Create FastAPI endpoint for RAG chatbot queries in `backend/src/main.py` (working: /query and /query-selection with page_url, chapter_id support)
- [X] T033 [US2] Implement RAG agent logic in `backend/src/chatbot/agent.py` (OpenAI Agents SDK with Gemini, tested and working)
- [X] T034 [US2] Integrate Book Content into Qdrant for RAG in `backend/src/rag/ingestion.py` (working ingestion pipeline)
- [X] T035 [P] [US2] Develop React chatbot UI component in `src/components/Chatbot/index.tsx` (complete with TypeScript, floating UI, message history)
- [X] T036 [US2] Embed chatbot component into Docusaurus layout in `src/theme/Root.tsx` (integrated with page context tracking)
- [X] T037 [US2] Implement frontend logic to send selected text to chatbot API (text selection listener, dedicated endpoint usage)
- [X] T038 [US2] Add basic CSS for chatbot styling in `src/components/Chatbot/Chatbot.module.css` (complete with dark mode, responsive design)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

### Content & Polish for User Story 2
- [X] T038.1 [US2] Create Physical AI textbook chapters (intro, chapter1, chapter2, chapter3) in `docs/physical-ai/`
- [X] T038.2 [US2] Add chapter_id, module_id, page_url fields to BookContent model in `backend/src/models/book_content.py`
- [X] T038.3 [US2] Create database migration for chapter metadata fields
- [X] T038.4 [US2] Update ingestion pipeline to extract and store chapter metadata in `backend/src/rag/ingestion.py`
- [X] T038.5 [US2] Create Qdrant payload indexes for chapter_id and module_id filtering
- [X] T038.6 [US2] Update RAG agent to support chapter-aware filtering in `backend/src/chatbot/agent.py`
- [X] T038.7 [US2] Enhance agent instructions to answer location queries ("what chapter am I on?")
- [X] T038.8 [US2] Simplify text selection UX (single send button for both general and selection queries)
- [X] T038.9 [US2] Add markdown rendering to chatbot UI using react-markdown in `src/components/Chatbot/index.tsx`
- [X] T038.10 [US2] Remove default Docusaurus tutorial content from docs and blog
- [X] T038.11 [US2] Update homepage features to reflect Physical AI content in `src/pages/index.tsx`
- [X] T038.12 [US2] Add sidebar_position frontmatter to chapters for correct ordering

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## ✅ **Phase 4 Completion Status**

**Date**: 2025-11-29  
**Status**: ✅ **COMPLETE - User Story 2 Fully Functional**

### Achievements:
- ✅ Backend API enhanced with page context support (page_url, chapter_id)
- ✅ React chatbot UI component with TypeScript
- ✅ Text selection feature working (highlights → ask questions)
- ✅ Floating chat UI with smooth animations
- ✅ Message history and loading states
- ✅ Dark mode support
- ✅ Responsive design (mobile + desktop)
- ✅ Integrated into Docusaurus via Root.tsx
- ✅ 4 comprehensive Physical AI chapters created
- ✅ Chapter-aware RAG filtering with Qdrant indexes
- ✅ Chatbot knows current chapter and can answer location queries
- ✅ Markdown rendering in chatbot responses
- ✅ Clean content (removed default Docusaurus tutorials)
- ✅ Proper sidebar ordering with frontmatter

### Features Implemented:
1. **General Questions**: Ask about any book content via RAG
2. **Text Selection**: Highlight text → Ask specific questions (simplified UX)
3. **Page-Aware**: Chatbot knows which page user is on and filters by chapter
4. **Location Aware**: Can answer "what chapter am I on?" type questions
5. **Source Attribution**: Shows where answers come from with chapter info
6. **Beautiful UI**: Gradient design, smooth animations, professional look
7. **Markdown Rendering**: Bot responses render markdown properly (bold, lists, code, etc.)
8. **Clean Navigation**: Proper chapter ordering in sidebar

### Technical Stack:
- **Frontend**: React + TypeScript + CSS Modules + react-markdown
- **Backend**: FastAPI + OpenAI Agents SDK + Gemini
- **Vector DB**: Qdrant Cloud (with payload indexes for filtering)
- **Embeddings**: FastEmbed (BAAI/bge-small-en-v1.5)
- **Content**: 4 Physical AI chapters (14 chunks in Qdrant)

---

## Phase 5: User Story 3 - User Authentication & Personalization (Priority: P2)

**Goal**: Sign up and sign in using a secure authentication system, providing software and hardware background, to access personalized content and features.

**Independent Test**: Complete signup, log in, verify background information capture, and personalization features accessibility.

### Tests for User Story 3
- [ ] T039 [US3] Backend unit test: Validate user registration and login in `backend/tests/unit/test_auth_service.py`
- [ ] T040 [US3] Backend unit test: Validate user profile update with background information in `backend/tests/unit/test_user_profile.py`
- [ ] T041 [US3] Frontend integration test: Verify signup and login flow in `frontend/tests/integration/test_auth_flow.spec.js`

### Implementation for User Story 3
- [X] T042 [US3] Configure FastAPI JWT auth in `backend/src/config/auth.py` (JWT config, bcrypt settings)
- [X] T043 [US3] Create FastAPI endpoints for user registration and login in `backend/src/api/auth.py` (/signup, /signin, /refresh, /me)
- [X] T044 [US3] Implement `AuthService` for user management (create, login, get profile) in `backend/src/services/auth_service.py`
- [X] T045 [P] [US3] Develop React components for signup form in `src/components/Auth/SignupForm.tsx` (3-step form with personalization)
- [X] T046 [P] [US3] Develop React components for login form in `src/components/Auth/LoginForm.tsx`
- [X] T047 [US3] Implement frontend context/store for user authentication state in `src/context/AuthContext.tsx` (useAuth, useSession hooks)
- [X] T048 [US3] Implement logic to collect software and hardware background during signup in `src/components/Auth/SignupForm.tsx`
- [X] T049 [US3] Add `software_background`, `hardware_background`, `personalization_preferences`, `selected_language` fields to User model in `backend/src/models/user.py` (already existed)
- [X] T050 [US3] Implement database migration to add new User model fields in `backend/migrations/` (already existed)
- [X] T050.1 [US3] Create UserButton component for navbar integration in `src/components/Auth/UserButton.tsx`
- [X] T050.2 [US3] Wrap app with AuthProvider in `src/theme/Root.tsx`
- [X] T050.3 [US3] Restrict chatbot access to authenticated users only in `src/components/Chatbot/index.tsx`
- [X] T050.4 [US3] Add login/signup buttons to chatbot for unauthenticated users
- [X] T050.5 [US3] Clear chatbot messages when user signs out

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## ✅ **Phase 5 Completion Status**

**Date**: 2025-11-30  
**Status**: ✅ **COMPLETE - User Story 3 Authentication Implemented**

### Technology Decision:
**FastAPI Native JWT Auth** chosen over Better-Auth because:
1. Single backend (Python) - no Node.js service needed
2. Simpler deployment for hackathon
3. Existing User model already compatible
4. FastAPI has excellent auth utilities (OAuth2, JWT)

### Achievements:
- ✅ JWT-based authentication with bcrypt password hashing
- ✅ OAuth2 password flow for standard auth
- ✅ Access tokens (1 hour) + Refresh tokens (7 days)
- ✅ AuthContext with `useAuth()` and `useSession()` hooks
- ✅ Two-step signup form collecting background info
- ✅ UserButton component for navbar integration
- ✅ Full CRUD for user profile (/auth/me)
- ✅ Chatbot restricted to authenticated users only
- ✅ Chat history clears on signout

### API Endpoints:
- `POST /auth/signup` - Register new user
- `POST /auth/signin` - Login (OAuth2 form)
- `POST /auth/refresh` - Refresh access token
- `GET /auth/me` - Get current user profile
- `PATCH /auth/me` - Update user profile

### Files Created:
- `backend/src/config/auth.py` - JWT configuration
- `backend/src/services/auth_service.py` - Auth business logic
- `backend/src/api/auth.py` - Auth endpoints
- `src/context/AuthContext.tsx` - React auth state
- `src/components/Auth/` - SignupForm, LoginForm, UserButton

---

## Phase 6: User Story 4 - Content Personalization (Priority: P2)

**Goal**: Logged-in student can personalize chapter content by pressing a button, so the textbook adapts to their background and learning preferences.

**Independent Test**: Log in, navigate to a chapter, activate personalization, and observe content changes relevant to the user's recorded background.

### Tests for User Story 4
- [ ] T051 [US4] Backend unit test: Validate content personalization logic based on user background in `backend/tests/unit/test_personalization_service.py`
- [ ] T052 [US4] Frontend integration test: Verify personalization button functionality and content adaptation in `frontend/tests/integration/test_personalization_feature.spec.js`

### Implementation for User Story 4
- [X] T053 [US4] Create FastAPI endpoint for content personalization requests in `backend/src/main.py` (`/personalize-chapter` endpoint)
- [X] T054 [US4] Implement personalization logic to dynamically adjust content based on user profile in `backend/src/main.py` (`build_personalization_prompt()`)
- [X] T055 [US4] Add personalization fields to User model: `programming_languages`, `operating_system`, `learning_goals`, `preferred_explanation_style`, `prior_knowledge`, `industry` in `backend/src/models/user.py`
- [X] T056 [US4] Create `PersonalizedContent` model for caching LLM-generated content in `backend/src/models/personalized_content.py`
- [X] T057 [US4] Implement database migration to add personalization fields and PersonalizedContent table in `backend/migrations/versions/e7f97eb76c81_*.py`
- [X] T058 [US4] Develop React `PersonalizeButton` component in `src/components/PersonalizeButton/index.tsx`
- [X] T059 [US4] Swizzle Docusaurus DocItem/Content to add personalization controls in `src/theme/DocItem/Content/index.tsx`
- [X] T060 [US4] Implement frontend logic to fetch and display personalized content from backend API
- [X] T061 [US4] Show user profile tags (OS, languages, style) on personalization button
- [X] T062 [US4] Add "View Original" button to revert personalized content
- [X] T063 [US4] Disable personalization button if user is not logged in (show "Sign in to personalize")
- [X] T064 [US4] Disable personalization if user has no preferences set (show "Update profile" prompt)
- [X] T065 [US4] Convert signup form to 3-step wizard with chip selection in `src/components/Auth/SignupForm.tsx`
- [X] T066 [US4] Add step indicator UI (dots + lines) to signup form
- [X] T067 [US4] Implement chip selection for multi-select fields (programming languages, learning goals, prior knowledge)
- [X] T068 [US4] Update API schemas to accept new personalization fields in signup/profile endpoints
- [X] T069 [US4] Inject user profile into chatbot system prompts for personalized responses in `backend/src/chatbot/agent.py`
- [X] T070 [US4] Create `UserProfile` dataclass and `build_personalized_instructions()` function in agent
- [X] T071 [US4] Update `/query` and `/query-selection` endpoints to accept auth token and personalize responses
- [X] T072 [US4] Send auth token with chatbot requests for personalized responses in `src/components/Chatbot/index.tsx`
- [X] T073 [US4] Add `token` to AuthContext for API authentication
- [X] T074 [US4] Cache personalized chapter content in database to avoid redundant LLM calls

**Checkpoint**: At this point, User Stories 1, 2, 3 AND 4 should all work independently

---

## ✅ **Phase 6 Completion Status**

**Date**: 2025-11-30  
**Status**: ✅ **COMPLETE - User Story 4 Personalization Implemented**

### Achievements:
- ✅ 6 new user profile fields for personalization
- ✅ PersonalizedContent model for caching LLM-generated content
- ✅ `/personalize-chapter` API endpoint with content caching
- ✅ 3-step signup wizard with chip selection UI
- ✅ Docusaurus DocItem/Content swizzle for "Personalize for me" button
- ✅ Chatbot system prompts inject user profile for personalized responses
- ✅ OS-specific commands (Windows/macOS/Linux)
- ✅ Programming language preference for code examples
- ✅ Explanation style adaptation (concise/detailed/visual/example-driven)
- ✅ Prior knowledge awareness (skip basic explanations)
- ✅ Learning goals emphasis
- ✅ Industry-specific examples

### Personalization Features:
1. **Operating System**: Commands adapted (PowerShell vs bash vs zsh)
2. **Programming Languages**: Code examples in preferred language
3. **Explanation Style**: Concise (bullets), Detailed (comprehensive), Visual (diagrams), Example-driven
4. **Prior Knowledge**: Skip basic explanations for known topics
5. **Learning Goals**: Emphasize relevant content
6. **Industry Focus**: Relate examples to user's field

### Files Created/Modified:
- `backend/migrations/versions/e7f97eb76c81_*.py` - Migration for new fields
- `backend/src/models/personalized_content.py` - Cache model
- `backend/src/models/user.py` - 6 new personalization fields
- `backend/src/chatbot/agent.py` - UserProfile, personalized instructions
- `backend/src/main.py` - /personalize-chapter endpoint
- `src/components/Auth/SignupForm.tsx` - 3-step wizard with chips
- `src/theme/DocItem/Content/index.tsx` - Personalize button on docs
- `src/context/AuthContext.tsx` - Added token to context

---

## Phase 7: User Story 5 - Urdu Translation (Priority: P3)

**Goal**: Logged-in student can translate the content of chapters into Urdu by pressing a button, so they can learn in their preferred language.

**Independent Test**: Log in, navigate to a chapter, activate Urdu translation, and verify that the chapter content is accurately displayed in Urdu.

### Tests for User Story 5
- [ ] T075 [US5] Backend unit test: Validate Urdu translation service integration (mock external service) in `backend/tests/unit/test_translation_service.py`
- [ ] T076 [US5] Frontend integration test: Verify Urdu translation button functionality and content display in `frontend/tests/integration/test_translation_feature.spec.js`

### Implementation for User Story 5
- [X] T077 [US5] Create static Urdu translation files in `static/translations/ur/` directory
- [X] T078 [US5] Implement language toggle button (English/اردو) in `src/theme/DocItem/Content/index.tsx`
- [X] T079 [US5] Implement frontend markdown-to-HTML converter for static translation files
- [X] T080 [US5] Add auth gating - translation only available for logged-in users
- [X] T081 [US5] Store original English content in sessionStorage for easy revert
- [X] T082 [US5] Style language toggle button in `src/theme/DocItem/Content/Content.module.css`
- [X] T083 [US5] Implement syntax highlighting for code blocks in personalized/translated content
- [X] T084 [US5] Support 12 programming languages with VS Code Dark+ theme colors

**Checkpoint**: All user stories should now be independently functional

---

## ✅ **Phase 7 Completion Status**

**Date**: 2025-11-30  
**Status**: ✅ **COMPLETE - User Story 5 Urdu Translation Implemented**

### Technology Decision:
**Static Pre-translated Files** chosen over LLM-based translation because:
1. Instant content switching (no LLM latency)
2. No API costs for translation
3. Human-reviewed translations ensure quality
4. Works without backend dependency

### Achievements:
- ✅ Static Urdu translation files in `/static/translations/ur/`
- ✅ Language toggle button (English/اردو) on chapter pages
- ✅ Auth-gated translation (logged-in users only)
- ✅ Session storage for original content restoration
- ✅ Markdown-to-HTML conversion for static files
- ✅ Syntax highlighting for code blocks (12 languages supported)
- ✅ VS Code Dark+ theme colors for syntax highlighting

### Files Created:
- `static/translations/ur/intro.md` - Urdu introduction
- `static/translations/ur/chapter1.md` - Urdu Chapter 1
- `static/translations/ur/chapter2.md` - Urdu Chapter 2
- `static/translations/ur/chapter3.md` - Urdu Chapter 3

### Syntax Highlighting Languages:
Python, JavaScript/TypeScript, Bash/Shell, JSON, YAML, HTML/XML, CSS/SCSS, SQL, C/C++, Rust, Go

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T085 Setup frontend testing framework (e.g., Playwright/Cypress) in `frontend/`
- [ ] T086 Setup backend testing framework (e.g., pytest) in `backend/`
- [ ] T087 Configure CI/CD pipeline (GitHub Actions) for Docusaurus build/deploy to GitHub Pages in `.github/workflows/deploy.yml`
- [ ] T088 Configure CI/CD pipeline (GitHub Actions) for backend deployment in `.github/workflows/backend_deploy.yml`
- [ ] T089 [P] Review and update all documentation (README.md, existing docs)
- [ ] T090 Code cleanup and refactoring across frontend and backend
- [ ] T091 Performance optimization for content loading and API responses
- [ ] T092 Security hardening for authentication and data handling
- [ ] T093 Run quickstart.md validation (create quickstart.md as a separate task if needed)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories, but builds on US1 content.
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories, but enables US4 and US5.
- **User Story 4 (P2)**: Depends on User Story 3 for logged-in user context.
- **User Story 5 (P3)**: Depends on User Story 3 for logged-in user context.

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Stories 1, 2, and 3 can start in parallel (if team capacity allows, as they have minimal direct dependencies on each other's *implementation*, though US2 uses US1's content, and US3 enables US4/US5).
- All tasks within a story marked [P] can run in parallel (e.g., different React components, different model definitions).

---

## Parallel Example: User Story 1, 2, 3 (Conceptual)

```bash
# After Foundational tasks are complete:

# Parallel work on US1 (Core Textbook Content)
Task: "T023 [US1] Create Docusaurus `docs/` structure for modules/chapters/weeks in `frontend/docs/`"
Task: "T024 [P] [US1] Add initial content for Module 1, Chapter 1 in `frontend/docs/module1/chapter1.mdx`"
Task: "T025 [P] [US1] Add initial content for Module 1, Week 1 breakdown in `frontend/docs/module1/week1.mdx`"

# Parallel work on US2 (Integrated RAG Chatbot)
Task: "T032 [US2] Create FastAPI endpoint for RAG chatbot queries in `backend/src/api/chatbot.py`"
Task: "T035 [P] [US2] Develop React chatbot UI component in `frontend/src/components/Chatbot.tsx`"

# Parallel work on US3 (User Authentication & Personalization)
Task: "T043 [US3] Create FastAPI endpoints for user registration and login in `backend/src/api/auth.py`"
Task: "T045 [P] [US3] Develop React components for signup form in `frontend/src/components/SignupForm.tsx`"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 & 2 (Frontend/Content focus)
   - Developer B: User Story 3 & 4 (Auth/Personalization focus)
   - Developer C: User Story 5 & RAG backend (Translation/Chatbot focus)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
