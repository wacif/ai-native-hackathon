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

- [ ] T001 Initialize Docusaurus project in `frontend/`
- [ ] T002 Initialize FastAPI project in `backend/`
- [ ] T003 Configure Docusaurus `docusaurus.config.ts` for textbook structure
- [ ] T004 Configure FastAPI `backend/main.py` for basic server setup
- [ ] T005 [P] Setup Git repository and initial commit (already exists, but for completeness)
- [ ] T006 Install frontend dependencies (Docusaurus) in `frontend/package.json`
- [ ] T007 Install backend dependencies (FastAPI, uvicorn, openai, qdrant-client, psycopg2-binary) in `backend/requirements.txt`
- [ ] T008 [P] Configure basic linting and formatting for frontend (`.prettierrc`, `.eslintrc.js`)
- [ ] T009 [P] Configure basic linting and formatting for backend (`pyproject.toml`, `.flake8`)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T010 Setup Neon Serverless Postgres database connection in `backend/src/config/db.py`
- [ ] T011 Setup Qdrant Cloud client connection in `backend/src/config/qdrant.py`
- [ ] T012 Create base User model in `backend/src/models/user.py` (id, username, email, password_hash)
- [ ] T013 Create base Book Content model in `backend/src/models/book_content.py` (id, title, raw_text)
- [ ] T014 Create Chatbot Interaction model in `backend/src/models/chatbot_interaction.py`
- [ ] T015 Implement initial database migration script for core models in `backend/migrations/`
- [ ] T016 Setup FastAPI router for API endpoints in `backend/src/api/`
- [ ] T017 Configure error handling and logging infrastructure for backend in `backend/src/utils/`
- [ ] T018 Setup environment configuration management (`.env` files) in `backend/.env.example`, `frontend/.env.example`
- [ ] T019 Implement utility for embedding generation (e.g., using OpenAI API) in `backend/src/services/embedding_service.py`
- [ ] T020 Implement utility for Qdrant indexing and search in `backend/src/services/qdrant_service.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

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
- [ ] T032 [US2] Create FastAPI endpoint for RAG chatbot queries in `backend/src/api/chatbot.py`
- [ ] T033 [US2] Implement RAG `ChatbotService` logic using `embedding_service` and `qdrant_service` in `backend/src/services/chatbot_service.py`
- [ ] T034 [US2] Integrate `Book Content` into Qdrant for RAG (initial indexing) in `backend/src/scripts/index_content.py`
- [ ] T035 [P] [US2] Develop React chatbot UI component in `frontend/src/components/Chatbot.tsx`
- [ ] T036 [US2] Embed chatbot component into Docusaurus chapter layout in `frontend/src/theme/Root.tsx`
- [ ] T037 [US2] Implement frontend logic to send selected text to chatbot API in `frontend/src/utils/text_selection.ts`
- [ ] T038 [US2] Add basic CSS for chatbot styling in `frontend/src/css/custom.css`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - User Authentication & Personalization (Priority: P2)

**Goal**: Sign up and sign in using a secure authentication system, providing software and hardware background, to access personalized content and features.

**Independent Test**: Complete signup, log in, verify background information capture, and personalization features accessibility.

### Tests for User Story 3
- [ ] T039 [US3] Backend unit test: Validate user registration and login with Better-Auth integration in `backend/tests/unit/test_auth_service.py`
- [ ] T040 [US3] Backend unit test: Validate user profile update with background information in `backend/tests/unit/test_user_profile.py`
- [ ] T041 [US3] Frontend integration test: Verify signup and login flow in `frontend/tests/integration/test_auth_flow.spec.js`

### Implementation for User Story 3
- [ ] T042 [US3] Configure Better-Auth integration with FastAPI backend in `backend/src/config/better_auth.py`
- [ ] T043 [US3] Create FastAPI endpoints for user registration and login in `backend/src/api/auth.py`
- [ ] T044 [US3] Implement `AuthService` for user management (create, login, get profile) in `backend/src/services/auth_service.py`
- [ ] T045 [P] [US3] Develop React components for signup form in `frontend/src/components/SignupForm.tsx`
- [ ] T046 [P] [US3] Develop React components for login form in `frontend/src/components/LoginForm.tsx`
- [ ] T047 [US3] Implement frontend context/store for user authentication state in `frontend/src/context/AuthContext.tsx`
- [ ] T048 [US3] Implement logic to collect software and hardware background during signup in `frontend/src/components/SignupForm.tsx`
- [ ] T049 [US3] Add `software_background`, `hardware_background`, `personalization_preferences`, `selected_language` fields to User model in `backend/src/models/user.py`
- [ ] T050 [US3] Implement database migration to add new User model fields in `backend/migrations/`

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Content Personalization (Priority: P2)

**Goal**: Logged-in student can personalize chapter content by pressing a button, so the textbook adapts to their background and learning preferences.

**Independent Test**: Log in, navigate to a chapter, activate personalization, and observe content changes relevant to the user's recorded background.

### Tests for User Story 4
- [ ] T051 [US4] Backend unit test: Validate content personalization logic based on user background in `backend/tests/unit/test_personalization_service.py`
- [ ] T052 [US4] Frontend integration test: Verify personalization button functionality and content adaptation in `frontend/tests/integration/test_personalization_feature.spec.js`

### Implementation for User Story 4
- [ ] T053 [US4] Create FastAPI endpoint for content personalization requests in `backend/src/api/personalization.py`
- [ ] T054 [US4] Implement `PersonalizationService` to dynamically adjust content based on user `software_background` and `hardware_background` in `backend/src/services/personalization_service.py`
- [ ] T055 [US4] Add `personalized_text` (JSONB) field to `Book Content` model in `backend/src/models/book_content.py`
- [ ] T056 [US4] Implement database migration to add `personalized_text` field to `Book Content` in `backend/migrations/`
- [ ] T057 [US4] Develop React `PersonalizeButton` component in `frontend/src/components/PersonalizeButton.tsx`
- [ ] T058 [US4] Integrate `PersonalizeButton` into Docusaurus chapter layout in `frontend/src/theme/DocItem/Content/index.tsx`
- [ ] T059 [US4] Implement frontend logic to fetch and display personalized content from backend API in `frontend/src/utils/personalization_hook.ts`
- [ ] T060 [US4] Disable personalization button if user is not logged in in `frontend/src/components/PersonalizeButton.tsx`

**Checkpoint**: At this point, User Stories 1, 2, 3 AND 4 should all work independently

---

## Phase 7: User Story 5 - Urdu Translation (Priority: P3)

**Goal**: Logged-in student can translate the content of chapters into Urdu by pressing a button, so they can learn in their preferred language.

**Independent Test**: Log in, navigate to a chapter, activate Urdu translation, and verify that the chapter content is accurately displayed in Urdu.

### Tests for User Story 5
- [ ] T061 [US5] Backend unit test: Validate Urdu translation service integration (mock external service) in `backend/tests/unit/test_translation_service.py`
- [ ] T062 [US5] Frontend integration test: Verify Urdu translation button functionality and content display in `frontend/tests/integration/test_translation_feature.spec.js`

### Implementation for User Story 5
- [ ] T063 [US5] Create FastAPI endpoint for content translation requests in `backend/src/api/translation.py`
- [ ] T064 [US5] Implement `TranslationService` to translate content to Urdu (e.g., using an external translation API or pre-translated content) in `backend/src/services/translation_service.py`
- [ ] T065 [US5] Add `urdu_text` field to `Book Content` model in `backend/src/models/book_content.py`
- [ ] T066 [US5] Implement database migration to add `urdu_text` field to `Book Content` in `backend/migrations/`
- [ ] T067 [US5] Develop React `TranslateButton` component for Urdu in `frontend/src/components/TranslateButton.tsx`
- [ ] T068 [US5] Integrate `TranslateButton` into Docusaurus chapter layout in `frontend/src/theme/DocItem/Content/index.tsx`
- [ ] T069 [US5] Implement frontend logic to fetch and display Urdu content from backend API in `frontend/src/utils/translation_hook.ts`
- [ ] T070 [US5] Disable translation button if user is not logged in in `frontend/src/components/TranslateButton.tsx`

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T071 Setup frontend testing framework (e.g., Playwright/Cypress) in `frontend/`
- [ ] T072 Setup backend testing framework (e.g., pytest) in `backend/`
- [ ] T073 Configure CI/CD pipeline (GitHub Actions) for Docusaurus build/deploy to GitHub Pages in `.github/workflows/deploy.yml`
- [ ] T074 Configure CI/CD pipeline (GitHub Actions) for backend deployment in `.github/workflows/backend_deploy.yml`
- [ ] T075 [P] Review and update all documentation (README.md, existing docs)
- [ ] T076 Code cleanup and refactoring across frontend and backend
- [ ] T077 Performance optimization for content loading and API responses
- [ ] T078 Security hardening for authentication and data handling
- [ ] T079 Run quickstart.md validation (create quickstart.md as a separate task if needed)

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
