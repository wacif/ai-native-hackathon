# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-robotics-textbook` | **Date**: 2025-11-28 | **Last Updated**: 2025-11-30 | **Spec**: specs/001-physical-ai-robotics-textbook/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This project aims to create an AI-native textbook for teaching Physical AI & Humanoid Robotics. It includes core textbook content, an integrated RAG chatbot, user authentication, content personalization, and Urdu translation capabilities.

## Implementation Status

| User Story | Status | Completion Date |
|------------|--------|----------------|
| US1: Core Textbook Content | ✅ Complete | 2025-11-29 |
| US2: Integrated RAG Chatbot | ✅ Complete | 2025-11-29 |
| US3: User Authentication | ✅ Complete | 2025-11-30 |
| US4: Content Personalization | ✅ Complete | 2025-11-30 |
| US5: Urdu Translation | ⏳ Not Started | - |

**Live Site**: https://wacif.github.io/ai-native-hackathon/
**PR**: https://github.com/wacif/ai-native-hackathon/pull/7

## Technical Context

**Language/Version**: TypeScript/JavaScript (Docusaurus 3.9.2 frontend), Python 3.11+ (FastAPI backend)
**Primary Dependencies**: Docusaurus 3.9.2, OpenAI Agents SDK with Google Gemini, FastAPI, Neon Serverless Postgres, Qdrant Cloud, FastEmbed (BAAI/bge-small-en-v1.5)
**Authentication**: FastAPI Native JWT Auth (OAuth2 + bcrypt) - *Changed from Better-Auth for simpler single-backend deployment*
**Storage**: Neon Serverless Postgres (user data, personalized content cache), Qdrant Cloud (vector embeddings for RAG)
**Testing**: pytest (backend), Playwright/Cypress (frontend - planned)
**Target Platform**: GitHub Pages (Docusaurus static site), Linux server (FastAPI backend on port 8000)
**Project Type**: Web application (Docusaurus frontend, FastAPI backend)
**Performance Goals**:
  - SC-001: Published book accessible and content displayed correctly within 5 seconds.
  - SC-002/SC-003: RAG chatbot answers 90% of questions accurately within 3 seconds.
  - SC-005: Content personalization visible within 2 seconds.
  - SC-006: Urdu translation displayed within 2 seconds.
**Constraints**: None explicitly mentioned beyond performance goals.
**Scale/Scope**: AI-native textbook with an integrated chatbot, user authentication, content personalization, and translation, targeting an audience with potential for 10k users requiring personalization/translation features.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Constitutional Persona & Core Capabilities
- [X] **Educational Systems Architect**: The plan aligns with designing AI-native educational experiences that activate reasoning.
- [X] **Distinctive Educational Experience**: Avoids generic patterns, focusing on decision frameworks and meta-awareness.

### Core Principles for All Reasoning
- [X] **Right Altitude Balance**: The plan will prioritize decision frameworks over rigid rules.
- [X] **Decision Frameworks Over Rules**: The plan will ensure students understand 'WHAT' before 'HOW'.
- [X] **Meta-Awareness Against Convergence**: The plan will incorporate varied teaching modalities.

### Technical Stack & Tools Alignment
- [X] **Docusaurus 3.9.2**: Confirmed for book generation and GitHub Pages deployment.
- [X] **OpenAI Agents SDK + Gemini, FastAPI, Neon Serverless Postgres, Qdrant Cloud**: Confirmed for RAG chatbot.
- [X] **FastAPI Native JWT Auth**: Implemented (replaced Better-Auth for simpler deployment).
- [X] **Claude Code & Spec-Kit Plus**: Confirmed for development workflow.

### Development Workflow & Quality Gates
- [X] **Spec-Driven Development (SDD)**: The project follows SDD principles.
- [X] **Prompt History Records (PHRs)**: PHRs will be created for every user interaction.
- [X] **Architectural Decision Records (ADRs)**: ADRs will be suggested for significant decisions.
- [ ] **Automated Testing & CI/CD**: Placeholder - details for specific testing frameworks and CI/CD pipeline are not yet elaborated in the spec, but the general commitment is present. This will be detailed in later phases.

**Gate Check Result**: All core constitutional principles are aligned. Specific details for Automated Testing & CI/CD will be elaborated in subsequent planning phases.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Frontend (root level)
src/
├── components/
│   ├── Auth/              # SignupForm, LoginForm, UserButton
│   ├── Chatbot/           # RAG chatbot UI component
│   ├── HomepageFeatures/  # Landing page features
│   └── PersonalizeButton/ # Chapter personalization
├── context/
│   └── AuthContext.tsx    # React auth state management
├── css/
│   └── custom.css         # Global styles
├── pages/
│   ├── index.tsx          # Homepage
│   ├── login.tsx          # Login page
│   └── signup.tsx         # Signup page
└── theme/
    ├── Root.tsx           # App wrapper with providers
    ├── DocItem/Content/   # Swizzled for personalize button
    ├── Navbar/            # Custom navbar with auth
    └── NavbarItem/        # Custom navbar items

docs/
└── physical-ai/           # Textbook chapters (intro, ch1-3)

# FastAPI Backend
backend/
├── src/
│   ├── api/
│   │   └── auth.py        # Auth endpoints (/signup, /signin, /me)
│   ├── chatbot/
│   │   └── agent.py       # RAG agent with personalization
│   ├── config/
│   │   ├── auth.py        # JWT configuration
│   │   ├── db.py          # PostgreSQL connection
│   │   └── qdrant.py      # Qdrant Cloud client
│   ├── models/
│   │   ├── user.py        # User model with personalization fields
│   │   ├── book_content.py
│   │   ├── chatbot_interaction.py
│   │   └── personalized_content.py  # LLM content cache
│   ├── rag/
│   │   └── ingestion.py   # Content ingestion pipeline
│   ├── services/
│   │   ├── auth_service.py
│   │   ├── embedding_service.py
│   │   └── qdrant_service.py
│   └── utils/
│       ├── logger.py
│       └── errors.py
├── migrations/            # Alembic migrations
└── main.py                # FastAPI app entry point
```

**Structure Decision**: Web application with Docusaurus frontend at root level and FastAPI backend in `backend/` directory. This structure was chosen because Docusaurus requires root-level configuration files (`docusaurus.config.ts`, `sidebars.ts`) while keeping backend isolated.

## Project Setup Verification

### Environment and Tooling Prerequisites

Explicit checks for `alembic` and `gh` CLI tools MUST be performed before any migration or PR creation tasks. `.env` files MUST be used to manage `DATABASE_URL` and other sensitive environment variables, ensuring they are loaded correctly by the application and tooling (e.g., in `env.py` for Alembic).

### Foundational File & Module Verification

Before critical operations (e.g., database migrations, API startup), all foundational project files (e.g., ORM base models like `backend/src/models/base.py`, core configuration files) MUST be explicitly defined and their existence verified. Ensure that Python module import paths (e.g., `sys.path`) are correctly configured to allow resolution of internal project modules across different execution contexts (e.g., when running scripts from the project root versus within `backend/`).

### Environment Variable Propagation for Tools

Environment variables, particularly sensitive ones like `DATABASE_URL`, MUST be explicitly loaded within relevant scripts (e.g., `backend/migrations/env.py` using `dotenv.load_dotenv()`) and accessed directly via `os.environ`. Tools and frameworks (e.g., Alembic) should NOT rely on their configuration files for environment variable expansion, but rather on the script's explicit handling of `os.environ` after `.env` loading. This ensures consistent and secure access to configuration across different execution environments.

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
