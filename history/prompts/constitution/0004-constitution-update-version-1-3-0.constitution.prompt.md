---
id: 0004
title: Constitution Update to Version 1.3.0 - Technical Stack Alignment
stage: constitution
date: 2025-11-30
surface: agent
model: claude-opus-4-5-20250929
feature: none
branch: 001-physical-ai-robotics-textbook
user: wasi
command: manual
labels: ["constitution", "technical-stack", "authentication", "version-control"]
links:
  spec: specs/001-physical-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: https://github.com/wacif/ai-native-hackathon/pull/7
files:
  - .specify/memory/constitution.md
  - specs/001-physical-ai-robotics-textbook/plan.md
  - specs/001-physical-ai-robotics-textbook/tasks.md
  - history/prompts/constitution/0004-constitution-update-version-1-3-0.constitution.prompt.md
tests:
  - none
---

## Prompt

User requested alignment of constitution.md and plan.md with the completed tasks documented in tasks.md. This required updating the Technical Stack section to reflect the actual implementation choices made during development.

## Context

**Previous Constitution Version**: 1.2.3
**New Constitution Version**: 1.3.0
**Change Type**: MINOR (Technical Stack & Tools section updated)

**Trigger**: All four user stories (US1-US4) completed, requiring documentation alignment across all spec files.

## Changes from Version 1.2.3 → 1.3.0

### Technical Stack & Tools Section Updated

**Previous (1.2.3)**:
```
The project will utilize Docusaurus for book generation and deployment to GitHub Pages. 
The RAG chatbot will be built with OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud. 
Better-Auth will be used for Signup/Signin. 
Development will be driven by Claude Code and Spec-Kit Plus.
```

**Updated (1.3.0)**:
```
The project utilizes Docusaurus 3.9.2 for book generation and deployment to GitHub Pages. 
The RAG chatbot is built with OpenAI Agents SDK (with Google Gemini as LLM), FastAPI, Neon Serverless Postgres, Qdrant Cloud, and FastEmbed (BAAI/bge-small-en-v1.5 for embeddings). 
FastAPI Native JWT Auth (OAuth2 + bcrypt) is used for Signup/Signin - chosen over Better-Auth for simpler single-backend Python deployment. 
Development is driven by Claude Code and Spec-Kit Plus.
```

### Key Changes

| Aspect | Before (1.2.3) | After (1.3.0) |
|--------|----------------|---------------|
| Docusaurus | Unspecified version | 3.9.2 |
| LLM | OpenAI Agents/ChatKit SDKs | OpenAI Agents SDK + Google Gemini |
| Embeddings | Not specified | FastEmbed (BAAI/bge-small-en-v1.5) |
| Authentication | Better-Auth | FastAPI Native JWT Auth (OAuth2 + bcrypt) |
| Tense | Future ("will utilize") | Present ("utilizes") |

### Rationale for Authentication Change

Better-Auth was replaced with FastAPI Native JWT Auth because:

1. **Single Backend**: Project uses Python/FastAPI only - no Node.js service needed
2. **Simpler Deployment**: One less service to deploy and manage for hackathon
3. **Existing User Model**: SQLAlchemy User model already compatible with JWT approach
4. **FastAPI Utilities**: Excellent built-in OAuth2 and JWT support

## Sync Impact Report

```
Version change: 1.2.3 -> 1.3.0 (MINOR: Updated Technical Stack to reflect actual implementation)
List of modified principles:
- Technical Stack & Tools: Updated to reflect FastAPI Native JWT Auth (replaced Better-Auth)
- Technical Stack & Tools: Added specific versions and technologies (Gemini, FastEmbed)
Added sections: None
Removed sections: None
Templates requiring updates:
- specs/001-physical-ai-robotics-textbook/plan.md ✅ updated
- specs/001-physical-ai-robotics-textbook/tasks.md ✅ updated
Follow-up TODOs: None
```

## Related Updates

### plan.md Updates
- Added "Last Updated" date (2025-11-30)
- Added Implementation Status table showing US1-US4 complete
- Added live site URL and PR link
- Updated Technical Context section
- Updated Technical Stack checklist
- Replaced placeholder project structure with actual implementation

### tasks.md Updates (prior to this session)
- Phase 5 (US3 Auth): Marked complete with FastAPI JWT implementation
- Phase 6 (US4 Personalization): Marked complete with all 22 tasks
- All completion status boxes updated

## Current Implementation Status

| User Story | Status | Description |
|------------|--------|-------------|
| US1: Core Textbook Content | ✅ Complete | 4 Physical AI chapters in docs/ |
| US2: Integrated RAG Chatbot | ✅ Complete | Gemini + Qdrant + FastEmbed |
| US3: User Authentication | ✅ Complete | FastAPI JWT + OAuth2 + bcrypt |
| US4: Content Personalization | ✅ Complete | 6 profile fields + personalized content |
| US5: Urdu Translation | ⏳ Not Started | - |

## Response Summary

Updated constitution.md from version 1.2.3 to 1.3.0 to align with actual implementation. The primary change was documenting the authentication technology decision (FastAPI Native JWT instead of Better-Auth) and adding specific versions/technologies for the stack.

## Outcome

- ✅ Impact: Constitution, plan, and tasks files now aligned and reflect actual implementation
- 📁 Files Updated:
  - `.specify/memory/constitution.md` (1.2.3 → 1.3.0)
  - `specs/001-physical-ai-robotics-textbook/plan.md` (structure + status)
  - `history/prompts/constitution/0004-constitution-update-version-1-3-0.constitution.prompt.md` (new)
- 🔁 Next prompts: Continue with US5 (Urdu Translation) or merge PR #7
- 🧠 Reflection: Keeping constitutional documents aligned with implementation ensures accurate project governance and helps future agents understand architectural decisions

## Architectural Decision: FastAPI Native JWT over Better-Auth

**Decision**: Use FastAPI Native JWT Auth instead of Better-Auth

**Status**: Accepted

**Context**: 
- Original spec called for Better-Auth (JavaScript-based auth library)
- Project has single Python backend (FastAPI)
- Hackathon timeline requires simple deployment

**Decision Rationale**:
- FastAPI has excellent built-in OAuth2 support
- JWT + bcrypt is standard, well-understood pattern
- No need to add Node.js service just for auth
- Existing SQLAlchemy User model works directly

**Consequences**:
- ✅ Simpler deployment (single Python service)
- ✅ Faster implementation
- ✅ No cross-service auth complexity
- ⚠️ Manual refresh token rotation (implemented)
- ⚠️ No social login out-of-box (not required for MVP)

---

## Constitutional Compliance Check

This update follows constitutional principles:

- ✅ Creates PHR for constitution amendment
- ✅ Uses semantic versioning (MINOR bump for non-breaking change)
- ✅ Documents rationale for architectural decisions
- ✅ Maintains sync between governance docs and implementation
- ✅ Provides clear context for future agents
