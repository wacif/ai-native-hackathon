---
id: 0012
title: Phase 5 - FastAPI Native Authentication Implementation
stage: feature
date: 2025-11-29
surface: agent
model: claude-opus-4-5-20250929
feature: user-authentication
branch: 001-physical-ai-robotics-textbook
user: wasi
command: user-request
labels: ["authentication", "jwt", "fastapi", "react", "phase-5", "user-story-3"]
links:
  spec: specs/001-physical-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/requirements.txt
  - backend/src/config/auth.py
  - backend/src/services/auth_service.py
  - backend/src/api/auth.py
  - backend/src/main.py
  - src/context/AuthContext.tsx
  - src/components/Auth/SignupForm.tsx
  - src/components/Auth/LoginForm.tsx
  - src/components/Auth/UserButton.tsx
  - src/components/Auth/Auth.module.css
  - src/components/Auth/index.tsx
  - src/theme/Root.tsx
  - specs/001-physical-ai-robotics-textbook/tasks.md
  - specs/001-physical-ai-robotics-textbook/contracts/auth.md
tests:
  - none (manual API testing via /docs)
---

## Prompt

User requested implementation of authentication for User Story 3 (Phase 5). Initial request was to research Better-Auth integration using Context7.

## Context

**User Story 3**: Sign up and sign in using a secure authentication system, providing software and hardware background, to access personalized content and features.

**Original Plan**: Use Better-Auth for authentication

**Architecture Discovery**: Better-Auth is a TypeScript/Node.js framework, while the project backend is Python/FastAPI. This creates an architectural mismatch.

## Analysis Performed

### Better-Auth Research (via Context7)

Researched Better-Auth documentation and found:
- Better-Auth is a TypeScript-first authentication framework
- Runs on Node.js, not Python
- Provides excellent React hooks (`useSession`, etc.)
- Has a rich plugin ecosystem (2FA, passkeys, SSO)

### Integration Options Evaluated

| Option | Description | Effort | Recommendation |
|--------|-------------|--------|----------------|
| A | Hybrid: Node.js auth service + FastAPI backend | High (2-3 days) | Complex, two services |
| B | FastAPI Native JWT Auth | Medium (1-2 days) | **Recommended** |
| C | Better-Auth as Auth Gateway | Very High (3-4 days) | Overkill |

### Decision Made

**Option B: FastAPI Native Auth** selected because:
1. Single backend (Python) - no Node.js service needed
2. Simpler deployment for hackathon context
3. Existing User model already has all required fields
4. FastAPI has excellent auth utilities (OAuth2, JWT)
5. Can create React hooks similar to Better-Auth's `useSession`

## Implementation Summary

### Backend (Python/FastAPI)

**1. Dependencies Added** (`backend/requirements.txt`):
```
pyjwt==2.8.0
passlib[bcrypt]==1.7.4
python-multipart==0.0.6
email-validator==2.3.0
```

**2. JWT Configuration** (`backend/src/config/auth.py`):
- JWT_SECRET_KEY (from env)
- HS256 algorithm
- 1-hour access tokens
- 7-day refresh tokens
- bcrypt with 12 rounds

**3. Auth Service** (`backend/src/services/auth_service.py`):
- Password hashing/verification (bcrypt)
- JWT token creation/validation
- User CRUD operations
- Password validation (8+ chars, uppercase, lowercase, digit)

**4. Auth Endpoints** (`backend/src/api/auth.py`):
- `POST /auth/signup` - Register new user
- `POST /auth/signin` - Login (OAuth2 form)
- `POST /auth/refresh` - Refresh access token
- `GET /auth/me` - Get current user profile
- `PATCH /auth/me` - Update user profile

### Frontend (React/Docusaurus)

**1. AuthContext** (`src/context/AuthContext.tsx`):
- `useAuth()` hook - full auth API
- `useSession()` hook - similar to Better-Auth
- localStorage persistence
- Token refresh mechanism

**2. Auth Components** (`src/components/Auth/`):
- `SignupForm` - Two-step form (credentials + background info)
- `LoginForm` - Email/password login
- `UserButton` - Navbar dropdown with avatar

**3. App Integration** (`src/theme/Root.tsx`):
- Wrapped app with `<AuthProvider>`

## API Contract

### Signup Request
```json
POST /auth/signup
{
  "username": "string",
  "email": "user@example.com",
  "password": "string",
  "software_background": "beginner|intermediate|advanced|expert",
  "hardware_background": "beginner|intermediate|advanced|expert"
}
```

### Signin Request (OAuth2 Form)
```
POST /auth/signin
Content-Type: application/x-www-form-urlencoded

username=user@example.com&password=yourpassword
```

### Response
```json
{
  "access_token": "eyJ...",
  "refresh_token": "eyJ...",
  "token_type": "bearer",
  "expires_in": 3600
}
```

## Tasks Completed

- [X] T042 - Configure FastAPI JWT auth in `backend/src/config/auth.py`
- [X] T043 - Create FastAPI endpoints in `backend/src/api/auth.py`
- [X] T044 - Implement AuthService in `backend/src/services/auth_service.py`
- [X] T045 - Develop SignupForm in `src/components/Auth/SignupForm.tsx`
- [X] T046 - Develop LoginForm in `src/components/Auth/LoginForm.tsx`
- [X] T047 - Implement AuthContext in `src/context/AuthContext.tsx`
- [X] T048 - Collect background info during signup (in SignupForm)
- [X] T049 - User model fields (already existed)
- [X] T050 - Database migration (already existed)
- [X] T050.1 - Create UserButton component
- [X] T050.2 - Wrap app with AuthProvider

## Outcome

- ✅ Impact: Full authentication system implemented for User Story 3
- 📁 Files: 14 files created/modified (see list above)
- 🔁 Next prompts: 
  - Integrate UserButton into Docusaurus navbar
  - Create auth page routes (/login, /signup)
  - Test full auth flow
  - Proceed to Phase 6 (Content Personalization)
- 🧠 Reflection: FastAPI native auth was the right choice - simpler, faster, and provides similar UX to Better-Auth through custom React hooks

## Verification

Backend auth routes confirmed working:
```
Routes: ['/auth/signup', '/auth/signin', '/auth/refresh', '/auth/me', '/auth/me', ...]
```

## Evaluation Notes (Flywheel)

- **Failure modes observed**: Better-Auth architectural mismatch (TypeScript vs Python)
- **Resolution**: Chose FastAPI native JWT auth instead
- **Lesson learned**: Always verify library language/runtime compatibility before planning integration
- **Next experiment**: Test full auth flow end-to-end before moving to personalization

---

## Constitutional Compliance Check

- ✅ Used Context7 for library research (Better-Auth docs)
- ✅ Virtual environment activated for all pip commands
- ✅ Created PHR documenting the interaction
- ✅ Updated tasks.md with completion status
- ✅ Updated auth.md contract with implementation status
- ✅ Decision framework applied (evaluated 3 options, chose simplest)
- ✅ Anti-convergence: Custom React hooks instead of generic auth library

