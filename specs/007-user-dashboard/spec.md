# Feature Specification: User Dashboard

**Feature Branch**: `007-user-dashboard`  
**Created**: 2025-12-01  
**Status**: Draft 📝  
**Input**: User observation: "After successful login, user sees a basic welcome screen showing username, email, software/hardware background, and language. This needs improvement."

## Clarifications

### Session 2025-12-01

- Q: Should dashboard be inline in login.tsx or separate component? → A: Separate Dashboard component, redirect login to dashboard when authenticated
- Q: What should dashboard display beyond user info? → A: Summary cards - User info + Learning progress card + Preferences card
- Q: What should learning progress card show? → A: Simple module count + "Start Module 1" link (no backend tracking)
- Q: Should dashboard have separate URL route? → A: New `/dashboard` route, redirect authenticated users from `/login`
- Q: What actions should be available? → A: Continue Learning, Edit Profile, Sign Out, Open Chatbot (4 actions)

## Problem Statement

Currently when a logged-in user visits `/login`, they see a minimal welcome screen:
- Avatar with initial
- Username + email
- Software Background / Hardware Background / Language
- "Continue Learning" button

This screen lacks:
- Quick access to key actions
- Learning progress visibility
- Navigation to settings
- Proper dashboard layout matching the futuristic theme

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Dashboard Overview (Priority: P0)

As a logged-in user, I want to see a comprehensive dashboard at `/dashboard` after login so that I can quickly access my learning journey and settings.

**Why this priority**: First impression after login should showcase the platform value.

**Independent Test**: Login, verify redirected to `/dashboard`, see user info card, learning progress card, preferences card, all matching futuristic theme.

**Acceptance Scenarios**:

1. **Given** I just logged in, **When** authentication succeeds, **Then** I am redirected to `/dashboard`.
2. **Given** I am on the dashboard, **When** I view the page, **Then** I see three cards: User Info, Learning Progress, Preferences Summary.
3. **Given** I am on the dashboard, **When** I view the page, **Then** it matches the futuristic dark theme with cyan accents.
4. **Given** I visit `/login` while authenticated, **When** page loads, **Then** I am redirected to `/dashboard`.

---

### User Story 2 - Quick Actions (Priority: P0)

As a logged-in user, I want four quick action buttons on my dashboard so that I can navigate to common destinations with one click.

**Why this priority**: Reduces friction for primary user journeys.

**Independent Test**: Login, verify 4 action buttons present, click each, verify navigation works.

**Acceptance Scenarios**:

1. **Given** I am on the dashboard, **When** I click "Continue Learning", **Then** I go to `/docs/physical-ai/intro`.
2. **Given** I am on the dashboard, **When** I click "Edit Profile", **Then** I go to `/profile`.
3. **Given** I am on the dashboard, **When** I click "Sign Out", **Then** I am logged out and redirected to `/login`.
4. **Given** I am on the dashboard, **When** I click "Open Chatbot", **Then** the chatbot opens.

---

### User Story 3 - Learning Progress Card (Priority: P1)

As a logged-in user, I want to see available modules on my dashboard so that I can quickly start or continue learning.

**Why this priority**: Helps users navigate to content.

**Independent Test**: Login, view dashboard, verify Learning Progress card shows module count and "Start Module 1" link.

**Acceptance Scenarios**:

1. **Given** I am on the dashboard, **When** I view the Learning Progress card, **Then** I see count of available modules.
2. **Given** I am on the dashboard, **When** I click "Start Module 1", **Then** I go to the first module.

---

### Edge Cases

- What happens if user has incomplete profile? → Show prompt to complete profile in Preferences card.
- What happens if user is not authenticated and visits `/dashboard`? → Redirect to `/login`.
- What happens on mobile? → Dashboard cards stack vertically.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST create new `/dashboard` page route.
- **FR-002**: System MUST redirect authenticated users from `/login` to `/dashboard`.
- **FR-003**: System MUST redirect unauthenticated users from `/dashboard` to `/login`.
- **FR-004**: Dashboard MUST display User Info card with avatar, username, email.
- **FR-005**: Dashboard MUST display Learning Progress card with module count and quick link.
- **FR-006**: Dashboard MUST display Preferences Summary card with current settings.
- **FR-007**: Dashboard MUST provide 4 action buttons: Continue Learning, Edit Profile, Sign Out, Open Chatbot.
- **FR-008**: Dashboard MUST match futuristic dark theme (cyan accents, glow effects, glass cards).
- **FR-009**: Dashboard MUST be responsive (cards stack on mobile).

### Technical Requirements

- **TR-001**: Create `src/pages/dashboard.tsx` as new route.
- **TR-002**: Create `src/pages/dashboard.module.css` for styling.
- **TR-003**: Update `src/pages/login.tsx` to redirect when authenticated.
- **TR-004**: Uses existing AuthContext for user data and logout.
- **TR-005**: CSS follows established custom.css patterns (glass effect, glow, cyan accents).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Dashboard loads in < 1 second
- **SC-002**: All 4 CTAs navigate/function correctly
- **SC-003**: Visual consistency with landing page theme (same CSS variables)
- **SC-004**: Responsive layout works on mobile (< 768px)

## Implementation Status

| Requirement | Status | Notes |
|-------------|--------|-------|
| FR-001 | ❌ Missing | No /dashboard page exists |
| FR-002 | ❌ Missing | Login shows inline welcome |
| FR-003 | ❌ Missing | No redirect logic |
| FR-004 | ⚠️ Partial | Basic avatar in login.tsx |
| FR-005 | ❌ Missing | No progress card |
| FR-006 | ⚠️ Partial | Shows fields but poor styling |
| FR-007 | ⚠️ Partial | Only "Continue Learning" exists |
| FR-008 | ❌ Missing | Uses basic inline styles |
| FR-009 | ❌ Missing | Not responsive |

**Feature Status**: ❌ NEEDS IMPLEMENTATION

