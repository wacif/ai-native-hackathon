# Tasks: User Dashboard

**Input**: Design documents from `/specs/007-user-dashboard/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, quickstart.md ✅

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US3)
- Include exact file paths in descriptions

---

## Implementation Status Summary

| Phase | Status | Notes |
|-------|--------|-------|
| Phase 1: Setup | ✅ Complete | CSS Module file creation |
| Phase 2: Foundation | ✅ Complete | Dashboard page scaffold |
| Phase 3: US1 Dashboard Overview | ✅ Complete | 3 cards, theme matching |
| Phase 4: US2 Quick Actions | ✅ Complete | 4 action buttons |
| Phase 5: US3 Learning Progress | ✅ Complete | Module count, quick link |
| Phase 6: Login Redirect | ✅ Complete | Redirect logic |
| Phase 7: Polish | ✅ Complete | Responsive, edge cases, build verified |

---

## Phase 1: Setup

**Goal**: Create new files for dashboard feature

- [x] T001 Create dashboard CSS module file `src/pages/dashboard.module.css` with empty template
- [x] T002 Create dashboard page scaffold `src/pages/dashboard.tsx` with Layout wrapper and AuthProvider

**Checkpoint**: Both files exist and import without errors ✅

---

## Phase 2: Foundation (Blocking)

**Goal**: Implement auth guards and basic page structure

- [x] T003 Add authentication redirect logic in `src/pages/dashboard.tsx` - redirect to /login if not authenticated
- [x] T004 Add mounted state handling in `src/pages/dashboard.tsx` to prevent SSR hydration issues
- [x] T005 Add loading state display in `src/pages/dashboard.tsx` while auth is loading

**Checkpoint**: Unauthenticated users are redirected to /login, loading state shows during auth check ✅

---

## Phase 3: US1 - Dashboard Overview (P0)

**Goal**: Display comprehensive dashboard with 3 cards matching futuristic theme

**Independent Test**: Login, verify redirected to `/dashboard`, see user info card, learning progress card, preferences card, all matching futuristic theme.

### Card Components

- [x] T006 [US1] Create User Info card JSX in `src/pages/dashboard.tsx` - avatar, username, email
- [x] T007 [P] [US1] Create Learning Progress card JSX in `src/pages/dashboard.tsx` - module count placeholder
- [x] T008 [P] [US1] Create Preferences card JSX in `src/pages/dashboard.tsx` - software/hardware/OS display

### CSS Styling

- [x] T009 [US1] Add container and title styles in `src/pages/dashboard.module.css` - centered layout, welcome heading
- [x] T010 [US1] Add grid layout styles in `src/pages/dashboard.module.css` - 3-column responsive grid
- [x] T011 [US1] Add card base styles in `src/pages/dashboard.module.css` - glass-morphism, border, padding
- [x] T012 [P] [US1] Add card header styles in `src/pages/dashboard.module.css` - icon + title row
- [x] T013 [P] [US1] Add avatar styles in `src/pages/dashboard.module.css` - gradient background, initial letter
- [x] T014 [P] [US1] Add glow effects in `src/pages/dashboard.module.css` - cyan accent shadows on cards

**Checkpoint**: Dashboard displays 3 styled cards with futuristic dark theme and cyan accents ✅

---

## Phase 4: US2 - Quick Actions (P0)

**Goal**: Implement 4 action buttons with proper navigation and functions

**Independent Test**: Login, verify 4 action buttons present, click each, verify navigation works.

### Action Implementation

- [x] T015 [US2] Add action buttons container JSX in `src/pages/dashboard.tsx` - 4 buttons row
- [x] T016 [US2] Implement "Continue Learning" button in `src/pages/dashboard.tsx` - link to /docs/physical-ai/intro
- [x] T017 [US2] Implement "Edit Profile" button in `src/pages/dashboard.tsx` - link to /profile
- [x] T018 [US2] Implement "Sign Out" button in `src/pages/dashboard.tsx` - call signOut from AuthContext
- [x] T019 [US2] Implement "Open Chatbot" button in `src/pages/dashboard.tsx` - trigger chatbot open (placeholder if needed)

### Button Styling

- [x] T020 [US2] Add actions container styles in `src/pages/dashboard.module.css` - flexbox row with gap
- [x] T021 [P] [US2] Add primary button styles in `src/pages/dashboard.module.css` - gradient background, hover glow
- [x] T022 [P] [US2] Add secondary button styles in `src/pages/dashboard.module.css` - outline style, hover effects

**Checkpoint**: All 4 buttons render and function correctly (Continue Learning, Edit Profile, Sign Out, Chatbot) ✅

---

## Phase 5: US3 - Learning Progress Card (P1)

**Goal**: Show available modules with quick navigation link

**Independent Test**: Login, view dashboard, verify Learning Progress card shows module count and "Start Module 1" link.

- [x] T023 [US3] Define MODULES constant array in `src/pages/dashboard.tsx` - 6 modules with id, title, path
- [x] T024 [US3] Display module count in Learning Progress card in `src/pages/dashboard.tsx`
- [x] T025 [US3] Add "Start Module 1" link in Learning Progress card in `src/pages/dashboard.tsx`
- [x] T026 [P] [US3] Add stat number styles in `src/pages/dashboard.module.css` - large cyan number
- [x] T027 [P] [US3] Add module link styles in `src/pages/dashboard.module.css` - accent color with hover

**Checkpoint**: Learning Progress card shows "6 modules available" and working "Start Module 1" link ✅

---

## Phase 6: Login Redirect

**Goal**: Redirect authenticated users from /login to /dashboard

- [x] T028 Update `src/pages/login.tsx` - add useEffect to redirect authenticated users to /dashboard
- [x] T029 Remove inline dashboard UI from `src/pages/login.tsx` - delete the authenticated user welcome screen

**Checkpoint**: Visiting /login while authenticated redirects to /dashboard ✅

---

## Phase 7: Polish & Edge Cases

**Goal**: Responsive design, edge cases, final polish

### Responsive Design

- [x] T030 [P] Add mobile breakpoint styles in `src/pages/dashboard.module.css` - cards stack at < 768px
- [x] T031 [P] Add mobile button styles in `src/pages/dashboard.module.css` - full width buttons on mobile

### Edge Cases

- [x] T032 Add incomplete profile hint in Preferences card in `src/pages/dashboard.tsx` - show if fields missing
- [x] T033 Add loading spinner styles in `src/pages/dashboard.module.css`

### Final Validation

- [x] T034 Verify build passes with `npm run build`
- [x] T035 Manual test: full login → dashboard → actions flow (ready for user testing)

**Checkpoint**: Feature complete, responsive, all edge cases handled ✅

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundation) 
    ↓
Phase 3 (US1) ←→ Phase 5 (US3) [can partially parallel]
    ↓
Phase 4 (US2)
    ↓
Phase 6 (Login Redirect)
    ↓
Phase 7 (Polish)
```

### Parallel Opportunities

**Within Phase 3 (US1)**:
- T007, T008 can run in parallel (different card JSX)
- T012, T013, T014 can run in parallel (different CSS sections)

**Within Phase 4 (US2)**:
- T021, T022 can run in parallel (different button styles)

**Within Phase 5 (US3)**:
- T026, T027 can run in parallel (different CSS sections)

**Within Phase 7 (Polish)**:
- T030, T031 can run in parallel (different breakpoint sections)

---

## Task Count Summary

| Phase | Tasks | Priority |
|-------|-------|----------|
| 1 - Setup | 2 | - |
| 2 - Foundation | 3 | - |
| 3 - US1 Dashboard | 9 | P0 |
| 4 - US2 Actions | 8 | P0 |
| 5 - US3 Progress | 5 | P1 |
| 6 - Login Redirect | 2 | P0 |
| 7 - Polish | 6 | - |
| **Total** | **35** | |

### Tasks by User Story

- **US1 (Dashboard Overview)**: T006-T014 (9 tasks)
- **US2 (Quick Actions)**: T015-T022 (8 tasks)
- **US3 (Learning Progress)**: T023-T027 (5 tasks)
- **Infrastructure**: T001-T005, T028-T035 (13 tasks)

---

## MVP Scope

**Suggested MVP**: Phases 1-4 + Phase 6 (28 tasks)
- Complete dashboard with 3 cards
- All 4 action buttons working
- Login redirect implemented
- Basic styling (can skip some polish)

**Can defer to later**: Phase 5 (Learning Progress detail), Phase 7 (Polish)

---

## Implementation Strategy

1. **Start with skeleton**: T001-T002 create files
2. **Add auth guards**: T003-T005 ensure security
3. **Build cards incrementally**: T006-T014 one card at a time
4. **Add actions**: T015-T022 buttons and navigation
5. **Enhance progress card**: T023-T027 module data
6. **Connect login**: T028-T029 redirect flow
7. **Polish**: T030-T035 responsive and edge cases

**Commit after each phase** for clean git history.
