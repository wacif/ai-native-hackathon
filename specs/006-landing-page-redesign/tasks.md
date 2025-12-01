# Tasks: Landing Page Redesign & Color Scheme

**Input**: Design documents from `/specs/006-landing-page-redesign/`
**Prerequisites**: plan.md ✅, spec.md ✅

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Global Theme Foundation)

**Purpose**: Establish color scheme and dark mode - MUST complete before components

- [ ] T001 [P] Update color variables in `src/css/custom.css` with new palette (#0a0a0f background, #00D4FF accent)
- [ ] T002 [P] Force dark mode default in `docusaurus.config.ts` (defaultMode: 'dark', disableSwitch: true)
- [ ] T003 [P] Create `src/components/Landing/` directory structure
- [ ] T004 Add CSS keyframe animations (fadeInUp, float, glow, blink) in `src/css/custom.css`
- [ ] T005 Add reduced motion media query support in `src/css/custom.css`

**Checkpoint**: Dark theme active, accent colors visible on existing pages

---

## Phase 2: User Story 1 - Impressive First Impression (Priority: P0) 🎯 MVP

**Goal**: Hero section with particles, typing effect, and CTA that wows visitors

**Independent Test**: Load homepage, see animated hero with particles and typing headline

### Implementation

- [ ] T006 [P] [US1] Create `src/components/Landing/ParticleBackground/index.tsx` - CSS-based floating particles
- [ ] T007 [P] [US1] Create `src/components/Landing/ParticleBackground/styles.module.css` - particle animations
- [ ] T008 [P] [US1] Create `src/components/Landing/AnimatedText/index.tsx` - typing effect component
- [ ] T009 [P] [US1] Create `src/components/Landing/AnimatedText/styles.module.css` - cursor blink animation
- [ ] T010 [US1] Create `src/components/Landing/HeroSection/index.tsx` - compose particles + text + CTA
- [ ] T011 [US1] Create `src/components/Landing/HeroSection/styles.module.css` - hero layout, glow effects
- [ ] T012 [US1] Update `src/pages/index.tsx` - replace HomepageHeader with HeroSection
- [ ] T013 [US1] Update `src/pages/index.module.css` - remove old hero styles, add new page styles

**Checkpoint**: Homepage shows animated hero with particles, typing text, and glowing CTA

---

## Phase 3: User Story 2 - Clear Value Proposition (Priority: P0)

**Goal**: Feature cards highlighting ROS 2, Simulation, Isaac, VLA

**Independent Test**: Scroll to features section, see 4 cards with hover effects

### Implementation

- [ ] T014 [P] [US2] Create `src/components/Landing/FeatureCard/index.tsx` - single feature card component
- [ ] T015 [P] [US2] Create `src/components/Landing/FeatureCard/styles.module.css` - card styling, hover glow
- [ ] T016 [US2] Create `src/components/Landing/FeaturesSection/index.tsx` - grid of 4 feature cards
- [ ] T017 [US2] Create `src/components/Landing/FeaturesSection/styles.module.css` - section layout, grid
- [ ] T018 [US2] Update `src/pages/index.tsx` - replace HomepageFeatures with FeaturesSection
- [ ] T019 [US2] Define feature content (icons, titles, descriptions) for ROS2, Simulation, Isaac, VLA

**Checkpoint**: Homepage shows 4 feature cards with hover effects below hero

---

## Phase 4: User Story 3 - Consistent Brand Experience (Priority: P1)

**Goal**: Apply dark theme + cyan accents to navbar, sidebar, docs, footer

**Independent Test**: Navigate from homepage to any doc page, verify consistent theming

### Implementation

- [ ] T020 [P] [US3] Style navbar in `src/css/custom.css` - dark background, cyan active states
- [ ] T021 [P] [US3] Style sidebar in `src/css/custom.css` - dark background, cyan highlights
- [ ] T022 [P] [US3] Style doc content area in `src/css/custom.css` - dark background, cyan links
- [ ] T023 [P] [US3] Style footer in `src/css/custom.css` - dark background, muted text
- [ ] T024 [US3] Style code blocks in `src/css/custom.css` - dark theme, cyan accents
- [ ] T025 [US3] Update `docusaurus.config.ts` footer config if needed

**Checkpoint**: All pages (landing, docs, navbar, footer) share consistent dark + cyan theme

---

## Phase 5: User Story 4 - Engaging Interactions (Priority: P1)

**Goal**: Scroll animations, hover effects, micro-interactions

**Independent Test**: Scroll page, see elements fade in; hover cards/buttons, see glow effects

### Implementation

- [ ] T026 [P] [US4] Create `src/hooks/useScrollAnimation.ts` - Intersection Observer hook
- [ ] T027 [US4] Apply scroll animation to HeroSection content in `src/components/Landing/HeroSection/index.tsx`
- [ ] T028 [US4] Apply scroll animation to FeatureCards (staggered delay) in `src/components/Landing/FeaturesSection/index.tsx`
- [ ] T029 [US4] Add button hover glow effect in `src/css/custom.css`
- [ ] T030 [US4] Add link hover transitions in `src/css/custom.css`

**Checkpoint**: Page has smooth scroll-triggered animations and interactive hover states

---

## Phase 6: User Story 5 - Mobile Responsive (Priority: P1)

**Goal**: Landing page looks great on mobile devices

**Independent Test**: View homepage at 375px width, verify readable layout

### Implementation

- [ ] T031 [P] [US5] Add mobile styles to `src/components/Landing/HeroSection/styles.module.css`
- [ ] T032 [P] [US5] Add mobile styles to `src/components/Landing/FeaturesSection/styles.module.css`
- [ ] T033 [P] [US5] Add mobile styles to `src/components/Landing/FeatureCard/styles.module.css`
- [ ] T034 [US5] Reduce particle count on mobile in `src/components/Landing/ParticleBackground/index.tsx`
- [ ] T035 [US5] Test and fix navbar mobile menu theming in `src/css/custom.css`

**Checkpoint**: Homepage fully responsive at 375px, 768px, 1024px, 1440px

---

## Phase 7: Polish & Validation

**Purpose**: Final cleanup, performance, accessibility

- [ ] T036 [P] Remove old `src/components/HomepageFeatures/` if no longer used
- [ ] T037 [P] Verify WCAG AA contrast ratios for all text/background combinations
- [ ] T038 Run Lighthouse audit - target Performance > 80, Accessibility > 90
- [ ] T039 Test `prefers-reduced-motion` disables non-essential animations
- [ ] T040 Cross-browser testing (Chrome, Firefox, Safari)
- [ ] T041 Final visual QA on all doc pages for theme consistency

**Checkpoint**: Feature complete, performant, accessible, cross-browser compatible

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (US1: Hero) ←── Phase 3 (US2: Features)
    ↓                         ↓
    └─────────┬───────────────┘
              ↓
         Phase 4 (US3: App-Wide Theme)
              ↓
         Phase 5 (US4: Animations)
              ↓
         Phase 6 (US5: Responsive)
              ↓
         Phase 7 (Polish)
```

### Parallel Opportunities

**Phase 1**: T001, T002, T003 can run in parallel
**Phase 2**: T006, T007, T008, T009 can run in parallel (different components)
**Phase 3**: T014, T015 can run in parallel
**Phase 4**: T020, T021, T022, T023 can run in parallel (different CSS sections)
**Phase 6**: T031, T032, T033 can run in parallel (different component styles)

---

## Task Count Summary

| Phase | Tasks | Parallel | Description |
|-------|-------|----------|-------------|
| 1 | 5 | 3 | Setup & Foundation |
| 2 | 8 | 4 | Hero Section (US1) |
| 3 | 6 | 2 | Features Section (US2) |
| 4 | 6 | 4 | App-Wide Theme (US3) |
| 5 | 5 | 1 | Animations (US4) |
| 6 | 5 | 3 | Responsive (US5) |
| 7 | 6 | 2 | Polish |
| **Total** | **41** | **19** | |

---

## File Creation Summary

### New Files (10)
1. `src/components/Landing/ParticleBackground/index.tsx`
2. `src/components/Landing/ParticleBackground/styles.module.css`
3. `src/components/Landing/AnimatedText/index.tsx`
4. `src/components/Landing/AnimatedText/styles.module.css`
5. `src/components/Landing/HeroSection/index.tsx`
6. `src/components/Landing/HeroSection/styles.module.css`
7. `src/components/Landing/FeatureCard/index.tsx`
8. `src/components/Landing/FeatureCard/styles.module.css`
9. `src/components/Landing/FeaturesSection/index.tsx`
10. `src/components/Landing/FeaturesSection/styles.module.css`
11. `src/hooks/useScrollAnimation.ts`

### Modified Files (4)
1. `src/css/custom.css`
2. `src/pages/index.tsx`
3. `src/pages/index.module.css`
4. `docusaurus.config.ts`

---

## Notes

- Complete Phase 1 before any component work
- US1 (Hero) and US2 (Features) can be developed in parallel after Phase 1
- Each checkpoint allows for demo/validation before proceeding
- Commit after each task or logical group
- Test reduced motion after Phase 5 (T039)
