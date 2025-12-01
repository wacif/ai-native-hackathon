# Research: User Dashboard

**Feature**: 007-user-dashboard  
**Date**: 2025-12-01

## Research Questions

### 1. Dashboard Page Structure in Docusaurus

**Decision**: Create `src/pages/dashboard.tsx` as a standard React page

**Rationale**: Docusaurus automatically routes files in `src/pages/` to their corresponding URL. A file at `dashboard.tsx` becomes `/dashboard` route (or `/ai-native-hackathon/dashboard` with baseUrl).

**Alternatives Considered**:
- MDX page: Rejected - need full React control for auth logic
- Custom plugin: Rejected - overkill for single page

### 2. Authentication Redirect Pattern

**Decision**: Use `useEffect` with client-side redirect via `window.location`

**Rationale**: 
- Docusaurus is SSG, no server-side redirects
- AuthContext already handles auth state
- Pattern already used in existing pages (profile.tsx)

**Implementation**:
```tsx
// In dashboard.tsx
useEffect(() => {
  if (!isLoading && !isAuthenticated) {
    window.location.href = '/ai-native-hackathon/login';
  }
}, [isLoading, isAuthenticated]);

// In login.tsx  
useEffect(() => {
  if (!isLoading && isAuthenticated) {
    window.location.href = '/ai-native-hackathon/dashboard';
  }
}, [isLoading, isAuthenticated]);
```

**Alternatives Considered**:
- React Router redirect: Not available in Docusaurus
- Next.js-style middleware: Not available in Docusaurus

### 3. CSS Module vs Inline Styles

**Decision**: Use CSS Module (`dashboard.module.css`) matching profile page pattern

**Rationale**: 
- Consistent with existing `profile.module.css`
- Scoped styles prevent conflicts
- Can reference CSS variables from `custom.css`

**Alternatives Considered**:
- Styled-components: Would add new dependency
- Inline styles: Current login.tsx uses this, but harder to maintain

### 4. Card Layout Pattern

**Decision**: CSS Grid for card container with glass-morphism styling

**Rationale**:
- Grid provides responsive layout easily
- Glass effect matches landing page theme
- Same CSS variables already defined (`--color-bg-secondary`, `--color-accent`, etc.)

**Implementation Pattern**:
```css
.dashboardGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.card {
  background: rgba(18, 18, 26, 0.8);
  backdrop-filter: blur(12px);
  border: 1px solid var(--color-border);
  border-radius: 16px;
  padding: 1.5rem;
}
```

### 5. Module Data (Learning Progress)

**Decision**: Hardcode module list from existing docs structure

**Rationale**: 
- No backend tracking per spec clarification
- Can extract from docs folder or hardcode
- Simple static data sufficient for MVP

**Module List** (from `/docs/physical-ai/`):
1. Introduction (intro.md)
2. Hardware Setup (hardware-setup.md)
3. Module 1: ROS 2 (module1-ros2.md)
4. Module 2: Simulation (module2-simulation.md)
5. Module 3: NVIDIA Isaac (module3-isaac.md)
6. Module 4: VLA (module4-vla.md)

**Alternatives Considered**:
- Fetch from sitemap: Adds complexity
- Backend tracking: Out of scope per clarification

### 6. Chatbot Integration

**Decision**: Import and trigger existing chatbot component

**Rationale**: Chatbot already exists in codebase, just need to trigger open

**Implementation**: 
- Check for existing chatbot toggle mechanism
- If none exists, can dispatch custom event or use context

## Technical Findings

### Existing CSS Variables (from custom.css)
```css
--color-bg-primary: #0a0a0f;
--color-bg-secondary: #12121a;
--color-bg-elevated: #1a1a2e;
--color-accent: #00D4FF;
--color-accent-glow: rgba(0, 212, 255, 0.3);
--color-border: #2a2a3e;
--color-text-primary: #FFFFFF;
--color-text-secondary: #E0E0E0;
--color-text-muted: #8892A0;
```

### AuthContext Available Data
```tsx
interface User {
  user_id: string;
  username: string;
  email: string;
  software_background: string | null;
  hardware_background: string | null;
  programming_languages: string[];
  operating_system: string | null;
  learning_goals: string[];
  preferred_explanation_style: string | null;
  prior_knowledge: string[];
  industry: string | null;
}

// Available methods
signOut: () => void;
```

## Resolved Questions

| Question | Resolution |
|----------|------------|
| How to create new page route? | `src/pages/dashboard.tsx` → `/dashboard` |
| How to handle auth redirect? | Client-side useEffect + window.location |
| How to style cards? | CSS Module with glass-morphism pattern |
| What module data to show? | Hardcoded list of 6 modules |
| How to open chatbot? | Check existing implementation, trigger open |

## No Data Model Needed

This feature is frontend-only using existing User data from AuthContext. No new entities or API contracts required.
