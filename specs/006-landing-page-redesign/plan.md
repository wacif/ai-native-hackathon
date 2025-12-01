# Implementation Plan: Landing Page Redesign & Color Scheme

**Branch**: `006-landing-page-redesign` | **Date**: 2025-12-01 | **Spec**: `specs/006-landing-page-redesign/spec.md`

## Summary

Transform the default Docusaurus landing page into a futuristic, animated experience with dark theme (#0a0a0f) and cyan neon accents (#00D4FF). Implement particle backgrounds, typing effects, hover animations, and apply consistent theming across the entire application.

## Technical Context

**Language/Version**: TypeScript 5.x, React 18+, CSS3  
**Primary Dependencies**: Docusaurus 3.9.2, React, clsx (existing)  
**New Dependencies**: None required (CSS-only animations preferred)  
**Storage**: N/A  
**Testing**: Visual testing, Lighthouse audits  
**Target Platform**: Web (GitHub Pages)  
**Project Type**: Web frontend (Docusaurus)  
**Performance Goals**: < 3s load time, 60fps animations, Lighthouse > 80  
**Constraints**: Must work within Docusaurus theming system, no heavy JS animation libraries  
**Scale/Scope**: 1 landing page, ~10 pages theme update

## Constitution Check

| Principle | Status | Notes |
|-----------|--------|-------|
| Spec-Driven Development | ✅ Pass | Spec created at `specs/006-landing-page-redesign/spec.md` |
| Right Altitude Balance | ✅ Pass | Concrete color values, animation specs defined |
| Accessibility | ✅ Pass | `prefers-reduced-motion` required, WCAG AA contrast |
| Performance | ✅ Pass | CSS-first animations, no heavy libraries |

## Project Structure

### Documentation (this feature)

```text
specs/006-landing-page-redesign/
├── spec.md              # Feature specification ✅
├── plan.md              # This file
└── tasks.md             # To be created via /sp.tasks
```

### Source Code Changes

```text
src/
├── css/
│   └── custom.css              # UPDATE: Global theme variables
├── pages/
│   ├── index.tsx               # UPDATE: New landing page structure
│   └── index.module.css        # UPDATE: Landing page styles
└── components/
    └── Landing/                # NEW: Landing page components
        ├── HeroSection/
        │   ├── index.tsx
        │   └── styles.module.css
        ├── FeatureCard/
        │   ├── index.tsx
        │   └── styles.module.css
        ├── ParticleBackground/
        │   ├── index.tsx
        │   └── styles.module.css
        └── AnimatedText/
            ├── index.tsx
            └── styles.module.css
```

**Structure Decision**: Extend existing Docusaurus structure with new Landing components folder. Keep existing HomepageFeatures for reference but replace with new components.

---

## Phase 1: Global Theme Setup

### 1.1 Color Variables Update

**File**: `src/css/custom.css`

```css
/* New color palette */
:root {
  /* Override for light mode - force dark aesthetic */
  --ifm-color-primary: #00D4FF;
  --ifm-color-primary-dark: #00B8E6;
  --ifm-color-primary-darker: #00A3CC;
  --ifm-color-primary-darkest: #007A99;
  --ifm-color-primary-light: #33DDFF;
  --ifm-color-primary-lighter: #66E5FF;
  --ifm-color-primary-lightest: #99EEFF;
  
  /* Background colors */
  --ifm-background-color: #0a0a0f;
  --ifm-background-surface-color: #12121a;
  
  /* Text colors */
  --ifm-font-color-base: #E0E0E0;
  --ifm-heading-color: #FFFFFF;
  
  /* Code */
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 212, 255, 0.1);
  
  /* Custom variables */
  --color-accent-glow: rgba(0, 212, 255, 0.3);
  --color-border: #2a2a3e;
  --color-surface-elevated: #1a1a2e;
}

[data-theme='dark'] {
  --ifm-color-primary: #00D4FF;
  --ifm-background-color: #0a0a0f;
  --ifm-background-surface-color: #12121a;
}
```

### 1.2 Force Dark Mode Default

**File**: `docusaurus.config.ts`

Update colorMode config to default to dark:

```typescript
colorMode: {
  defaultMode: 'dark',
  disableSwitch: true,  // Optional: force dark only
  respectPrefersColorScheme: false,
}
```

---

## Phase 2: Landing Page Components

### 2.1 ParticleBackground Component

**Purpose**: Animated floating particles/dots in background

**Implementation**: CSS-only with pseudo-elements and keyframe animations

```tsx
// Key features:
// - 50-80 particles using CSS ::before/::after or generated divs
// - Random positions, sizes (2-6px), opacity (0.3-0.8)
// - Slow floating animation (3-8s duration)
// - Respects prefers-reduced-motion
```

### 2.2 AnimatedText Component

**Purpose**: Typing effect for hero headline

**Props**:
```typescript
interface AnimatedTextProps {
  text: string;
  speed?: number;  // ms per character, default 80
  className?: string;
  onComplete?: () => void;
}
```

**Implementation**: 
- useState for displayed text
- useEffect with setInterval
- CSS cursor blink animation

### 2.3 HeroSection Component

**Purpose**: Main banner with headline, subheadline, CTA

**Structure**:
```tsx
<section className={styles.hero}>
  <ParticleBackground />
  <div className={styles.content}>
    <AnimatedText text="Physical AI & Humanoid Robotics" />
    <p className={styles.subtitle}>
      CoLearning Embodied Intelligence with ROS 2 and NVIDIA Isaac
    </p>
    <Link className={styles.ctaButton} to="/docs/physical-ai/intro">
      Get Started →
    </Link>
  </div>
</section>
```

**Styles**:
- Full viewport height (100vh) or min-height
- Centered content with flexbox
- CTA button with glow effect on hover
- Fade-in animation on load

### 2.4 FeatureCard Component

**Purpose**: Individual feature highlight

**Props**:
```typescript
interface FeatureCardProps {
  icon: string;  // Emoji or icon component
  title: string;
  description: string;
  delay?: number;  // Animation delay for stagger effect
}
```

**Styles**:
- Dark card background (#12121a)
- Cyan border on hover with glow
- Scale/lift transform on hover
- Fade-in-up animation on scroll

### 2.5 FeaturesSection Component

**Purpose**: Grid of 4 feature cards

**Features**:
1. 🤖 ROS 2 Fundamentals
2. 🎮 Physics Simulation
3. 🧠 NVIDIA Isaac
4. 🚀 Vision-Language-Action

**Layout**: 
- Desktop: 2x2 grid or 4-column row
- Mobile: Single column stack

---

## Phase 3: Animation System

### 3.1 Scroll Animations

**Implementation**: Intersection Observer API

```tsx
// useScrollAnimation hook
const useScrollAnimation = (threshold = 0.1) => {
  const ref = useRef(null);
  const [isVisible, setIsVisible] = useState(false);
  
  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => setIsVisible(entry.isIntersecting),
      { threshold }
    );
    if (ref.current) observer.observe(ref.current);
    return () => observer.disconnect();
  }, [threshold]);
  
  return { ref, isVisible };
};
```

### 3.2 CSS Keyframes

```css
@keyframes fadeInUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

@keyframes float {
  0%, 100% { transform: translateY(0); }
  50% { transform: translateY(-10px); }
}

@keyframes glow {
  0%, 100% { box-shadow: 0 0 5px var(--color-accent-glow); }
  50% { box-shadow: 0 0 20px var(--color-accent-glow); }
}

@keyframes blink {
  0%, 50% { opacity: 1; }
  51%, 100% { opacity: 0; }
}
```

### 3.3 Reduced Motion Support

```css
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

---

## Phase 4: App-Wide Theme Application

### 4.1 Navbar Styling

**Updates to custom.css**:
- Dark background (#0a0a0f)
- Cyan accent for active links
- Subtle bottom border glow

### 4.2 Sidebar Styling

**Updates**:
- Dark background matching navbar
- Cyan highlight for active item
- Hover effects on items

### 4.3 Doc Pages Styling

**Updates**:
- Dark background for content area
- Cyan accent for links
- Code blocks with dark theme
- Heading styles with subtle glow

### 4.4 Footer Styling

**Updates**:
- Dark background
- Muted text colors
- Cyan links on hover

---

## Phase 5: Responsive Design

### Breakpoints

```css
/* Mobile first approach */
/* Base: 320px+ (mobile) */
/* sm: 576px+ */
/* md: 768px+ (tablet) */
/* lg: 996px+ (Docusaurus default) */
/* xl: 1200px+ */
/* xxl: 1440px+ */
```

### Key Responsive Adjustments

1. **Hero Section**:
   - Mobile: Smaller heading, reduced particle count
   - Tablet: Medium heading
   - Desktop: Full effect

2. **Feature Cards**:
   - Mobile: Single column, full width
   - Tablet: 2-column grid
   - Desktop: 4-column or 2x2 grid

3. **Animations**:
   - Mobile: Reduced/simplified animations for performance

---

## Implementation Order

| Phase | Priority | Description | Dependencies |
|-------|----------|-------------|--------------|
| 1.1 | P0 | Color variables in custom.css | None |
| 1.2 | P0 | Force dark mode in config | Phase 1.1 |
| 2.1 | P1 | ParticleBackground component | Phase 1 |
| 2.2 | P1 | AnimatedText component | None |
| 2.3 | P1 | HeroSection component | 2.1, 2.2 |
| 2.4 | P1 | FeatureCard component | Phase 1 |
| 2.5 | P1 | FeaturesSection component | 2.4 |
| 3.1 | P1 | Scroll animation hook | None |
| 3.2 | P1 | CSS keyframes | Phase 1 |
| 3.3 | P1 | Reduced motion support | Phase 3.2 |
| 4.1 | P2 | Navbar styling | Phase 1 |
| 4.2 | P2 | Sidebar styling | Phase 1 |
| 4.3 | P2 | Doc pages styling | Phase 1 |
| 4.4 | P2 | Footer styling | Phase 1 |
| 5 | P2 | Responsive adjustments | All above |

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Animation performance on low-end devices | Medium | CSS-only animations, reduced motion support |
| Dark theme readability issues | Medium | Ensure WCAG AA contrast ratios |
| Docusaurus theme conflicts | Low | Use CSS specificity carefully, test all pages |
| Particle background performance | Low | Limit particle count, use CSS transforms |

---

## Success Validation

- [ ] Landing page loads in < 3 seconds
- [ ] Lighthouse Performance > 80
- [ ] Lighthouse Accessibility > 90
- [ ] All animations run at 60fps
- [ ] Theme consistent across all pages
- [ ] Mobile responsive at all breakpoints
- [ ] Reduced motion preference respected
- [ ] WCAG AA contrast met

---

## Files to Create/Modify

### New Files
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

### Modified Files
1. `src/css/custom.css` - Global theme variables
2. `src/pages/index.tsx` - New landing page structure
3. `src/pages/index.module.css` - Landing page specific styles
4. `docusaurus.config.ts` - Dark mode default
