# Feature Specification: Content Personalization

**Feature Branch**: `004-content-personalization`  
**Created**: 2025-12-01  
**Status**: Partial ⚠️  
**Input**: User description: "Allow logged-in users to personalize chapter content based on their profile preferences (OS, programming languages, learning style) using AI-powered content adaptation"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Personalize Chapter Content (Priority: P0)

As a logged-in student with profile preferences, I want to click "Personalize for me" on any chapter so that the content adapts to my operating system, programming language preferences, and learning style.

**Why this priority**: This is the core value proposition - adaptive learning content.

**Independent Test**: Login with profile (e.g., Windows, Python, concise style), navigate to a chapter, click personalize, verify content adapts (e.g., Windows commands, Python examples, shorter explanations).

**Acceptance Scenarios**:

1. **Given** I am logged in with OS=Windows, **When** I personalize a chapter with bash commands, **Then** commands are converted to PowerShell/CMD equivalents.
2. **Given** I am logged in with preferred_language=Python, **When** I personalize a chapter with code examples, **Then** examples prioritize Python.
3. **Given** I am logged in with style=concise, **When** I personalize a chapter, **Then** explanations are shortened and use bullet points.
4. **Given** I have prior_knowledge=["python", "linux"], **When** I personalize, **Then** basic Python/Linux concepts are condensed.

---

### User Story 2 - View Original Content (Priority: P0)

As a user who has personalized content, I want to revert to the original so that I can see the standard version.

**Why this priority**: Users need ability to compare or undo personalization.

**Independent Test**: Personalize a chapter, click "View Original", verify original content is restored.

**Acceptance Scenarios**:

1. **Given** I have personalized a chapter, **When** I click "View Original", **Then** the original content is displayed.
2. **Given** I reverted to original, **When** I click "Personalize for me" again, **Then** personalized content is regenerated.

---

### User Story 3 - Cached Personalization (Priority: P1)

As a returning user, I want my personalized content to load instantly if I've already personalized that chapter.

**Why this priority**: Caching improves UX and reduces API costs.

**Independent Test**: Personalize a chapter, navigate away, return, click personalize, verify cached version loads quickly.

**Acceptance Scenarios**:

1. **Given** I personalized Chapter X yesterday, **When** I click personalize on Chapter X today, **Then** cached version loads in < 1 second.
2. **Given** the chapter content has changed, **When** I personalize, **Then** a new version is generated (cache invalidated by content hash).
3. **Given** I want fresh personalization, **When** I click with force_refresh=true, **Then** new content is generated.

---

### User Story 4 - No Preferences Set (Priority: P1)

As a logged-in user without profile preferences, I want to see a hint to set up my profile so that I understand why personalization isn't available.

**Why this priority**: Clear guidance prevents user confusion.

**Independent Test**: Login with empty profile, navigate to chapter, verify hint message appears instead of personalize button.

**Acceptance Scenarios**:

1. **Given** I am logged in with no preferences, **When** I view a chapter, **Then** I see "Set up your profile to enable personalized content".
2. **Given** I am not logged in, **When** I view a chapter, **Then** I see "Sign in to access personalization".

---

### Edge Cases

- What happens if Gemini API fails during personalization? → Show error with retry option.
- What happens if personalization takes too long? → Show timeout error after 30 seconds.
- What happens if content is very long? → Chunk and process (100KB limit enforced).
- What happens if user profile changes after caching? → Cache still valid (keyed by content hash, not profile hash) - user can force refresh.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST show "Personalize for me" button on doc pages for authenticated users with preferences.
- **FR-002**: System MUST hide/disable button for unauthenticated users or users without preferences.
- **FR-003**: System MUST send chapter content + user profile to backend for personalization.
- **FR-004**: System MUST use Gemini LLM to adapt content based on:
  - Operating system (adapt commands, file paths)
  - Programming languages (prefer user's languages in examples)
  - Explanation style (concise/detailed/visual/example-driven)
  - Prior knowledge (skip basics user already knows)
  - Learning goals (emphasize relevant content)
  - Industry (relate examples to user's field)
- **FR-005**: System MUST cache personalized content by (user_id, chapter_id, content_hash).
- **FR-006**: System MUST display "View Original" button after personalization.
- **FR-007**: System MUST show loading state during personalization.
- **FR-008**: System MUST show error state with retry option on failure.
- **FR-009**: System MUST show user's preference tags below the button.

### Technical Requirements

- **TR-001**: Backend endpoint: `POST /personalize-chapter` (requires auth).
- **TR-002**: Request body: `{ chapter_id, chapter_content, force_refresh }`.
- **TR-003**: Response: `{ personalized_content, cached, chapter_id }`.
- **TR-004**: Content limit: 100KB per chapter.
- **TR-005**: Personalized content stored in `personalized_content` table.

### Key Entities

- **PersonalizedContent**: id, user_id, chapter_id, original_content_hash, personalized_content, created_at, updated_at

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Personalization completes within 10 seconds for typical chapters
- **SC-002**: Cached content loads in < 1 second
- **SC-003**: Content correctly adapts to user's OS (commands, paths)
- **SC-004**: Content correctly adapts to user's preferred programming language
- **SC-005**: Error rate < 5%

## Implementation Status

| Requirement | Status | Notes |
|-------------|--------|-------|
| FR-001 | ✅ Complete | Button in DocItem/Content wrapper |
| FR-002 | ✅ Complete | Conditional rendering based on auth + preferences |
| FR-003 | ✅ Complete | Frontend sends content to backend |
| FR-004 | ✅ Complete | Gemini prompt includes all profile fields |
| FR-005 | ✅ Complete | Database caching implemented |
| FR-006 | ✅ Complete | "View Original" button works |
| FR-007 | ⚠️ Partial | Loading spinner exists but could be improved |
| FR-008 | ⚠️ Partial | Error shown but no retry button |
| FR-009 | ⚠️ Partial | Tags display but styling needs polish |

**Feature Status**: ⚠️ PARTIAL - Backend complete, frontend needs UX polish

### Remaining Work

1. **UX-001**: Add retry button on error state
2. **UX-002**: Improve loading state animation
3. **UX-003**: Polish preference tags styling
4. **UX-004**: Add success toast notification
5. **TEST-001**: End-to-end testing with various profiles
