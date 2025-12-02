# Feature Specification: Urdu Translation

**Feature Branch**: `005-urdu-translation`  
**Created**: 2025-12-01  
**Status**: Complete ✅  
**Input**: User description: "Allow logged-in users to read chapter content in Urdu by clicking a language toggle button, with RTL text support"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Switch to Urdu (Priority: P0)

As a logged-in student who prefers Urdu, I want to click the "اردو" button to read chapters in Urdu so that I can learn in my native language.

**Why this priority**: Core translation functionality.

**Independent Test**: Login, navigate to intro chapter, click "اردو" button, verify content displays in Urdu with RTL layout.

**Acceptance Scenarios**:

1. **Given** I am logged in on a chapter with Urdu translation, **When** I click "اردو", **Then** the content displays in Urdu.
2. **Given** I switched to Urdu, **When** I view the page, **Then** text is right-to-left aligned.
3. **Given** I am viewing Urdu content, **When** I view code blocks, **Then** code remains in English (technical content).

---

### User Story 2 - Switch Back to English (Priority: P0)

As a user viewing Urdu content, I want to click "English" to switch back so that I can see the original content.

**Why this priority**: Users need to toggle between languages.

**Independent Test**: Switch to Urdu, click "English", verify original content restored with LTR layout.

**Acceptance Scenarios**:

1. **Given** I am viewing Urdu content, **When** I click "English", **Then** the original English content is displayed.
2. **Given** I switched back to English, **When** I view the page, **Then** text is left-to-right aligned.

---

### User Story 3 - Translation Not Available (Priority: P1)

As a user on a chapter without Urdu translation, I want to see a clear message so that I understand why translation isn't working.

**Why this priority**: Graceful degradation for incomplete translations.

**Independent Test**: Navigate to a chapter without Urdu translation file, click "اردو", verify error message displays.

**Acceptance Scenarios**:

1. **Given** I am on a chapter without Urdu translation, **When** I click "اردو", **Then** I see "اردو ترجمہ دستیاب نہیں ہے (Urdu translation not available)".
2. **Given** translation failed, **When** I view the page, **Then** original English content remains displayed.

---

### User Story 4 - Authentication Required (Priority: P1)

As an unauthenticated user, I want to see that translation requires login so that I understand how to access it.

**Why this priority**: Clear guidance for unauthenticated users.

**Independent Test**: View chapter without login, verify translation button is not visible or shows login hint.

**Acceptance Scenarios**:

1. **Given** I am not logged in, **When** I view a chapter, **Then** I see "Sign in to access translation & personalization" hint.
2. **Given** I am not logged in, **When** I view the language toggle, **Then** it is either hidden or disabled.

---

### Edge Cases

- What happens if translation file is malformed? → Show error, keep English content.
- What happens if user switches chapters while in Urdu mode? → Reset to English (per-chapter state).
- What happens with mixed content (English + code)? → Code blocks stay in English, prose is translated.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST show language toggle (🌐 English/اردو) on doc pages for authenticated users.
- **FR-002**: System MUST hide toggle for unauthenticated users (show login hint instead).
- **FR-003**: System MUST fetch Urdu translations from `/static/translations/ur/{chapter_id}.md`.
- **FR-004**: System MUST apply RTL (right-to-left) styling when Urdu is active.
- **FR-005**: System MUST preserve English content in code blocks.
- **FR-006**: System MUST allow switching back to English.
- **FR-007**: System MUST show error message if translation file not found.
- **FR-008**: System MUST reset language state when navigating to new chapter.

### Content Requirements

- **CR-001**: Urdu translation for `intro.md` ✅
- **CR-002**: Urdu translation for `hardware-setup.md` ✅
- **CR-003**: Urdu translation for `module1-ros2.md` ✅
- **CR-004**: Urdu translation for `module2-simulation.md` ✅
- **CR-005**: Urdu translation for `module3-isaac.md` ✅
- **CR-006**: Urdu translation for `module4-vla.md` ✅
- **CR-007**: Urdu translation for `weekly-schedule.md` ✅

**Note**: All translations completed on 2025-12-02. Legacy files (chapter1.md, chapter2.md, chapter3.md) retained for backward compatibility.

### Technical Requirements

- **TR-001**: Translations stored as static markdown files (no backend needed).
- **TR-002**: Frontend fetches from `{baseUrl}/translations/ur/{chapterId}.md`.
- **TR-003**: Markdown-to-HTML conversion happens client-side.
- **TR-004**: RTL styling via `dir="rtl"` and `text-align: right`.

### Key Entities

- **Translation File**: Static markdown file at `/static/translations/ur/{chapter_id}.md`

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Translation loads in < 2 seconds
- **SC-002**: RTL layout renders correctly
- **SC-003**: Code blocks remain readable (English, LTR)
- **SC-004**: 100% of chapters have Urdu translations available
- **SC-005**: Error handling works for missing translations

## Implementation Status

| Requirement | Status | Notes |
|-------------|--------|-------|
| FR-001 | ✅ Complete | Toggle button in DocItem/Content |
| FR-002 | ✅ Complete | Conditional rendering |
| FR-003 | ✅ Complete | Fetch from static folder |
| FR-004 | ✅ Complete | RTL styling applied |
| FR-005 | ✅ Complete | Code blocks preserved |
| FR-006 | ✅ Complete | "English" button works |
| FR-007 | ✅ Complete | Error message shown |
| FR-008 | ✅ Complete | State resets on navigation |
| CR-001 to CR-007 | ✅ Complete | All 7 chapters translated |

**Feature Status**: ✅ COMPLETE - Frontend and translations complete

### Remaining Work

All translations completed on 2025-12-02:
- ✅ **TRANS-001**: Created Urdu translation for `hardware-setup.md`
- ✅ **TRANS-002**: Created Urdu translation for `module1-ros2.md`
- ✅ **TRANS-003**: Created Urdu translation for `module2-simulation.md`
- ✅ **TRANS-004**: Created Urdu translation for `module3-isaac.md`
- ✅ **TRANS-005**: Created Urdu translation for `module4-vla.md`
- ✅ **TRANS-006**: Created Urdu translation for `weekly-schedule.md`
- ⚠️ **FIX-001**: Legacy chapter1.md, chapter2.md, chapter3.md retained for backward compatibility
- ⏳ **TEST-001**: End-to-end testing of all translations (pending)
