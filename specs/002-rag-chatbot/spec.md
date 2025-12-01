# Feature Specification: RAG Chatbot

**Feature Branch**: `002-rag-chatbot`  
**Created**: 2025-12-01  
**Status**: Complete ✅  
**Input**: User description: "Integrate a RAG-powered chatbot that can answer questions about the textbook content, including questions about user-selected text"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask General Questions (Priority: P0)

As a student reading the textbook, I want to ask the chatbot questions about any topic in the book so that I can get immediate clarifications without searching manually.

**Why this priority**: This is the core RAG functionality - answering questions from the knowledge base.

**Independent Test**: Open chatbot, ask "What is ROS 2?", verify response is accurate and sourced from book content.

**Acceptance Scenarios**:

1. **Given** I am on any doc page, **When** I click the chatbot icon, **Then** a chat interface opens.
2. **Given** the chat is open, **When** I type "What are ROS 2 nodes?" and submit, **Then** I receive an accurate answer within 5 seconds.
3. **Given** I ask about content not in the book, **When** the chatbot responds, **Then** it indicates it cannot answer from available content.

---

### User Story 2 - Ask About Selected Text (Priority: P0)

As a student reading a specific passage, I want to select text and ask the chatbot about it so that I get context-specific explanations.

**Why this priority**: This enables deeper learning by allowing questions about specific content the user is reading.

**Independent Test**: Select a paragraph, click "Ask about selection", type a question, verify response uses selected text as context.

**Acceptance Scenarios**:

1. **Given** I select text on a page, **When** I open the chatbot, **Then** I see an option to ask about my selection.
2. **Given** I have selected text about URDF, **When** I ask "Explain this in simpler terms", **Then** the response specifically addresses the selected text.
3. **Given** I selected text, **When** I receive a response, **Then** the source shows "selected_text" type.

---

### User Story 3 - Personalized Responses (Priority: P1)

As a logged-in student, I want the chatbot to consider my profile when answering so that explanations match my background and learning style.

**Why this priority**: Personalization enhances learning experience but requires authentication to be implemented first.

**Independent Test**: Login with a profile (e.g., "Python developer, prefer concise"), ask a question, verify response style matches preferences.

**Acceptance Scenarios**:

1. **Given** I am logged in with "prefer concise" style, **When** I ask a question, **Then** the response is brief and to-the-point.
2. **Given** I am logged in with "Python" as preferred language, **When** I ask about code, **Then** examples are in Python.
3. **Given** I am not logged in, **When** I ask a question, **Then** I receive a generic (but accurate) response.

---

### Edge Cases

- What happens if Qdrant is unreachable? → Return error message: "Unable to search content. Please try again."
- What happens if Gemini API fails? → Return error message: "Unable to generate response. Please try again."
- What happens with very long questions? → Truncate at 2000 characters with validation error.
- What happens with empty questions? → Validation error: "Question must be at least 2 characters."

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a chatbot UI component on all doc pages.
- **FR-002**: System MUST accept text questions up to 2000 characters.
- **FR-003**: System MUST search the Qdrant vector store for relevant content.
- **FR-004**: System MUST use Gemini LLM to generate answers based on retrieved content.
- **FR-005**: System MUST support text selection context for questions.
- **FR-006**: System MUST return responses within 5 seconds.
- **FR-007**: System SHOULD personalize responses based on user profile (if authenticated).
- **FR-008**: System MUST indicate when it cannot answer from available content.

### Technical Requirements

- **TR-001**: Backend endpoint: `POST /query` for general questions.
- **TR-002**: Backend endpoint: `POST /query-selection` for text selection questions.
- **TR-003**: Vector embeddings using FastEmbed (BAAI/bge-small-en-v1.5).
- **TR-004**: Vector store: Qdrant Cloud with `query_points()` API (v1.16+).
- **TR-005**: LLM: Google Gemini 2.5 Flash via OpenAI SDK compatibility layer.

### Key Entities

- **ChatbotInteraction**: User question, context (if any), response, timestamp, user_id (optional)
- **VectorChunk**: Embedded text chunk from book content, stored in Qdrant

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot responds to 95% of questions within 5 seconds ✅
- **SC-002**: Chatbot answers are relevant to book content 90% of the time ✅
- **SC-003**: Text selection questions use selection as primary context ✅
- **SC-004**: Error messages are user-friendly (no raw stack traces) ✅

## Implementation Status

| Requirement | Status | Notes |
|-------------|--------|-------|
| FR-001 | ✅ Complete | Chatbot component in `src/components/Chatbot/` |
| FR-002 | ✅ Complete | Pydantic validation with 2000 char limit |
| FR-003 | ✅ Complete | Qdrant `query_points()` implementation |
| FR-004 | ✅ Complete | Gemini 2.5 Flash integration |
| FR-005 | ✅ Complete | `/query-selection` endpoint |
| FR-006 | ✅ Complete | Average response ~3 seconds |
| FR-007 | ✅ Complete | UserProfile passed to agent |
| FR-008 | ✅ Complete | Agent indicates when no relevant content found |

**Feature Status**: ✅ COMPLETE
