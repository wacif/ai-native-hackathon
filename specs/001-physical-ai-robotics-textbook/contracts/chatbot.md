# API Contract: RAG Chatbot Service

## Status: ✅ **IMPLEMENTED**

## Technology Stack
- **Backend**: FastAPI
- **LLM**: Google Gemini 2.5 Flash Lite (via OpenAI-compatible API)
- **Agent Framework**: OpenAI Agents SDK
- **Vector Database**: Qdrant Cloud
- **Embeddings**: BAAI/bge-small-en-v1.5 (via FastEmbed)
- **Frontend**: React + TypeScript (Docusaurus component)

## Base URL: `http://localhost:8000` (Development) | `https://your-backend-url.com` (Production)

---

## Endpoints

### 1. General Query

-   **Endpoint**: `POST /query`
-   **Description**: Submits a general query to the RAG chatbot about the book content. Uses Gemini agent with vector search tool to retrieve relevant content from Qdrant.
-   **Authentication**: None (Optional user_id field for future logging).
-   **Request Body** (JSON):
    ```json
    {
        "question": "string",           // Required: User's question
        "user_id": "string",            // Optional: For logging interactions
        "page_url": "string",           // Optional: Current page URL for context
        "chapter_id": "string",         // Optional: Current chapter (e.g., "intro", "chapter1")
        "max_results": 5                // Optional: Max vector search results (default: 5)
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "answer": "string",             // AI-generated answer from Gemini agent
        "sources": [                    // Source metadata
            {
                "source": "agent_search",
                "type": "rag",
                "score": 1.0
            }
        ],
        "confidence_score": 0.95,       // Fixed confidence score
        "page_context": "string"        // Echo of page_url if provided
    }
    ```
-   **Error Responses**:
    -   `500 Internal Server Error`: Gemini API key not configured or chatbot processing error.

### 2. Query with Selected Text Context

-   **Endpoint**: `POST /query-selection`
-   **Description**: Submits a query to the RAG chatbot with specific user-selected text as context. This provides more focused answers based on highlighted text.
-   **Authentication**: None (Optional user_id field for future logging).
-   **Request Body** (JSON):
    ```json
    {
        "question": "string",           // Required: User's question about the selected text
        "selected_text": "string",      // Required: Text highlighted by user
        "user_id": "string",            // Optional: For logging interactions
        "page_url": "string",           // Optional: Current page URL
        "chapter_id": "string"          // Optional: Current chapter (e.g., "intro", "chapter1")
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "answer": "string",             // AI-generated answer focused on selected text
        "sources": [                    // Source metadata
            {
                "source": "selected_text",
                "type": "selection",
                "score": 1.0,
                "text_preview": "string"  // First 100 chars of selected text
            }
        ],
        "confidence_score": 0.98,       // Higher confidence for direct text selection
        "page_context": "string"        // Echo of page_url if provided
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Missing question or selected text.
    -   `500 Internal Server Error`: Gemini API key not configured or processing error.

### 3. Health Check

-   **Endpoint**: `GET /health`
-   **Description**: Checks the health status of the API and its dependencies.
-   **Response** (200 OK, JSON):
    ```json
    {
        "status": "healthy",
        "qdrant": "connected",          // Qdrant connection status
        "collection_exists": true,      // Whether "book_content" collection exists
        "gemini_configured": true,      // Whether Gemini API key is set
        "agent": "gemini-2.5-flash-lite" // LLM model being used
    }
    ```

### 4. List Collections

-   **Endpoint**: `GET /collections`
-   **Description**: Lists all available Qdrant collections.
-   **Response** (200 OK, JSON):
    ```json
    {
        "collections": ["book_content", ...]  // List of collection names
    }
    ```
-   **Error Responses**:
    -   `500 Internal Server Error`: Failed to fetch collections from Qdrant.

---

## Agent Capabilities

The RAG agent (`Book Assistant`) has the following capabilities:

1. **Vector Search Tool**: `search_book_content(query, chapter_id, limit)`
   - Searches the book content using semantic similarity
   - Can filter by chapter_id for context-aware responses
   - Returns top-k most relevant chunks with metadata

2. **Context Awareness**:
   - Knows which chapter the user is currently viewing
   - Can answer "What chapter am I on?" type questions
   - Prioritizes content from the current chapter when chapter_id is provided

3. **Intelligent Routing**:
   - For location questions: Answers directly from context without search
   - For technical questions: Uses vector search tool automatically
   - For selected text: Focuses on the provided context

---

## Frontend Integration

The chatbot is integrated as a floating widget in the Docusaurus theme:

- **Component**: `src/components/Chatbot/index.tsx`
- **Features**:
  - Floating chat button (bottom right)
  - Text selection detection
  - Chapter-aware context (automatically detects current chapter)
  - Markdown rendering for bot responses
  - Auto-scroll to latest message
  - Loading states and error handling

---

## Data Model

### Qdrant Collection: `book_content`

**Vector Dimension**: 384 (BAAI/bge-small-en-v1.5)

**Payload Schema**:
```json
{
    "text": "string",           // Content chunk
    "source": "string",         // Source file name
    "chapter_id": "string",     // Chapter identifier (e.g., "intro", "chapter1")
    "module_id": "string",      // Module identifier
    "page_url": "string"        // URL path to the page
}
```

---

## Notes

- **Authentication**: Not yet implemented. The `user_id` field is accepted but not used for authentication.
- **Interaction Logging**: Database models exist (`ChatbotInteraction`) but logging is not yet implemented.
- **Rate Limiting**: Not implemented.
- **Caching**: Not implemented.
- **Chapter Mapping**: Hardcoded in agent.py (intro, chapter1, chapter2, chapter3)
