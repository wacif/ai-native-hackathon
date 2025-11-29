# API Contract: RAG Chatbot Service

## Base Path: `/chatbot`

### 1. Query Chatbot

-   **Endpoint**: `POST /query`
-   **Description**: Submits a general query to the RAG chatbot about the book content.
-   **Authentication**: Optional (for logging user interactions).
-   **Request Body** (JSON):
    ```json
    {
        "user_id": "uuid", // Optional, if authenticated
        "query_text": "string"
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "response": "string",
        "source_chapters": ["string"], // List of chapter titles/IDs that provided context
        "confidence_score": "float"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Missing query text.
    -   `500 Internal Server Error`: Chatbot processing error.

### 2. Query Chatbot with Context

-   **Endpoint**: `POST /query-context`
-   **Description**: Submits a query to the RAG chatbot with specific user-selected text as context.
-   **Authentication**: Optional (for logging user interactions).
-   **Request Body** (JSON):
    ```json
    {
        "user_id": "uuid", // Optional, if authenticated
        "query_text": "string",
        "selected_context": "string",
        "chapter_id": "uuid" // Optional, for more precise context retrieval
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "response": "string",
        "source_text": "string", // The exact context used
        "confidence_score": "float"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Missing query text or selected context.
    -   `500 Internal Server Error`: Chatbot processing error.
