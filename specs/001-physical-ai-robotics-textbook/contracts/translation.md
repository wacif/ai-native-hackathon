# API Contract: Translation Service

## Base Path: `/content`

### 1. Translate Chapter Content to Urdu

-   **Endpoint**: `POST /{chapter_id}/translate/urdu`
-   **Description**: Returns the Urdu translation of a specific chapter.
-   **Authentication**: Requires valid access token in `Authorization` header.
-   **Path Parameters**:
    -   `chapter_id` (UUID): The ID of the chapter to translate.
-   **Request Body**: None.
-   **Response** (200 OK, JSON):
    ```json
    {
        "chapter_id": "uuid",
        "translated_content": "string", // Markdown or HTML of Urdu content
        "language": "ur"
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Missing or invalid authentication token.
    -   `404 Not Found`: Chapter not found.
    -   `500 Internal Server Error`: Translation service error.
