# API Contract: Content Personalization Service

## Base Path: `/content`

### 1. Personalize Chapter Content

-   **Endpoint**: `POST /{chapter_id}/personalize`
-   **Description**: Returns a personalized version of a specific chapter based on the authenticated user's background.
-   **Authentication**: Requires valid access token in `Authorization` header.
-   **Path Parameters**:
    -   `chapter_id` (UUID): The ID of the chapter to personalize.
-   **Request Body**: None (personalization rules derived from user's stored profile).
-   **Response** (200 OK, JSON):
    ```json
    {
        "chapter_id": "uuid",
        "personalized_content": "string", // Markdown or HTML of personalized content
        "language": "string", // e.g., "en"
        "personalization_applied": "boolean"
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Missing or invalid authentication token.
    -   `404 Not Found`: Chapter not found.
    -   `500 Internal Server Error`: Personalization processing error.
