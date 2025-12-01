# API Contract: Content Personalization Service

## Status: ⚠️ **NOT YET IMPLEMENTED**

## Planned Technology Stack
- **AI/LLM**: Google Gemini or similar for content adaptation
- **Backend**: FastAPI
- **Database**: PostgreSQL (user preferences)
- **Dependencies**: Authentication service (prerequisite)

## Database Support: ✅ **READY**

The User model includes fields for personalization:

```python
class User(Base):
    software_background: str       # User's software experience
    hardware_background: str       # User's hardware experience
    personalization_preferences: JSONB  # Flexible JSON for future preferences
    selected_language: str         # Currently selected language
```

## Base Path: `/content` (Planned)

---

## Planned Endpoints

### 1. Personalize Chapter Content

-   **Endpoint**: `POST /content/{chapter_id}/personalize`
-   **Description**: Returns a personalized version of a specific chapter based on the authenticated user's background and preferences.
-   **Authentication**: Required - Bearer token in `Authorization` header.
-   **Path Parameters**:
    -   `chapter_id` (string): The ID of the chapter to personalize (e.g., "intro", "chapter1", "chapter2").
-   **Request Headers**:
    ```
    Authorization: Bearer <access_token>
    ```
-   **Request Body**: None (personalization rules derived from user's stored profile).
-   **Response** (200 OK, JSON):
    ```json
    {
        "chapter_id": "string",
        "original_content": "string",          // Original Markdown content
        "personalized_content": "string",      // Adapted Markdown content
        "language": "string",                  // e.g., "en"
        "personalization_applied": true,
        "adaptations": {                       // What was personalized
            "difficulty_level": "beginner",    // Adjusted to user's background
            "examples_added": ["software"],    // Based on software_background
            "analogies": ["hardware"]          // Based on hardware_background
        }
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Missing or invalid authentication token.
    -   `404 Not Found`: Chapter not found.
    -   `500 Internal Server Error`: Personalization processing error.

### 2. Get Personalization Preferences

-   **Endpoint**: `GET /content/preferences`
-   **Description**: Retrieves the user's current personalization preferences.
-   **Authentication**: Required - Bearer token in `Authorization` header.
-   **Response** (200 OK, JSON):
    ```json
    {
        "software_background": "string",       // e.g., "beginner", "intermediate", "advanced"
        "hardware_background": "string",       // e.g., "beginner", "intermediate", "advanced"
        "preferred_examples": ["string"],      // e.g., ["python", "robotics"]
        "learning_style": "string",            // e.g., "visual", "practical", "theoretical"
        "personalization_preferences": {}      // Additional custom preferences
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Missing or invalid authentication token.

### 3. Update Personalization Preferences

-   **Endpoint**: `PATCH /content/preferences`
-   **Description**: Updates the user's personalization preferences.
-   **Authentication**: Required - Bearer token in `Authorization` header.
-   **Request Body** (JSON):
    ```json
    {
        "software_background": "string",       // Optional
        "hardware_background": "string",       // Optional
        "preferred_examples": ["string"],      // Optional
        "learning_style": "string",            // Optional
        "personalization_preferences": {}      // Optional: Custom preferences
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "message": "Preferences updated successfully",
        "preferences": { /* Updated preferences object */ }
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Invalid preference values.
    -   `401 Unauthorized`: Missing or invalid authentication token.

---

## Personalization Strategy

### Content Adaptation Based on Background

1. **Software Background**:
   - **Beginner**: Add more explanations for programming concepts, include Python basics
   - **Intermediate**: Focus on ROS 2 and framework-specific details
   - **Advanced**: Include advanced optimization techniques, low-level details

2. **Hardware Background**:
   - **Beginner**: Explain sensors, actuators with basic analogies
   - **Intermediate**: Include circuit diagrams, specifications
   - **Advanced**: Deep dive into hardware integration, custom PCB design

3. **Combined Personalization**:
   - Users strong in software but weak in hardware get more hardware context
   - Users strong in hardware but weak in software get more programming examples
   - Balanced users get comprehensive coverage

### Example Personalization Flow

```
1. User logs in → System retrieves background preferences
2. User navigates to chapter → Frontend requests personalized version
3. Backend:
   a. Fetches original chapter content from docs
   b. Analyzes user's software_background and hardware_background
   c. Uses LLM (Gemini) to adapt content:
      - Adjust difficulty level
      - Add relevant examples
      - Include helpful analogies
      - Expand or condense sections
   d. Returns personalized Markdown
4. Frontend renders adapted content
```

---

## Implementation Status

### ✅ Completed
- Database fields in User model
- Basic user background fields

### ⏳ To Be Implemented
- FastAPI router for personalization endpoints
- LLM integration for content adaptation
- Content caching (personalized versions)
- A/B testing framework for personalization effectiveness
- Analytics to track which personalizations help learning
- Frontend UI for setting preferences
- Real-time personalization toggle

---

## Dependencies

### Blocked By
1. **Authentication Service**: Must be implemented first (user identification required)
2. **User Profiles**: Users need to have background information stored

### Integrates With
- **Chatbot**: Could provide personalized answers based on user background
- **Translation**: Personalized content can be translated
- **Analytics**: Track effectiveness of personalization

---

## Future Enhancements

1. **Dynamic Difficulty Adjustment**: Adapt difficulty as user progresses through chapters
2. **Learning Path Recommendations**: Suggest chapters based on user's knowledge gaps
3. **Interactive Examples**: Generate personalized code examples in user's preferred language
4. **Progress-Based Personalization**: Adapt content based on quiz results and interaction history
5. **Peer Comparison**: Show how user's background compares to others taking the course

---

## Notes

- Current implementation has no personalization - all users see identical content
- The chatbot doesn't consider user background when answering questions
- Database schema is ready, but no API endpoints exist
- Frontend would need significant updates to support personalized content rendering
