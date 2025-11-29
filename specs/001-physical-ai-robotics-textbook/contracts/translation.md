# API Contract: Translation Service

## Status: ⚠️ **NOT YET IMPLEMENTED**

## Planned Technology Stack
- **Translation API**: Google Translate API or Google Gemini with translation prompts
- **Backend**: FastAPI
- **Cache**: Redis or database for translated content caching
- **Dependencies**: Authentication service (optional, for user preferences)

## Database Support: ✅ **READY**

The User model includes a language preference field:

```python
class User(Base):
    selected_language: str  # Default: "en", supports "ur" and future languages
```

## Base Path: `/content` (Planned)

---

## Planned Endpoints

### 1. Translate Chapter to Urdu

-   **Endpoint**: `POST /content/{chapter_id}/translate/urdu`
-   **Description**: Returns the Urdu translation of a specific chapter. Caches translations for performance.
-   **Authentication**: Optional - Bearer token for saving language preference.
-   **Path Parameters**:
    -   `chapter_id` (string): The ID of the chapter to translate (e.g., "intro", "chapter1").
-   **Request Headers** (Optional):
    ```
    Authorization: Bearer <access_token>
    ```
-   **Request Body**: None.
-   **Response** (200 OK, JSON):
    ```json
    {
        "chapter_id": "string",
        "original_language": "en",
        "target_language": "ur",
        "translated_content": "string",      // Markdown content in Urdu
        "translation_cached": true,          // Whether served from cache
        "translation_date": "2024-01-01T00:00:00Z"
    }
    ```
-   **Error Responses**:
    -   `404 Not Found`: Chapter not found.
    -   `500 Internal Server Error`: Translation service error.
    -   `503 Service Unavailable`: Translation API unavailable.

### 2. Translate Chapter to Any Language

-   **Endpoint**: `POST /content/{chapter_id}/translate/{language_code}`
-   **Description**: Returns the translation of a specific chapter to any supported language.
-   **Authentication**: Optional - Bearer token for saving language preference.
-   **Path Parameters**:
    -   `chapter_id` (string): The ID of the chapter to translate.
    -   `language_code` (string): Target language code (ISO 639-1, e.g., "ur", "ar", "es", "fr").
-   **Response** (200 OK, JSON):
    ```json
    {
        "chapter_id": "string",
        "original_language": "en",
        "target_language": "string",         // e.g., "ur"
        "translated_content": "string",      // Markdown content in target language
        "translation_cached": true,
        "translation_date": "2024-01-01T00:00:00Z"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Unsupported language code.
    -   `404 Not Found`: Chapter not found.
    -   `500 Internal Server Error`: Translation service error.

### 3. Get Supported Languages

-   **Endpoint**: `GET /content/translate/languages`
-   **Description**: Returns a list of all supported translation languages.
-   **Response** (200 OK, JSON):
    ```json
    {
        "supported_languages": [
            {
                "code": "ur",
                "name": "Urdu",
                "native_name": "اردو"
            },
            {
                "code": "ar",
                "name": "Arabic",
                "native_name": "العربية"
            },
            {
                "code": "es",
                "name": "Spanish",
                "native_name": "Español"
            }
        ]
    }
    ```

### 4. Translate Chatbot Response

-   **Endpoint**: `POST /content/translate/response`
-   **Description**: Translates a chatbot response to the user's preferred language.
-   **Authentication**: Optional.
-   **Request Body** (JSON):
    ```json
    {
        "text": "string",                    // Text to translate
        "target_language": "string"          // Language code (e.g., "ur")
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "original_text": "string",
        "translated_text": "string",
        "source_language": "en",
        "target_language": "ur"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Missing text or invalid language code.
    -   `500 Internal Server Error`: Translation service error.

### 5. Save Language Preference

-   **Endpoint**: `PATCH /content/translate/preference`
-   **Description**: Saves the user's language preference (requires authentication).
-   **Authentication**: Required - Bearer token in `Authorization` header.
-   **Request Body** (JSON):
    ```json
    {
        "language": "string"                 // Language code (e.g., "ur", "en")
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "message": "Language preference updated",
        "selected_language": "ur"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Invalid language code.
    -   `401 Unauthorized`: Missing or invalid authentication token.

---

## Translation Strategy

### Approach 1: Google Translate API (Recommended for Phase 1)
- Fast, cost-effective
- Good quality for technical content
- Supports 100+ languages
- ~$20 per million characters

### Approach 2: Google Gemini with Translation Prompts (Recommended for Phase 2)
- Better context understanding
- Can preserve technical terms
- Can maintain Markdown formatting
- More expensive but higher quality
- Example prompt:
  ```
  Translate the following technical documentation to Urdu.
  Preserve all Markdown formatting, code blocks, and technical terms like "ROS 2", "NVIDIA Isaac".
  
  [Content here]
  ```

### Caching Strategy
1. **First Request**: Translate and cache in database/Redis
2. **Subsequent Requests**: Serve from cache (instant response)
3. **Cache Key**: `translation:{chapter_id}:{language_code}`
4. **Cache Invalidation**: When chapter content is updated

### Markdown Preservation
- Translate text content only
- Preserve Markdown syntax (headings, lists, code blocks, links)
- Keep code snippets in original language
- Maintain technical terminology

---

## Implementation Status

### ✅ Completed
- User model has `selected_language` field
- Database migrations

### ⏳ To Be Implemented
- FastAPI router for translation endpoints
- Integration with Google Translate API or Gemini
- Translation caching mechanism (Redis or PostgreSQL)
- Markdown-aware translation logic
- Frontend language selector UI
- Real-time translation toggle
- Static site generation with pre-translated pages (optional)

---

## Integration Points

### Frontend (Docusaurus)
1. **Language Selector**: Dropdown in navbar
2. **Per-Chapter Translation**: Button to view translated version
3. **Persistent Preference**: Save to localStorage or user profile
4. **Chatbot Integration**: Translate bot responses based on user language

### Chatbot Service
- Translate chatbot responses based on user's `selected_language`
- Handle multilingual queries (detect language and respond accordingly)

### Personalization Service
- Combine translation with personalization (translated + adapted content)

---

## Priority Languages

1. **Urdu (ur)** - Primary target for Pakistan
2. **Arabic (ar)** - Wide audience in Middle East
3. **Spanish (es)** - Large global audience
4. **French (fr)** - Africa and Europe
5. **Chinese (zh)** - Large tech audience

---

## Performance Considerations

1. **Pre-translation**: For static content, pre-translate during build time
2. **Lazy Loading**: Translate chapters on-demand, not all at once
3. **CDN Caching**: Cache translated content on CDN for faster delivery
4. **Partial Translation**: Start with intro and first chapter, expand gradually

---

## Quality Assurance

1. **Human Review**: Have native speakers review translations
2. **Technical Accuracy**: Verify technical terms are correct
3. **User Feedback**: Allow users to report translation issues
4. **Version Control**: Track translation versions with content updates

---

## Cost Estimation

### Google Translate API
- Average chapter: ~5000 words = ~30,000 characters
- 3 chapters × 30,000 chars = 90,000 characters
- 5 languages = 450,000 characters total
- Cost: ~$9 per full book translation (one-time)

### Google Gemini (Alternative)
- Same 450,000 characters
- Gemini 2.5 Flash: ~$0.30 per million input tokens
- Estimated cost: ~$15-20 per full book translation
- Better quality, higher cost

---

## Notes

- No translation functionality exists in current implementation
- The static site (Docusaurus) doesn't support i18n currently
- User language preference field exists but is not used
- Chatbot responses are always in English
- Consider using Docusaurus i18n plugin for static translations
