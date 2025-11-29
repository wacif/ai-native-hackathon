# API Contract: Authentication Service

## Status: ⚠️ **NOT YET IMPLEMENTED**

## Planned Technology Stack
- **Authentication**: JWT (JSON Web Tokens)
- **Password Hashing**: bcrypt or argon2
- **Database**: PostgreSQL (models already exist)
- **Framework**: FastAPI with OAuth2

## Database Models: ✅ **CREATED**

The User model is already defined in `backend/src/models/user.py`:

```python
class User(Base):
    __tablename__ = "users"
    
    id: UUID
    username: str (unique, indexed)
    email: str (unique, indexed)
    password_hash: str
    software_background: str (optional)
    hardware_background: str (optional)
    personalization_preferences: JSONB (optional)
    selected_language: str (default: "en")
    created_at: DateTime
    updated_at: DateTime
```

## Base Path: `/auth` (Planned)

---

## Planned Endpoints

### 1. User Signup

-   **Endpoint**: `POST /auth/signup`
-   **Description**: Registers a new user with the system.
-   **Authentication**: None
-   **Request Body** (JSON):
    ```json
    {
        "username": "string",
        "email": "user@example.com",
        "password": "string",
        "software_background": "string",  // Optional: User's software experience level
        "hardware_background": "string"   // Optional: User's hardware experience level
    }
    ```
-   **Response** (201 Created, JSON):
    ```json
    {
        "message": "User registered successfully",
        "user_id": "uuid"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Invalid input (e.g., missing required fields, weak password).
    -   `409 Conflict`: Email or username already exists.

### 2. User Signin

-   **Endpoint**: `POST /auth/signin`
-   **Description**: Authenticates a user and returns a JWT access token.
-   **Authentication**: None
-   **Request Body** (JSON):
    ```json
    {
        "email": "user@example.com",
        "password": "string"
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "message": "Login successful",
        "access_token": "string",        // JWT token
        "token_type": "bearer",
        "expires_in": 3600               // Token expiry in seconds
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Missing email or password.
    -   `401 Unauthorized`: Invalid credentials.

### 3. Get Current User Profile

-   **Endpoint**: `GET /auth/me`
-   **Description**: Retrieves the profile information of the authenticated user.
-   **Authentication**: Required - Bearer token in `Authorization` header.
-   **Request Headers**:
    ```
    Authorization: Bearer <access_token>
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "user_id": "uuid",
        "username": "string",
        "email": "user@example.com",
        "software_background": "string",
        "hardware_background": "string",
        "personalization_preferences": {},
        "selected_language": "string",
        "created_at": "2024-01-01T00:00:00Z",
        "updated_at": "2024-01-01T00:00:00Z"
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Missing or invalid authentication token.

### 4. Update User Profile

-   **Endpoint**: `PATCH /auth/me`
-   **Description**: Updates the authenticated user's profile information.
-   **Authentication**: Required - Bearer token in `Authorization` header.
-   **Request Body** (JSON):
    ```json
    {
        "username": "string",             // Optional
        "software_background": "string",  // Optional
        "hardware_background": "string",  // Optional
        "selected_language": "string",    // Optional: "en", "ur", etc.
        "personalization_preferences": {} // Optional: JSON object
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "message": "Profile updated successfully",
        "user": { /* Updated user object */ }
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Invalid input data.
    -   `401 Unauthorized`: Missing or invalid authentication token.
    -   `409 Conflict`: Username already taken.

### 5. Refresh Token

-   **Endpoint**: `POST /auth/refresh`
-   **Description**: Refreshes an expired access token.
-   **Authentication**: Required - Refresh token.
-   **Request Body** (JSON):
    ```json
    {
        "refresh_token": "string"
    }
    ```
-   **Response** (200 OK, JSON):
    ```json
    {
        "access_token": "string",
        "token_type": "bearer",
        "expires_in": 3600
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Invalid or expired refresh token.

---

## Implementation Status

### ✅ Completed
- User model with all required fields
- Database migrations
- AuthenticationException and AuthorizationException error classes

### ⏳ To Be Implemented
- FastAPI auth router (`/auth` endpoints)
- Password hashing utilities (bcrypt/argon2)
- JWT token generation and validation
- OAuth2 password flow
- Token refresh mechanism
- Rate limiting for auth endpoints
- Email verification (optional)
- Password reset flow (optional)

---

## Integration Points

### Chatbot Service
Once implemented, the chatbot endpoints (`/query`, `/query-selection`) will:
- Accept optional `user_id` from the authenticated token
- Log interactions to `ChatbotInteraction` model
- Support future personalization features

### Personalization Service
Authentication is a prerequisite for:
- Content personalization based on user background
- Saving user preferences
- Translation preferences

---

## Security Considerations

1. **Password Requirements**: Minimum 8 characters, mix of letters, numbers, and special characters
2. **Token Expiry**: Access tokens expire after 1 hour, refresh tokens after 7 days
3. **Password Hashing**: Use bcrypt with appropriate salt rounds (12+) or argon2
4. **HTTPS Only**: All auth endpoints must use HTTPS in production
5. **CORS**: Restrict to frontend domain in production
6. **Rate Limiting**: Implement rate limiting on login/signup endpoints

---

## Notes

- The current chatbot implementation accepts `user_id` but doesn't validate or use it.
- Database schema supports user features, but no API endpoints exist yet.
- Frontend (Docusaurus) doesn't have login UI components yet.
