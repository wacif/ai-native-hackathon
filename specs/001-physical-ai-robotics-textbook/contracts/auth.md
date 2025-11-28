# API Contract: Authentication Service

## Base Path: `/auth`

### 1. User Signup

-   **Endpoint**: `POST /signup`
-   **Description**: Registers a new user with the system.
-   **Request Body** (JSON):
    ```json
    {
        "username": "string",
        "email": "user@example.com",
        "password": "string",
        "software_background": "string",
        "hardware_background": "string"
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
    -   `400 Bad Request`: Invalid input (e.g., missing fields, weak password).
    -   `409 Conflict`: Email or username already exists.

### 2. User Signin

-   **Endpoint**: `POST /signin`
-   **Description**: Authenticates a user and returns an authentication token.
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
        "access_token": "string",
        "token_type": "bearer"
    }
    ```
-   **Error Responses**:
    -   `400 Bad Request`: Invalid credentials.

### 3. Get Current User Profile

-   **Endpoint**: `GET /me`
-   **Description**: Retrieves the profile information of the authenticated user.
-   **Authentication**: Requires valid access token in `Authorization` header.
-   **Request Body**: None
-   **Response** (200 OK, JSON):
    ```json
    {
        "user_id": "uuid",
        "username": "string",
        "email": "user@example.com",
        "software_background": "string",
        "hardware_background": "string",
        "personalization_preferences": {},
        "selected_language": "string"
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: Missing or invalid authentication token.
