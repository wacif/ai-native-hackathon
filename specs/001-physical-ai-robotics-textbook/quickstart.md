# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

This guide provides a quick overview of how to set up and run the "Physical AI & Humanoid Robotics Textbook" project locally.

## 1. Prerequisites

Ensure you have the following installed:

*   **Node.js** (LTS version) and **npm** or **Yarn** (for Docusaurus frontend)
*   **Python 3.9+** and **pip** (for FastAPI backend)
*   **Git**
*   **Docker** (optional, for easier database/Qdrant setup)

## 2. Project Setup

1.  **Clone the repository**:
    ```bash
    git clone git@github.com:wacif/ai-native-hackathon.git
    cd ai-native-hackathon
    ```

2.  **Frontend (Docusaurus) Setup**:
    ```bash
    npm install # or yarn install
    ```

3.  **Backend (FastAPI) Setup**:
    ```bash
    cd backend
    pip install -r requirements.txt
    cd ..
    ```

4.  **Database (Neon Serverless Postgres)**:
    *   Create a Neon project and obtain your connection string.
    *   Set the connection string as an environment variable (e.g., `DATABASE_URL`).

5.  **Vector Database (Qdrant Cloud)**:
    *   Create a Qdrant Cloud account and get your API key and URL.
    *   Set these as environment variables (e.g., `QDRANT_URL`, `QDRANT_API_KEY`).

6.  **OpenAI API Key** (for RAG chatbot):
    *   Obtain an OpenAI API key.
    *   Set it as an environment variable (e.g., `OPENAI_API_KEY`).

7.  **Better-Auth Credentials** (optional, if implementing authentication):
    *   Set up an account with Better-Auth.
    *   Configure necessary environment variables for client ID, secret, etc.

## 3. Running the Project

### 1. Start the FastAPI Backend

From the project root, navigate to the `backend` directory and run:

```bash
cd backend
uvicorn main:app --reload
```

The backend API will be available at `http://localhost:8000` (or configured port).

### 2. Start the Docusaurus Frontend

From the project root, run:

```bash
npm start # or yarn start
```

The Docusaurus development server will start, and the textbook will be accessible in your browser, typically at `http://localhost:3000`.

## 4. Deployment to GitHub Pages

The Docusaurus project is configured for deployment to GitHub Pages. Ensure your `docusaurus.config.js` is correctly configured with your repository name. The deployment will typically be handled via GitHub Actions upon pushing to the `main` (or `gh-pages`) branch.

## 5. Next Steps

*   Populate the `docs/` directory with your textbook content.
*   Implement the RAG chatbot logic within the `backend/` services.
*   Integrate authentication, personalization, and translation features as per the specification.
