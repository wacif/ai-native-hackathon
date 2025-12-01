# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

Get the project running in under 10 minutes.

## Prerequisites

- **Node.js** v20+ with npm
- **Python** 3.11+
- **Git**

### External Services (free tiers available)
- [Neon](https://neon.tech) - Serverless PostgreSQL
- [Qdrant Cloud](https://qdrant.tech) - Vector database
- [Google AI Studio](https://aistudio.google.com) - Gemini API key

## Quick Start

### 1. Clone & Install Frontend

```bash
git clone git@github.com:wacif/ai-native-hackathon.git
cd ai-native-hackathon
npm install
```

### 2. Install Backend

```bash
cd backend
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Configure Environment

Create `backend/.env`:

```env
# Database (Neon Serverless Postgres)
DATABASE_URL=postgresql://user:pass@host:5432/db

# Vector Database (Qdrant Cloud)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# AI (Google Gemini)
GEMINI_API_KEY=your-gemini-api-key

# Auth (generate a secure random string)
JWT_SECRET_KEY=your-secret-key-minimum-32-characters
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60
REFRESH_TOKEN_EXPIRE_DAYS=7
```

### 4. Initialize Database

```bash
cd backend
source .venv/bin/activate
alembic upgrade head
```

### 5. Ingest Book Content

```bash
python -m src.rag.ingestion
```

### 6. Start Servers

**Terminal 1 - Backend:**
```bash
cd backend
source .venv/bin/activate
uvicorn src.main:app --host 0.0.0.0 --port 8000
```

**Terminal 2 - Frontend:**
```bash
npm start
```

### 7. Access the App

- **Frontend**: http://localhost:3000/ai-native-hackathon/
- **Backend API**: http://localhost:8000
- **API Docs**: http://localhost:8000/docs

## Test Features

1. **Signup**: Create an account with your preferences
2. **Chatbot**: Ask questions about Physical AI topics
3. **Text Selection**: Highlight text and ask specific questions
4. **Personalize**: Click "Personalize for me" on any chapter
5. **Translate**: Click "اردو" to view Urdu translation

## Common Issues

### Port already in use
```bash
# Kill process on port 8000
lsof -ti:8000 | xargs kill -9
# Kill process on port 3000
lsof -ti:3000 | xargs kill -9
```

### Database connection error
- Verify `DATABASE_URL` in `.env`
- Check Neon dashboard for connection status

### Qdrant connection error
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check Qdrant Cloud dashboard

### Module not found
```bash
cd backend
pip install -r requirements.txt
```

## Development Workflow

```bash
# Frontend hot reload
npm start

# Backend hot reload
uvicorn src.main:app --reload

# Run linting
cd backend && ruff check src/

# Run type checking
npm run typecheck
```

## Deployment

### Frontend (GitHub Pages)
Automatic deployment via GitHub Actions on push to `main` branch.

### Backend
Deploy to any Python-compatible platform (Railway, Render, Fly.io, etc.)

## Need Help?

- Check `/specs` for detailed specifications
- Review `/history/prompts` for development context
- Open an issue on GitHub
