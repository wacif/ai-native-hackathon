# Physical AI & Humanoid Robotics Textbook

An interactive, AI-native textbook on Physical AI & Humanoid Robotics with personalized learning, RAG chatbot, and multi-language support.

## 🌟 Features

### 📚 Interactive Textbook
- Built with Docusaurus 3.9.2 (React-based static site generator)
- 4 comprehensive chapters on Physical AI & Robotics
- Clean, responsive design with dark mode support

### 🤖 AI-Powered RAG Chatbot
- Context-aware chatbot powered by Google Gemini
- Text selection support for targeted queries
- Chapter-aware responses with automatic page context
- Markdown rendering for formatted answers
- **Personalized responses** based on user profile

### 👤 User Authentication & Personalization
- JWT-based authentication (signup, signin, refresh tokens)
- User profiles with:
  - Operating system preference (Windows/macOS/Linux)
  - Programming language preferences
  - Learning style (concise/detailed/visual/example-driven)
  - Prior knowledge areas
  - Learning goals
  - Industry focus
- **Chapter personalization**: One-click content adaptation based on profile
- Cached personalized content to avoid redundant LLM calls

### 🌐 Multi-Language Support
- **Urdu translation** for all chapters
- Language toggle button (English/اردو) on chapter pages
- Static pre-translated files for instant switching
- Auth-gated translation (logged-in users only)

### 💻 Code Syntax Highlighting
- VS Code Dark+ theme colors
- 12 programming languages supported:
  - Python, JavaScript/TypeScript, Bash/Shell
  - JSON, YAML, HTML/XML, CSS/SCSS
  - SQL, C/C++, Rust, Go

## 🏗️ Architecture

### Frontend
- **Framework**: Docusaurus 3.9.2 (React 19)
- **Styling**: CSS Modules, custom theming
- **State**: React Context (AuthContext)
- **Deployment**: GitHub Pages

### Backend
- **API**: FastAPI (Python 3.11)
- **Database**: Neon Serverless PostgreSQL (SQLAlchemy)
- **Vector Store**: Qdrant Cloud
- **AI Model**: Google Gemini 2.5 Flash
- **Embeddings**: FastEmbed (BAAI/bge-small-en-v1.5)
- **Auth**: JWT with bcrypt password hashing

## 📚 Course Content

| Chapter | Topic |
|---------|-------|
| Introduction | Overview of Physical AI & Robotics |
| Chapter 1 | The Rise of Embodied AI |
| Chapter 2 | Robot Hardware & Sensors |
| Chapter 3 | Software & Simulation |

Future topics planned:
- ROS 2 (Robot Operating System) Fundamentals
- Robot Simulation with Gazebo & Unity
- NVIDIA Isaac Platform
- Vision-Language-Action (VLA) Models

## 🚀 Getting Started

### Prerequisites

- Node.js v20+ and npm
- Python 3.11+
- Neon Serverless Postgres account
- Qdrant Cloud account
- Google Gemini API key

### Quick Setup

```bash
# Clone repository
git clone git@github.com:wacif/ai-native-hackathon.git
cd ai-native-hackathon

# Frontend setup
npm install
npm start  # http://localhost:3000/ai-native-hackathon/

# Backend setup (new terminal)
cd backend
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your credentials

# Run migrations
alembic upgrade head

# Ingest book content
python -m src.rag.ingestion

# Start backend
uvicorn src.main:app --host 0.0.0.0 --port 8000
```

### Environment Variables

Create `backend/.env`:
```env
DATABASE_URL=postgresql://user:pass@host:5432/db
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
GEMINI_API_KEY=your-gemini-api-key
JWT_SECRET_KEY=your-secret-key-min-32-chars
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60
REFRESH_TOKEN_EXPIRE_DAYS=7
```

## 📁 Project Structure

```
.
├── .github/workflows/     # CI/CD pipelines
│   ├── deploy.yml         # Docusaurus → GitHub Pages
│   └── backend_ci.yml     # Backend linting & tests
├── backend/
│   ├── src/
│   │   ├── api/           # Auth endpoints
│   │   ├── chatbot/       # RAG agent logic
│   │   ├── config/        # DB, auth, Qdrant config
│   │   ├── models/        # SQLAlchemy models
│   │   ├── rag/           # Content ingestion
│   │   ├── services/      # Business logic
│   │   ├── utils/         # Logging, errors
│   │   └── main.py        # FastAPI app
│   ├── migrations/        # Alembic migrations
│   └── requirements.txt
├── docs/
│   └── physical-ai/       # Textbook chapters (MDX)
├── static/
│   └── translations/ur/   # Urdu translations
├── src/
│   ├── components/
│   │   ├── Auth/          # Login, Signup, UserButton
│   │   └── Chatbot/       # RAG chatbot UI
│   ├── context/           # AuthContext
│   └── theme/             # Docusaurus overrides
│       ├── Root.tsx       # Auth provider wrapper
│       └── DocItem/       # Personalize/Translate buttons
├── specs/                 # Design specifications
├── history/               # Prompt history records
├── docusaurus.config.ts
└── package.json
```

## 🔌 API Endpoints

### Authentication
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/auth/signup` | Register new user |
| POST | `/auth/signin` | Login (returns JWT) |
| POST | `/auth/refresh` | Refresh access token |
| GET | `/auth/me` | Get current user profile |
| PATCH | `/auth/me` | Update user profile |

### Chatbot & Personalization
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/query` | RAG chatbot query |
| POST | `/query-selection` | Query with text selection |
| POST | `/personalize-chapter` | Get personalized chapter content |

### Health & Info
| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | API info |
| GET | `/health` | Health check |
| GET | `/collections` | List Qdrant collections |

## 🛠️ Development Commands

### Frontend
```bash
npm start          # Dev server (hot reload)
npm run build      # Production build
npm run serve      # Serve production build
npm run typecheck  # TypeScript check
npm run clear      # Clear Docusaurus cache
```

### Backend
```bash
uvicorn src.main:app --reload              # Dev server
alembic revision --autogenerate -m "msg"   # Create migration
alembic upgrade head                       # Apply migrations
python -m src.rag.ingestion                # Re-ingest content
ruff check src/                            # Lint code
```

## 🎯 Project Status

### ✅ Completed
- **Phase 1-2**: Infrastructure & Database Setup
- **Phase 3**: Core Textbook Content (4 chapters)
- **Phase 4**: RAG Chatbot with Gemini
- **Phase 5**: User Authentication (JWT)
- **Phase 6**: Content Personalization
- **Phase 7**: Urdu Translation

### 🔜 Upcoming
- Phase N: Polish & Cross-Cutting Concerns
  - Performance optimization
  - Security hardening
  - Comprehensive testing

## 📝 Development Philosophy

This project follows **Spec-Driven Development (SDD)** principles:
- AI-assisted development using Claude
- Comprehensive specifications in `/specs`
- Prompt history records in `/history`
- Modular, maintainable architecture

## 🔗 Links

- **Live Site**: [wacif.github.io/ai-native-hackathon](https://wacif.github.io/ai-native-hackathon/)
- **Specifications**: See `/specs` directory
- **Prompt History**: See `/history` directory

## 📄 License

MIT License - See LICENSE file for details.

---

Built with ❤️ for the AI-Native Hackathon
