# Physical AI & Humanoid Robotics Textbook

An interactive, AI-native textbook on Physical AI & Humanoid Robotics with an embedded RAG chatbot for enhanced learning.

## 🌟 Features

- **Interactive Textbook**: Built with Docusaurus, featuring 4 comprehensive chapters on Physical AI & Robotics
- **AI-Powered RAG Chatbot**: Context-aware chatbot that answers questions about the content
  - Text selection support for targeted queries
  - Chapter-aware responses with automatic page context
  - Markdown rendering for clean, formatted answers
- **Modern Tech Stack**: FastAPI backend, Qdrant vector database, OpenAI Agents SDK with Gemini

## 🏗️ Architecture

### Frontend
- **Framework**: Docusaurus (React-based static site generator)
- **Features**: Responsive design, integrated chatbot, text selection
- **Deployment**: GitHub Pages

### Backend
- **API**: FastAPI (Python)
- **Database**: Neon Serverless PostgreSQL (via SQLAlchemy)
- **Vector Store**: Qdrant Cloud
- **AI**: OpenAI Agents SDK + Google Gemini
- **Embeddings**: FastEmbed

## 📚 Course Content

The textbook currently includes:
- **Introduction**: Overview of Physical AI & Robotics
- **Chapter 1**: Robot Sensors and Perception
- **Chapter 2**: Robot Actuators and Control Systems
- **Chapter 3**: Autonomous Navigation and SLAM

Future topics will cover:
- ROS 2 (Robot Operating System) Fundamentals
- Robot Simulation with Gazebo & Unity
- NVIDIA Isaac Platform
- Vision-Language-Action (VLA) Models

## 🚀 Getting Started

### Prerequisites

- Node.js (v18 or higher)
- Python 3.9+
- PostgreSQL (or Neon Serverless Postgres account)
- Qdrant Cloud account

### Frontend Setup

1. **Install dependencies**:
```bash
npm install
```

2. **Start development server**:
```bash
npm start
```
The site will be available at `http://localhost:3000/ai-native-hackathon/`

3. **Build for production**:
```bash
npm run build
```

4. **Preview production build**:
```bash
npm run serve
```

### Backend Setup

1. **Navigate to backend directory**:
```bash
cd backend
```

2. **Create virtual environment**:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. **Install dependencies**:
```bash
pip install -r requirements.txt
```

4. **Set up environment variables**:
Create a `.env` file in the `backend` directory:
```env
DATABASE_URL=postgresql://user:password@host:5432/dbname
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
GEMINI_API_KEY=your-gemini-api-key
```

5. **Run database migrations**:
```bash
cd src
alembic upgrade head
```

6. **Ingest book content into Qdrant**:
```bash
python -m rag.ingestion
```

7. **Start the FastAPI server**:
```bash
cd ..
uvicorn src.main:app --reload
```
The API will be available at `http://localhost:8000`

## 🧪 Testing the RAG Chatbot

1. Ensure both frontend and backend servers are running
2. Navigate to any chapter in the textbook
3. Open the chatbot widget (bottom-right corner)
4. Try these test scenarios:
   - Ask "What chapter am I on?" - should respond with current chapter
   - Select text and ask a question - should use selected context
   - Ask about specific topics - should return relevant, markdown-formatted answers

## 📁 Project Structure

```
.
├── backend/
│   ├── src/
│   │   ├── chatbot/          # RAG agent logic
│   │   ├── models/           # Database models
│   │   ├── rag/              # Ingestion & vector search
│   │   └── main.py           # FastAPI app
│   ├── migrations/           # Alembic migrations
│   └── requirements.txt
├── docs/
│   └── physical-ai/          # Textbook chapters
├── src/
│   ├── components/
│   │   └── Chatbot/          # React chatbot component
│   └── theme/
│       └── Root.tsx          # Page context tracking
├── docusaurus.config.ts      # Docusaurus configuration
└── package.json
```

## 🛠️ Development Commands

### Frontend
```bash
npm start              # Start dev server
npm run build          # Production build
npm run serve          # Serve production build
npm run clear          # Clear cache
npm run deploy         # Deploy to GitHub Pages
```

### Backend
```bash
uvicorn src.main:app --reload              # Start dev server
alembic revision --autogenerate -m "msg"   # Create migration
alembic upgrade head                       # Apply migrations
python -m rag.ingestion                    # Re-ingest content
```

## 🎯 Current Status

### Completed Phases
- ✅ **Phase 1-2**: Infrastructure & Database Setup
- ✅ **Phase 3**: RAG Backend with Qdrant
- ✅ **Phase 4**: Interactive Chatbot Frontend
  - Chapter/page awareness
  - Text selection integration
  - Markdown rendering
  - Enhanced context handling

### Upcoming Phases
- 🔜 **Phase 5**: User Authentication & Personalization
- 🔜 **Phase 6**: Advanced Features (Code examples, diagrams)
- 🔜 **Phase 7**: Urdu Translation (i18n)

## 📝 Development Philosophy

This project follows Spec-Driven Development (SDD) principles:
- AI-assisted development using Claude Code
- Comprehensive specifications in `/specs`
- Prompt history records in `/history`
- Modular, maintainable architecture

## 🤝 Contributing

This is an educational project developed as part of the AI-Native Hackathon. For details on the development process, see the `/specs` and `/history` directories.

## 📄 License

MIT License - See LICENSE file for details

## 🔗 Links

- **Live Site**: [GitHub Pages](https://your-username.github.io/ai-native-hackathon/)
- **Documentation**: See `/specs` for detailed specifications
- **Prompt History**: See `/history` for development records
