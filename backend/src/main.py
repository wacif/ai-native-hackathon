import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
from dotenv import load_dotenv
from .chatbot.agent import query_agent, qdrant_client

load_dotenv()

app = FastAPI(
    title="RAG Chatbot API",
    description="AI-powered chatbot with RAG capabilities using Gemini and Qdrant",
    version="2.0.0"
)

# CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your Docusaurus domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_COLLECTION_NAME = "book_content"

# Request models
class QueryRequest(BaseModel):
    question: str
    max_results: int = 5

class TextSelectionRequest(BaseModel):
    question: str
    selected_text: str

# Response models
class QueryResponse(BaseModel):
    answer: str
    sources: list[dict]

@app.get("/")
async def root():
    return {
        "message": "RAG Chatbot API for PIAIC Hackathon",
        "status": "running",
        "endpoints": {
            "health": "/health",
            "query": "/query",
            "query_selection": "/query-selection",
            "collections": "/collections"
        }
    }

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    try:
        collections = qdrant_client.get_collections()
        qdrant_status = "connected"
        collection_exists = any(c.name == QDRANT_COLLECTION_NAME for c in collections.collections)
    except Exception as e:
        qdrant_status = f"error: {str(e)}"
        collection_exists = False

    return {
        "status": "healthy",
        "qdrant": qdrant_status,
        "collection_exists": collection_exists,
        "gemini_configured": GEMINI_API_KEY is not None,
        "agent": "gemini-2.5-flash-lite"
    }

@app.get("/collections")
async def list_collections():
    """List all Qdrant collections."""
    try:
        collections = qdrant_client.get_collections()
        return {
            "collections": [c.name for c in collections.collections]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to fetch collections: {str(e)}")

@app.post("/query", response_model=QueryResponse)
async def query_book(request: QueryRequest):
    """Query the book content using RAG with Gemini agent."""
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=500, detail="Gemini API key not configured")

    try:
        # Use the agent to answer the question
        # The agent will automatically use the search_book_content tool
        result = await query_agent(request.question)

        if not result["success"]:
            raise HTTPException(
                status_code=500,
                detail=result.get("error", "Error processing query")
            )

        # Note: The agent handles search internally via tools
        # We return a simplified response for now
        return QueryResponse(
            answer=result["answer"],
            sources=[{
                "source": "agent_search",
                "type": "rag",
                "score": 1.0
            }]
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/query-selection", response_model=QueryResponse)
async def query_selected_text(request: TextSelectionRequest):
    """Answer questions based on user-selected text from the book."""
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=500, detail="Gemini API key not configured")

    try:
        # Use the agent with the selected text as context
        result = await query_agent(
            question=request.question,
            context=request.selected_text
        )

        if not result["success"]:
            raise HTTPException(
                status_code=500,
                detail=result.get("error", "Error processing query")
            )

        return QueryResponse(
            answer=result["answer"],
            sources=[{"source": "selected_text", "type": "selection", "score": 1.0}]
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")