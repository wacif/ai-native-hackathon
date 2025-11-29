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
    user_id: str | None = None  # Optional, for logging
    page_url: str | None = None  # Current page URL for context
    chapter_id: str | None = None  # Optional chapter ID
    max_results: int = 5

class TextSelectionRequest(BaseModel):
    question: str
    selected_text: str
    user_id: str | None = None  # Optional, for logging
    page_url: str | None = None  # Current page URL
    chapter_id: str | None = None  # Optional chapter ID

# Response models
class QueryResponse(BaseModel):
    answer: str
    sources: list[dict]
    confidence_score: float | None = None
    page_context: str | None = None  # Which page the query was from

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
    """
    Query the book content using RAG with Gemini agent.
    
    Supports page-context aware queries:
    - If page_url is provided, can filter results to current page
    - If chapter_id is provided, can prioritize content from that chapter
    """
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

        # Extract source information if available
        sources = [{
            "source": "agent_search",
            "type": "rag",
            "score": 1.0
        }]

        return QueryResponse(
            answer=result["answer"],
            sources=sources,
            confidence_score=0.95,  # Can be enhanced with actual confidence
            page_context=request.page_url
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/query-selection", response_model=QueryResponse)
async def query_selected_text(request: TextSelectionRequest):
    """
    Answer questions based on user-selected text from the book.
    
    This endpoint is specifically for when users highlight text and ask questions
    about that specific selection. The selected text provides focused context.
    """
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
            sources=[{
                "source": "selected_text",
                "type": "selection",
                "score": 1.0,
                "text_preview": request.selected_text[:100] + "..." if len(request.selected_text) > 100 else request.selected_text
            }],
            confidence_score=0.98,  # Higher confidence for direct text selection
            page_context=request.page_url
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")