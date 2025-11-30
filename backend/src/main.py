import os
import hashlib
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
from qdrant_client import QdrantClient
from dotenv import load_dotenv
from sqlalchemy.orm import Session
from sqlalchemy import select
from .chatbot.agent import query_agent, qdrant_client, UserProfile
from .api.auth import router as auth_router, get_current_user
from .models.user import User
from .models.personalized_content import PersonalizedContent
from .config.db import get_db

load_dotenv()

app = FastAPI(
    title="Physical AI Textbook API",
    description="AI-powered textbook with RAG chatbot, authentication, and personalization",
    version="3.0.0"
)

# Register routers
app.include_router(auth_router)

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

# Optional auth for chatbot - allows both authenticated and unauthenticated requests
security = HTTPBearer(auto_error=False)

def build_user_profile(user: Optional[User]) -> Optional[UserProfile]:
    """Build UserProfile from database user model"""
    if not user:
        return None
    
    return UserProfile(
        programming_languages=user.programming_languages,
        operating_system=user.operating_system,
        learning_goals=user.learning_goals,
        preferred_explanation_style=user.preferred_explanation_style,
        prior_knowledge=user.prior_knowledge,
        industry=user.industry
    )

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

class PersonalizeChapterRequest(BaseModel):
    chapter_id: str
    chapter_content: str  # The markdown content of the chapter
    force_refresh: bool = False  # Force regeneration even if cached

class PersonalizeChapterResponse(BaseModel):
    personalized_content: str
    cached: bool
    chapter_id: str

# Response models
class QueryResponse(BaseModel):
    answer: str
    sources: list[dict]
    confidence_score: float | None = None
    page_context: str | None = None  # Which page the query was from

@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook API",
        "status": "running",
        "endpoints": {
            "health": "/health",
            "query": "/query",
            "query_selection": "/query-selection",
            "personalize_chapter": "/personalize-chapter",
            "collections": "/collections",
            "auth": {
                "signup": "/auth/signup",
                "signin": "/auth/signin",
                "refresh": "/auth/refresh",
                "me": "/auth/me"
            }
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
async def query_book(
    request: QueryRequest,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
):
    """
    Query the book content using RAG with Gemini agent.
    
    Supports page-context aware queries:
    - If page_url is provided, can filter results to current page
    - If chapter_id is provided, can prioritize content from that chapter
    
    If an auth token is provided, responses will be personalized based on user profile.
    """
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=500, detail="Gemini API key not configured")

    try:
        # Try to get user profile if authenticated
        user_profile = None
        if credentials:
            try:
                user = await get_current_user(credentials)
                user_profile = build_user_profile(user)
            except Exception:
                # If token is invalid, continue without personalization
                pass
        
        # Use the agent to answer the question
        # The agent will automatically use the search_book_content tool
        result = await query_agent(
            request.question, 
            chapter_id=request.chapter_id,
            user_profile=user_profile
        )

        if not result["success"]:
            raise HTTPException(
                status_code=500,
                detail=result.get("error", "Error processing query")
            )

        # Extract source information if available
        sources = [{
            "source": "agent_search",
            "type": "rag",
            "score": 1.0,
            "personalized": result.get("personalized", False)
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
async def query_selected_text(
    request: TextSelectionRequest,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
):
    """
    Answer questions based on user-selected text from the book.
    
    This endpoint is specifically for when users highlight text and ask questions
    about that specific selection. The selected text provides focused context.
    
    If an auth token is provided, responses will be personalized based on user profile.
    """
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=500, detail="Gemini API key not configured")

    try:
        # Try to get user profile if authenticated
        user_profile = None
        if credentials:
            try:
                user = await get_current_user(credentials)
                user_profile = build_user_profile(user)
            except Exception:
                # If token is invalid, continue without personalization
                pass
        
        # Use the agent with the selected text as context
        result = await query_agent(
            question=request.question,
            context=request.selected_text,
            chapter_id=request.chapter_id,
            user_profile=user_profile
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
                "text_preview": request.selected_text[:100] + "..." if len(request.selected_text) > 100 else request.selected_text,
                "personalized": result.get("personalized", False)
            }],
            confidence_score=0.98,  # Higher confidence for direct text selection
            page_context=request.page_url
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


# Required auth security for personalization
auth_security = HTTPBearer()


async def get_user_from_credentials(
    credentials: HTTPAuthorizationCredentials,
    db: Session
) -> User:
    """Extract user from HTTPBearer credentials."""
    from .services.auth_service import AuthService
    from .utils.errors import AuthenticationException
    
    token = credentials.credentials
    try:
        payload = AuthService.decode_token(token)
        user_id = payload.get("sub")
        token_type = payload.get("type")

        if not user_id or token_type != "access":
            raise HTTPException(status_code=401, detail="Invalid token")

        # Use AuthService to get user (sync operation)
        user = AuthService.get_user_by_id(db, user_id)
        
        if not user:
            raise HTTPException(status_code=401, detail="User not found")

        return user
    except AuthenticationException as e:
        raise HTTPException(status_code=401, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=401, detail=f"Could not validate credentials: {str(e)}")


def build_personalization_prompt(user: User, chapter_content: str) -> str:
    """Build a prompt to personalize chapter content based on user profile"""
    profile_parts = []
    
    if user.operating_system:
        os_name = user.operating_system.capitalize()
        profile_parts.append(f"- Operating System: {os_name}")
        if user.operating_system == "windows":
            profile_parts.append("  (Adapt commands to PowerShell/CMD, use backslashes for paths)")
        elif user.operating_system == "macos":
            profile_parts.append("  (Use bash/zsh commands, reference Homebrew for package management)")
        elif user.operating_system == "linux":
            profile_parts.append("  (Use bash commands, reference apt/dnf as appropriate)")
    
    if user.programming_languages:
        langs = ", ".join(user.programming_languages)
        profile_parts.append(f"- Preferred Programming Languages: {langs}")
        profile_parts.append(f"  (When showing code examples, prefer {user.programming_languages[0]})")
    
    if user.preferred_explanation_style:
        style = user.preferred_explanation_style
        profile_parts.append(f"- Explanation Style: {style}")
        if style == "concise":
            profile_parts.append("  (Keep explanations brief, use bullet points)")
        elif style == "detailed":
            profile_parts.append("  (Provide thorough explanations with background context)")
        elif style == "visual":
            profile_parts.append("  (Use diagrams and visual representations where possible)")
        elif style == "example-driven":
            profile_parts.append("  (Lead with practical examples before theory)")
    
    if user.prior_knowledge:
        knowledge = ", ".join(user.prior_knowledge)
        profile_parts.append(f"- Prior Knowledge: {knowledge}")
        profile_parts.append(f"  (You can skip basic explanations of: {knowledge})")
    
    if user.learning_goals:
        goals = ", ".join(user.learning_goals)
        profile_parts.append(f"- Learning Goals: {goals}")
        profile_parts.append(f"  (Emphasize content relevant to: {goals})")
    
    if user.industry:
        profile_parts.append(f"- Industry/Focus: {user.industry}")
        profile_parts.append(f"  (Relate examples to {user.industry} when relevant)")
    
    profile_section = "\n".join(profile_parts) if profile_parts else "No specific preferences"
    
    return f"""You are an expert educational content adapter. Your task is to personalize the following textbook chapter content based on the user's profile.

USER PROFILE:
{profile_section}

INSTRUCTIONS:
1. Adapt the content to match the user's operating system for any commands or file paths
2. Use code examples in the user's preferred programming language when applicable
3. Adjust the explanation depth based on their preferred style
4. Skip or condense explanations for topics they already know
5. Emphasize content relevant to their learning goals
6. Add industry-specific examples when relevant
7. Maintain the original structure (headings, sections) but adapt the content
8. Keep markdown formatting intact
9. Do NOT add any meta-commentary - just output the personalized content

ORIGINAL CHAPTER CONTENT:
{chapter_content}

PERSONALIZED CHAPTER CONTENT:"""


@app.post("/personalize-chapter", response_model=PersonalizeChapterResponse)
async def personalize_chapter(
    request: PersonalizeChapterRequest,
    credentials: HTTPAuthorizationCredentials = Depends(auth_security),
    db: Session = Depends(get_db)
):
    """
    Personalize chapter content based on user's profile.
    
    This endpoint takes the original chapter content and returns a personalized version
    adapted to the user's operating system, programming languages, learning style, etc.
    
    Results are cached in the database to avoid regenerating for the same content.
    """
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=500, detail="Gemini API key not configured")

    try:
        # Get authenticated user
        user = await get_user_from_credentials(credentials, db)
        
        # Check if user has any personalization preferences
        has_preferences = any([
            user.programming_languages,
            user.operating_system,
            user.learning_goals,
            user.preferred_explanation_style,
            user.prior_knowledge,
            user.industry
        ])
        
        if not has_preferences:
            # No personalization needed, return original content
            return PersonalizeChapterResponse(
                personalized_content=request.chapter_content,
                cached=False,
                chapter_id=request.chapter_id
            )
        
        # Compute hash of original content for cache lookup
        content_hash = hashlib.sha256(request.chapter_content.encode()).hexdigest()
        
        # Check cache if not forcing refresh
        if not request.force_refresh:
            cached = db.query(PersonalizedContent).filter(
                PersonalizedContent.user_id == user.id,
                PersonalizedContent.chapter_id == request.chapter_id,
                PersonalizedContent.original_content_hash == content_hash
            ).first()
            
            if cached:
                return PersonalizeChapterResponse(
                    personalized_content=cached.personalized_content,
                    cached=True,
                    chapter_id=request.chapter_id
                )
        
        # Generate personalized content using Gemini
        from openai import AsyncOpenAI
        
        gemini_client = AsyncOpenAI(
            api_key=GEMINI_API_KEY,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )
        
        prompt = build_personalization_prompt(user, request.chapter_content)
        
        response = await gemini_client.chat.completions.create(
            model="gemini-2.5-flash-lite",
            messages=[
                {"role": "user", "content": prompt}
            ],
            max_tokens=8000,
            temperature=0.3  # Lower temperature for more consistent output
        )
        
        personalized_content = response.choices[0].message.content
        
        # Cache the result
        # First, check for existing entry for this user/chapter
        existing_record = db.query(PersonalizedContent).filter(
            PersonalizedContent.user_id == user.id,
            PersonalizedContent.chapter_id == request.chapter_id
        ).first()
        
        if existing_record:
            existing_record.original_content_hash = content_hash
            existing_record.personalized_content = personalized_content
        else:
            new_record = PersonalizedContent(
                user_id=user.id,
                chapter_id=request.chapter_id,
                original_content_hash=content_hash,
                personalized_content=personalized_content
            )
            db.add(new_record)
        
        db.commit()
        
        return PersonalizeChapterResponse(
            personalized_content=personalized_content,
            cached=False,
            chapter_id=request.chapter_id
        )
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error personalizing content: {str(e)}")