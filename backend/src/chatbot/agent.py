"""
RAG Agent Module using OpenAI Agents SDK with Gemini
"""
import os
from pathlib import Path
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from openai import AsyncOpenAI
from agents import Agent, OpenAIChatCompletionsModel, Runner, set_tracing_disabled, function_tool
from qdrant_client import QdrantClient
from fastembed import TextEmbedding
from dataclasses import dataclass

# Load .env file from backend directory
backend_dir = Path(__file__).resolve().parent.parent.parent
load_dotenv(dotenv_path=backend_dir / '.env')

# Environment variables
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = "book_content"

# Initialize Gemini client with OpenAI-compatible API
gemini_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    prefer_grpc=False,
)

# Initialize embedding model (cached globally)
embedding_model = None

def get_embedding_model():
    """Lazy load embedding model"""
    global embedding_model
    if embedding_model is None:
        embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
    return embedding_model

# Disable tracing for cleaner output
set_tracing_disabled(disabled=True)


@dataclass
class UserProfile:
    """User profile for personalization"""
    programming_languages: Optional[List[str]] = None
    operating_system: Optional[str] = None
    learning_goals: Optional[List[str]] = None
    preferred_explanation_style: Optional[str] = None
    prior_knowledge: Optional[List[str]] = None
    industry: Optional[str] = None


@function_tool
def search_book_content(query: str, chapter_id: str = None, limit: int = 5) -> str:
    """Search book content using vector search to find relevant information.

    Args:
        query: The search query to find relevant book content.
        chapter_id: Optional chapter ID to filter results (e.g., "chapter1", "intro").
        limit: Maximum number of results to return (default: 5).
    """
    try:
        # Generate embedding for the query
        model = get_embedding_model()
        query_embedding = list(model.embed([query]))[0]

        # Build filter if chapter_id is provided
        query_filter = None
        if chapter_id:
            from qdrant_client.models import Filter, FieldCondition, MatchValue
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="chapter_id",
                        match=MatchValue(value=chapter_id)
                    )
                ]
            )

        # Search in Qdrant with optional filter
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_embedding.tolist(),
            query_filter=query_filter,
            limit=limit
        )

        if not search_results:
            if chapter_id:
                return f"No relevant content found in chapter '{chapter_id}'."
            return "No relevant content found in the book."

        # Format results as a readable string
        formatted_results = []
        for i, result in enumerate(search_results, 1):
            text = result.payload.get("text", "")
            source = result.payload.get("source", "unknown")
            chapter = result.payload.get("chapter_id", "unknown")
            page_url = result.payload.get("page_url", "")
            score = result.score

            formatted_results.append(
                f"[Result {i}] (Relevance: {score:.2f})\n"
                f"Chapter: {chapter}\n"
                f"Source: {source}\n"
                f"Page URL: {page_url}\n"
                f"Content: {text}\n"
            )

        return "\n---\n".join(formatted_results)

    except Exception as e:
        return f"Error searching book content: {str(e)}"


def build_personalized_instructions(user_profile: Optional[UserProfile] = None) -> str:
    """Build agent instructions based on user profile"""
    base_instructions = """You are a helpful AI assistant that answers questions about the Physical AI & Humanoid Robotics textbook.

Your capabilities:
1. You have access to a search_book_content tool that retrieves relevant information from the book
2. Always use the search tool to find relevant information before answering technical questions
3. If a chapter_id is provided in the context (e.g., [Current Chapter: chapter1]), you know which chapter the user is currently viewing
4. When asked "what chapter am I on?" or similar questions, respond based on the [Current Chapter: X] information in the prompt
5. Base your answers on the retrieved content from the search tool
6. If the retrieved content doesn't contain enough information, say so clearly
7. Cite sources when possible (mention the chapter and source file)
8. Be concise but comprehensive
9. If asked about selected text, focus specifically on that text

Guidelines:
- For questions about current location/chapter: Answer directly from the [Current Chapter: X] context without using the search tool
- For technical questions: Use the search_book_content tool, and when chapter_id is available, pass it to filter results
- When user is viewing a specific chapter, prioritize content from that chapter in search results
- Provide accurate, helpful responses based on the retrieved context
- If you're not sure, admit it rather than making up information
- Format your responses clearly and professionally
- Be friendly and encouraging to learners
"""
    
    if not user_profile:
        return base_instructions
    
    # Build personalization section
    personalization_parts = ["\n\n[USER PROFILE - Personalize your responses accordingly]"]
    
    if user_profile.operating_system:
        os_name = user_profile.operating_system.capitalize()
        personalization_parts.append(f"- Operating System: {os_name}")
        personalization_parts.append(f"  → When providing terminal commands, shell scripts, or installation instructions, use {os_name}-specific syntax and paths.")
        if user_profile.operating_system == "windows":
            personalization_parts.append("  → Use PowerShell or CMD commands. Use backslashes for paths. Reference Windows-specific tools.")
        elif user_profile.operating_system == "macos":
            personalization_parts.append("  → Use bash/zsh commands. Use forward slashes. Reference Homebrew for package management.")
        elif user_profile.operating_system == "linux":
            personalization_parts.append("  → Use bash commands. Use forward slashes. Reference apt/dnf/pacman as appropriate.")
    
    if user_profile.programming_languages:
        langs = ", ".join(user_profile.programming_languages)
        personalization_parts.append(f"- Preferred Programming Languages: {langs}")
        personalization_parts.append(f"  → When providing code examples, prefer using {user_profile.programming_languages[0]} when possible.")
        personalization_parts.append(f"  → The user is familiar with: {langs}")
    
    if user_profile.preferred_explanation_style:
        style = user_profile.preferred_explanation_style
        personalization_parts.append(f"- Preferred Explanation Style: {style}")
        if style == "concise":
            personalization_parts.append("  → Keep explanations brief and to the point. Use bullet points. Avoid lengthy prose.")
        elif style == "detailed":
            personalization_parts.append("  → Provide thorough, comprehensive explanations. Include background context and examples.")
        elif style == "visual":
            personalization_parts.append("  → Use ASCII diagrams, code visualizations, and structured formatting. Break down complex concepts visually.")
        elif style == "example-driven":
            personalization_parts.append("  → Lead with practical examples before theory. Show code/usage first, then explain.")
    
    if user_profile.prior_knowledge:
        knowledge = ", ".join(user_profile.prior_knowledge)
        personalization_parts.append(f"- Prior Knowledge: {knowledge}")
        personalization_parts.append(f"  → You can assume familiarity with: {knowledge}. Skip basic explanations of these topics.")
    
    if user_profile.learning_goals:
        goals = ", ".join(user_profile.learning_goals)
        personalization_parts.append(f"- Learning Goals: {goals}")
        personalization_parts.append(f"  → Emphasize content relevant to: {goals}")
    
    if user_profile.industry:
        personalization_parts.append(f"- Industry/Focus Area: {user_profile.industry}")
        personalization_parts.append(f"  → Relate examples and applications to {user_profile.industry} when relevant.")
    
    personalization_parts.append("\n[END USER PROFILE]")
    
    return base_instructions + "\n".join(personalization_parts)


def create_rag_agent(user_profile: Optional[UserProfile] = None) -> Agent:
    """Create a RAG agent with optional personalization"""
    instructions = build_personalized_instructions(user_profile)
    
    return Agent(
        name="Book Assistant",
        instructions=instructions,
        model=OpenAIChatCompletionsModel(
            model="gemini-2.5-flash-lite",
            openai_client=gemini_client
        ),
        tools=[search_book_content]
    )


# Default agent (no personalization)
rag_agent = create_rag_agent()


async def query_agent(
    question: str, 
    context: str = None, 
    chapter_id: str = None,
    user_profile: Optional[UserProfile] = None
) -> Dict[str, Any]:
    """
    Query the RAG agent with a question.

    Args:
        question: The user's question
        context: Optional context (e.g., selected text)
        chapter_id: Optional chapter ID for filtering results (e.g., "chapter1", "intro")
        user_profile: Optional user profile for personalization

    Returns:
        Dictionary with answer and metadata
    """
    try:
        # Create personalized agent if user profile provided, otherwise use default
        agent = create_rag_agent(user_profile) if user_profile else rag_agent
        
        # Prepare the query
        if context:
            prompt = f"""Context (selected text from the book):
{context}

Question: {question}

Please answer based on the provided context."""
        else:
            prompt = question
        
        # Add chapter context if provided
        if chapter_id:
            # Map chapter_id to human-readable names
            chapter_names = {
                "intro": "Introduction to Physical AI & Humanoid Robotics",
                "chapter1": "Chapter 1: Robot Sensors and Perception",
                "chapter2": "Chapter 2: Actuators and Movement Control",
                "chapter3": "Chapter 3: AI Algorithms for Robotics"
            }
            chapter_name = chapter_names.get(chapter_id, f"Chapter: {chapter_id}")
            
            prompt = f"""[CURRENT LOCATION]
You are currently on: {chapter_name}
Chapter ID: {chapter_id}

If the user asks where they are or what chapter they're on, tell them they're on "{chapter_name}".

{prompt}

When searching for information, use the chapter_id parameter ('{chapter_id}') in the search_book_content tool to prioritize content from this chapter."""

        # Run the agent
        result = await Runner.run(
            agent,
            prompt
        )

        return {
            "answer": result.final_output,
            "success": True,
            "chapter_id": chapter_id,
            "personalized": user_profile is not None
        }

    except Exception as e:
        return {
            "answer": f"Error processing query: {str(e)}",
            "success": False,
            "error": str(e)
        }


async def test_agent():
    """Test function to verify agent is working"""
    print("🤖 Testing RAG Agent with Gemini...")
    print("-" * 70)

    test_questions = [
        "What is Docusaurus?",
        "How do I create a new page?",
        "How to deploy the site?"
    ]

    for question in test_questions:
        print(f"\n❓ Question: {question}")
        result = await query_agent(question)

        if result["success"]:
            print(f"✅ Answer: {result['answer']}")
        else:
            print(f"❌ Error: {result.get('error', 'Unknown error')}")
        print("-" * 70)


if __name__ == "__main__":
    import asyncio
    asyncio.run(test_agent())