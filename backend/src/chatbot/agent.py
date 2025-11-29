"""
RAG Agent Module using OpenAI Agents SDK with Gemini
"""
import os
from pathlib import Path
from typing import List, Dict, Any
from dotenv import load_dotenv
from openai import AsyncOpenAI
from agents import Agent, OpenAIChatCompletionsModel, Runner, set_tracing_disabled, function_tool
from qdrant_client import QdrantClient
from fastembed import TextEmbedding

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


@function_tool
def search_book_content(query: str, limit: int = 5) -> str:
    """Search book content using vector search to find relevant information.

    Args:
        query: The search query to find relevant book content.
        limit: Maximum number of results to return (default: 5).
    """
    try:
        # Generate embedding for the query
        model = get_embedding_model()
        query_embedding = list(model.embed([query]))[0]

        # Search in Qdrant
        search_results = qdrant_client.query_points(
            collection_name=QDRANT_COLLECTION_NAME,
            query=query_embedding.tolist(),
            limit=limit
        ).points

        if not search_results:
            return "No relevant content found in the book."

        # Format results as a readable string
        formatted_results = []
        for i, result in enumerate(search_results, 1):
            text = result.payload.get("text", "")
            source = result.payload.get("source", "unknown")
            score = result.score

            formatted_results.append(
                f"[Result {i}] (Relevance: {score:.2f})\n"
                f"Source: {source}\n"
                f"Content: {text}\n"
            )

        return "\n---\n".join(formatted_results)

    except Exception as e:
        return f"Error searching book content: {str(e)}"


# Create the RAG agent
rag_agent = Agent(
    name="Book Assistant",
    instructions="""You are a helpful AI assistant that answers questions about the book content.

Your capabilities:
1. You have access to a search_book_content tool that retrieves relevant information from the book
2. Always use the search tool to find relevant information before answering
3. Base your answers on the retrieved content
4. If the retrieved content doesn't contain enough information, say so clearly
5. Cite sources when possible (mention the source file)
6. Be concise but comprehensive
7. If asked about selected text, focus specifically on that text

Guidelines:
- Use the search tool for general questions about the book
- Provide accurate, helpful responses based on the retrieved context
- If you're not sure, admit it rather than making up information
- Format your responses clearly and professionally
""",
    model=OpenAIChatCompletionsModel(
        model="gemini-2.5-flash-lite",
        openai_client=gemini_client
    ),
    tools=[search_book_content]
)


async def query_agent(question: str, context: str = None) -> Dict[str, Any]:
    """
    Query the RAG agent with a question.

    Args:
        question: The user's question
        context: Optional context (e.g., selected text)

    Returns:
        Dictionary with answer and metadata
    """
    try:
        # Prepare the query
        if context:
            prompt = f"""Context (selected text from the book):
{context}

Question: {question}

Please answer based on the provided context."""
        else:
            prompt = question

        # Run the agent
        result = await Runner.run(
            rag_agent,
            prompt
        )

        return {
            "answer": result.final_output,
            "success": True
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