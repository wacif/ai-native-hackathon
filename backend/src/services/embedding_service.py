"""
Embedding service for generating embeddings using OpenAI or other providers.
"""

from typing import List
from dotenv import load_dotenv
from fastembed import TextEmbedding
from ..utils.logger import logger
from ..utils.errors import ExternalServiceException

load_dotenv()

# Global embedding model instance (lazy loaded)
_embedding_model = None


def get_embedding_model() -> TextEmbedding:
    """
    Get or initialize the embedding model.
    Uses BAAI/bge-small-en-v1.5 for fast and efficient embeddings.
    
    Returns:
        TextEmbedding: Initialized embedding model
    """
    global _embedding_model
    if _embedding_model is None:
        try:
            logger.info("Initializing embedding model: BAAI/bge-small-en-v1.5")
            _embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
            logger.info("Embedding model initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize embedding model: {str(e)}")
            raise ExternalServiceException(
                message="Failed to initialize embedding model",
                service_name="FastEmbed",
                details={"error": str(e)}
            )
    return _embedding_model


def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text.
    
    Args:
        text: Text to generate embedding for
        
    Returns:
        List[float]: Embedding vector
        
    Raises:
        ExternalServiceException: If embedding generation fails
    """
    try:
        model = get_embedding_model()
        embeddings = list(model.embed([text]))
        if not embeddings:
            raise ValueError("No embeddings generated")
        return embeddings[0].tolist()
    except Exception as e:
        logger.error(f"Failed to generate embedding: {str(e)}")
        raise ExternalServiceException(
            message="Failed to generate embedding",
            service_name="FastEmbed",
            details={"error": str(e), "text_length": len(text)}
        )


def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts.
    
    Args:
        texts: List of texts to generate embeddings for
        
    Returns:
        List[List[float]]: List of embedding vectors
        
    Raises:
        ExternalServiceException: If embedding generation fails
    """
    try:
        model = get_embedding_model()
        embeddings = list(model.embed(texts))
        return [emb.tolist() for emb in embeddings]
    except Exception as e:
        logger.error(f"Failed to generate embeddings: {str(e)}")
        raise ExternalServiceException(
            message="Failed to generate embeddings",
            service_name="FastEmbed",
            details={"error": str(e), "num_texts": len(texts)}
        )

