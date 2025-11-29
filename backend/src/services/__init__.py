"""
Services for the Physical AI & Humanoid Robotics Textbook backend.
"""

from .embedding_service import generate_embedding, generate_embeddings, get_embedding_model
from .qdrant_service import QdrantService, qdrant_service

__all__ = [
    "generate_embedding",
    "generate_embeddings",
    "get_embedding_model",
    "QdrantService",
    "qdrant_service",
]

